package pathing

import (
	"math/bits"
)

// JPS implements Jump Point Search, an optimization of A* for uniform-cost grids.
// It dramatically reduces the number of nodes expanded by "jumping" over
// intermediate nodes along straight lines, only stopping at jump points
// (nodes with forced neighbors) or the goal.
//
// JPS always uses 8-directional (diagonal) movement.
// It treats any non-zero GridLayer cost as passable (uniform cost).
// If you need weighted pathfinding, use AStar instead.
//
// Once created, you should re-use it to build paths.
// Do not throw the instance away after building the path once.
//
// Bit Prune reference article https://cloud.tencent.com/developer/article/1748191.
type JPS struct {
	frontier *minheap[jpsCoord]
	// costmap tracks g-scores for jump points only (used for stale-entry detection).
	costmap *coordMap
	// pathmap stores the direction each cell was first reached from (write-once).
	// Used by constructPath for path reconstruction.
	pathmap *pathDirMap
	// parentDir tracks the best-known parent direction for each jump point.
	// Updated whenever a better cost is found. Used by identifySuccessors
	// for neighbor pruning, separate from pathmap to avoid reconstruction cycles.
	parentDir *pathDirMap
	bitGrid   jpsBitGrid

	// Fallback tracking: updated during jump scans for partial-path support.
	fallbackCoord GridCoord
	fallbackDist  int
	fallbackCost  int
	fallbackSet   bool
}

// JPSConfig is a NewJPS() function parameter.
type JPSConfig struct {
	// NumCols and NumRows are size hints for the JPS constructor.
	// Grid.NumCols() and Grid.NumRows() methods will come in handy to initialize these.
	// If you keep them at 0, the max amount of the working space will be allocated.
	NumCols uint
	NumRows uint
}

type jpsCoord struct {
	Coord GridCoord
	Cost  int32
}

// Scan event types for jumpBit* functions.
const (
	scanWall = 0
	scanJump = 1
	scanGoal = 2
)

// Bit=1 means blocked. Lazy per-row/column construction with generation tracking.
// The bitgrid is cached across BuildPath calls when the grid hasn't changed.
type jpsBitGrid struct {
	rowData     []uint64 // rowData[y*wordsPerRow + w]
	colData     []uint64 // colData[x*wordsPerCol + w]
	rowGen      []uint32
	colGen      []uint32
	generation  uint32
	wordsPerRow int
	wordsPerCol int
	numCols     int
	numRows     int

	// Cache tracking: skip bitgrid rebuild when grid is unchanged.
	cachedGrid   *Grid
	cachedLayer  GridLayer
	cachedMutGen uint32
}

func newJPSBitGrid(numCols, numRows int) jpsBitGrid {
	wpr := (numCols + 63) / 64
	wpc := (numRows + 63) / 64
	return jpsBitGrid{
		rowData:     make([]uint64, numRows*wpr),
		colData:     make([]uint64, numCols*wpc),
		rowGen:      make([]uint32, numRows),
		colGen:      make([]uint32, numCols),
		wordsPerRow: wpr,
		wordsPerCol: wpc,
		numCols:     numCols,
		numRows:     numRows,
	}
}

// resetForGrid checks if the bitgrid cache is still valid for the given grid
// and layer. If the grid hasn't been mutated, all previously-built rows/cols
// remain valid and no generation increment is needed. Otherwise, the generation
// is bumped to force lazy rebuilds.
func (bg *jpsBitGrid) resetForGrid(g *Grid, l GridLayer) {
	if bg.cachedGrid == g && bg.cachedLayer == l && bg.cachedMutGen == g.mutGen {
		return
	}
	bg.generation++
	bg.cachedGrid = g
	bg.cachedLayer = l
	bg.cachedMutGen = g.mutGen
}

// ensureRow lazily builds the bitboard for row y.
func (bg *jpsBitGrid) ensureRow(g *Grid, y int, l GridLayer) {
	if bg.rowGen[y] == bg.generation {
		return
	}
	bg.rowGen[y] = bg.generation
	base := y * bg.wordsPerRow
	for w := 0; w < bg.wordsPerRow; w++ {
		bg.rowData[base+w] = 0
	}
	numCols := bg.numCols
	uy := uint(y)
	for x := 0; x < numCols; x++ {
		if g.getCellCost(uint(x), uy, l) == 0 {
			bg.rowData[base+x/64] |= 1 << uint(x%64)
		}
	}
	// Mark OOB bits in last word as blocked.
	rem := numCols % 64
	if rem != 0 {
		lastW := base + bg.wordsPerRow - 1
		bg.rowData[lastW] |= ^((1 << uint(rem)) - 1)
	}
}

// ensureCol lazily builds the bitboard for column x.
func (bg *jpsBitGrid) ensureCol(g *Grid, x int, l GridLayer) {
	if bg.colGen[x] == bg.generation {
		return
	}
	bg.colGen[x] = bg.generation
	base := x * bg.wordsPerCol
	for w := 0; w < bg.wordsPerCol; w++ {
		bg.colData[base+w] = 0
	}
	numRows := bg.numRows
	ux := uint(x)
	for y := 0; y < numRows; y++ {
		if g.getCellCost(ux, uint(y), l) == 0 {
			bg.colData[base+y/64] |= 1 << uint(y%64)
		}
	}
	rem := numRows % 64
	if rem != 0 {
		lastW := base + bg.wordsPerCol - 1
		bg.colData[lastW] |= ^((1 << uint(rem)) - 1)
	}
}

// rowWord returns word w of row y's bitboard.
func (bg *jpsBitGrid) rowWord(y, w int) uint64 {
	return bg.rowData[y*bg.wordsPerRow+w]
}

// colWord returns word w of column x's bitboard.
func (bg *jpsBitGrid) colWord(x, w int) uint64 {
	return bg.colData[x*bg.wordsPerCol+w]
}

// NewJPS creates a ready-to-use JPS object.
func NewJPS(config JPSConfig) *JPS {
	if config.NumCols == 0 {
		config.NumCols = gridMapSide
	}
	if config.NumRows == 0 {
		config.NumRows = gridMapSide
	}

	coordMapCols := int(config.NumCols)
	coordMapRows := int(config.NumRows)

	return &JPS{
		frontier:  newMinheap[jpsCoord](32),
		pathmap:   newPathDirMap(coordMapCols, coordMapRows),
		parentDir: newPathDirMap(coordMapCols, coordMapRows),
		costmap:   newCoordMap(coordMapCols, coordMapRows),
		bitGrid:   newJPSBitGrid(coordMapCols, coordMapRows),
	}
}

// BuildPath attempts to find a path between the two coordinates.
// JPS treats 0 as "blocked" and any other value as "passable" (uniform cost).
// It always uses 8-directional movement.
func (jps *JPS) BuildPath(g *Grid, from, to GridCoord, l GridLayer) BuildPathResult {
	var result BuildPathResult
	if from == to {
		result.Finish = to
		return result
	}

	frontier := jps.frontier
	frontier.Reset()

	pathmap := jps.pathmap
	pathmap.Reset()

	jps.parentDir.Reset()

	costmap := jps.costmap
	costmap.Reset()

	jps.bitGrid.resetForGrid(g, l)

	frontier.Push(0, jpsCoord{Coord: from})
	costmap.Set(costmap.packCoord(from), 0)

	jps.fallbackSet = false
	foundPath := false
	for !frontier.IsEmpty() {
		current := frontier.Pop()
		currentKey := costmap.packCoord(current.Coord)
		currentCost, ok := costmap.Get(currentKey)
		if ok && current.Cost != int32(currentCost) {
			continue
		}
		if !ok {
			// Defensive: should not happen since all frontier entries go through
			// pushJumpPoint which calls costmap.Set. Use frontier cost as fallback.
			currentCost = uint32(current.Cost)
		}

		if current.Coord == to {
			result.Steps = constructPath(from, to, pathmap)
			result.Finish = to
			result.Cost = int(current.Cost)
			foundPath = true
			break
		}

		jps.updateFallback(current.Coord, int(currentCost), to)
		jps.identifySuccessors(g, current.Coord, from, int(currentCost), to, l)
	}

	if !foundPath {
		result.Steps = constructPath(from, jps.fallbackCoord, pathmap)
		result.Finish = jps.fallbackCoord
		result.Cost = result.Steps.Len()
		result.Partial = true
	}

	return result
}

func (jps *JPS) updateFallback(pos GridCoord, cost int, to GridCoord) {
	dist := chebyshev(pos.X, pos.Y, to.X, to.Y)
	if !jps.fallbackSet || dist < jps.fallbackDist || (dist == jps.fallbackDist && cost < jps.fallbackCost) {
		jps.fallbackCoord = pos
		jps.fallbackDist = dist
		jps.fallbackCost = cost
		jps.fallbackSet = true
	}
}

func (jps *JPS) identifySuccessors(g *Grid, current, from GridCoord, currentCost int, to GridCoord, l GridLayer) {
	var dirs [8]Direction
	ndirs := 0

	if current == from {
		for _, d := range neighborDiagonal {
			dirs[ndirs] = d.Direction
			ndirs++
		}
	} else {
		k := jps.parentDir.packCoord(current)
		pd, _ := jps.parentDir.Get(k)
		ndirs = jpsPrunedDirs(g, current, pd, l, &dirs)
	}

	for i := 0; i < ndirs; i++ {
		dir := dirs[i]
		if isDiagonalDir(dir) {
			// Move one step diagonally, then run pruned diagonal scan.
			next := current.Move(dir)
			if g.GetCellCost(next, l) == 0 {
				continue
			}
			jps.pathmap.setIfAbsent(jps.pathmap.packCoord(next), dir)
			if next == to {
				jps.pushJumpPoint(next, currentCost+1, to, dir)
				continue
			}
			jps.jumpDiagPrune(g, next, dir, to, l, currentCost+1)
		} else {
			jp := jps.jumpCardBit(g, current, dir, to, l, currentCost)
			if jp.X < 0 {
				continue
			}
			jumpDist := intabs(jp.X-current.X) + intabs(jp.Y-current.Y)
			jps.pushJumpPoint(jp, currentCost+jumpDist, to, dir)
		}
	}
}

// pushJumpPoint adds a jump point to the frontier if it improves the known cost.
// Pathmap is NOT written here — callers (recordCardinalScan, identifySuccessors,
// jumpDiagPrune) already write it before calling pushJumpPoint.
func (jps *JPS) pushJumpPoint(jp GridCoord, newCost int, to GridCoord, dir Direction) {
	jpk := jps.costmap.packCoord(jp)
	oldCost, exists := jps.costmap.Get(jpk)
	if exists && uint32(newCost) >= oldCost {
		return
	}
	jps.costmap.Set(jpk, uint32(newCost))
	// Always update parentDir for correct pruning in identifySuccessors.
	jps.parentDir.Set(jps.parentDir.packCoord(jp), dir)
	priority := newCost + chebyshev(jp.X, jp.Y, to.X, to.Y)
	jps.frontier.Push(priority, jpsCoord{Coord: jp, Cost: int32(newCost)})
}

// jumpCardBit performs a cardinal jump using bitboard scanning.
// It moves one step first, then scans using bit operations.
// Returns the jump point, or {-1,-1} if none found.
func (jps *JPS) jumpCardBit(g *Grid, pos GridCoord, dir Direction, to GridCoord, l GridLayer, baseCost int) GridCoord {
	next := pos.Move(dir)
	if g.GetCellCost(next, l) == 0 {
		return GridCoord{X: -1, Y: -1}
	}
	jps.pathmap.setIfAbsent(jps.pathmap.packCoord(next), dir)
	if next == to {
		return next
	}

	switch dir {
	case DirRight:
		return jps.jumpBitRight(g, next.X, next.Y, to, l, baseCost+1)
	case DirLeft:
		return jps.jumpBitLeft(g, next.X, next.Y, to, l, baseCost+1)
	case DirDown:
		return jps.jumpBitDown(g, next.X, next.Y, to, l, baseCost+1)
	case DirUp:
		return jps.jumpBitUp(g, next.X, next.Y, to, l, baseCost+1)
	}

	return GridCoord{X: -1, Y: -1}
}

// Note: jumpBitRight, jumpBitLeft, jumpBitDown, and jumpBitUp are intentionally
// duplicated and unrolled for performance to avoid branching and function call overhead.
// Keep their logic synchronized when making changes.
//
// jumpBitRight scans rightward from (x,y) using row bitboards.
func (jps *JPS) jumpBitRight(g *Grid, x, y int, to GridCoord, l GridLayer, cost int) GridCoord {
	bg := &jps.bitGrid
	bg.ensureRow(g, y, l)
	hasUp := y > 0
	hasDown := y < bg.numRows-1
	if hasUp {
		bg.ensureRow(g, y-1, l)
	}
	if hasDown {
		bg.ensureRow(g, y+1, l)
	}

	startWord := x / 64
	startBit := uint(x % 64)

	// Hoist loop-invariant goal computation.
	goalW := to.X / 64
	goalBit := to.X % 64
	goalActive := to.Y == y && to.X > x &&
		(goalW > startWord || goalBit >= int(startBit))

	for w := startWord; w < bg.wordsPerRow; w++ {
		row := bg.rowWord(y, w)

		var forced uint64
		if hasUp {
			rowUp := bg.rowWord(y-1, w)
			nextBitUp := uint64(0)
			if w+1 < bg.wordsPerRow {
				nextBitUp = bg.rowWord(y-1, w+1) & 1
			}
			// Detect forced neighbors: shift the upper row by 1 to align diagonal cells.
			// A forced neighbor exists if the adjacent cell (rowUp) is blocked (1)
			// but the diagonal cell (^upShifted) is free (0).
			upShifted := (rowUp >> 1) | (nextBitUp << 63)
			forced = rowUp & ^upShifted
		}
		if hasDown {
			rowDn := bg.rowWord(y+1, w)
			nextBitDn := uint64(0)
			if w+1 < bg.wordsPerRow {
				nextBitDn = bg.rowWord(y+1, w+1) & 1
			}
			// Similar forced neighbor detection for the lower row.
			dnShifted := (rowDn >> 1) | (nextBitDn << 63)
			forced |= rowDn & ^dnShifted
		}

		jumpCandidates := forced & ^row

		var mask uint64
		if w == startWord {
			mask = ^uint64(0) << startBit
		} else {
			mask = ^uint64(0)
		}
		jumpCandidates &= mask
		wallBits := row & mask

		wallDist := 64
		if wallBits != 0 {
			wallDist = bits.TrailingZeros64(wallBits)
		}
		jumpDist := 64
		if jumpCandidates != 0 {
			jumpDist = bits.TrailingZeros64(jumpCandidates)
		}

		goalDist := 64
		if goalActive && goalW == w {
			goalDist = goalBit
		}

		minDist := wallDist
		minType := scanWall
		if jumpDist < minDist {
			minDist = jumpDist
			minType = scanJump
		}
		if goalDist < minDist {
			minDist = goalDist
			minType = scanGoal
		}

		if minDist < 64 {
			resultX := w*64 + minDist
			if minType == scanWall {
				if resultX-1 >= x {
					jps.recordCardinalScan(resultX-1, x, y, DirRight, cost, to)
				}
				return GridCoord{X: -1, Y: -1}
			}
			jps.recordCardinalScan(resultX, x, y, DirRight, cost, to)
			return GridCoord{X: resultX, Y: y}
		}

		// No event in this word — continue without per-word pathmap writes.
		// The final recordCardinalScan (wall/jump/goal) covers the full range.
	}
	// Fell off grid edge.
	endX := bg.numCols - 1
	if endX >= x {
		jps.recordCardinalScan(endX, x, y, DirRight, cost, to)
	}
	return GridCoord{X: -1, Y: -1}
}

// Note: intentionally duplicated for performance. Keep synchronized with other jumpBit* functions.
//
// jumpBitLeft scans leftward from (x,y) using row bitboards.
func (jps *JPS) jumpBitLeft(g *Grid, x, y int, to GridCoord, l GridLayer, cost int) GridCoord {
	bg := &jps.bitGrid
	bg.ensureRow(g, y, l)
	hasUp := y > 0
	hasDown := y < bg.numRows-1
	if hasUp {
		bg.ensureRow(g, y-1, l)
	}
	if hasDown {
		bg.ensureRow(g, y+1, l)
	}

	goalOnRow := to.Y == y && to.X < x

	startWord := x / 64
	startBit := uint(x % 64)

	// Hoist loop-invariant goal computation.
	goalW := to.X / 64
	goalDistVal := 63 - (to.X % 64)

	for w := startWord; w >= 0; w-- {
		row := bg.rowWord(y, w)

		var forced uint64
		if hasUp {
			rowUp := bg.rowWord(y-1, w)
			prevBitUp := uint64(0)
			if w > 0 {
				prevBitUp = (bg.rowWord(y-1, w-1) >> 63) & 1
			}
			upShifted := (rowUp << 1) | prevBitUp
			forced = rowUp & ^upShifted
		}
		if hasDown {
			rowDn := bg.rowWord(y+1, w)
			prevBitDn := uint64(0)
			if w > 0 {
				prevBitDn = (bg.rowWord(y+1, w-1) >> 63) & 1
			}
			dnShifted := (rowDn << 1) | prevBitDn
			forced |= rowDn & ^dnShifted
		}

		jumpCandidates := forced & ^row

		var mask uint64
		if w == startWord {
			mask = (1 << (startBit + 1)) - 1
		} else {
			mask = ^uint64(0)
		}
		jumpCandidates &= mask
		wallBits := row & mask

		wallDist := 64
		if wallBits != 0 {
			wallDist = bits.LeadingZeros64(wallBits)
		}
		jumpDist := 64
		if jumpCandidates != 0 {
			jumpDist = bits.LeadingZeros64(jumpCandidates)
		}

		goalDist := 64
		if goalOnRow && goalW == w {
			goalDist = goalDistVal
		}

		minDist := wallDist
		minType := scanWall
		if jumpDist < minDist {
			minDist = jumpDist
			minType = scanJump
		}
		if goalDist < minDist {
			minDist = goalDist
			minType = scanGoal
		}

		if minDist < 64 {
			resultX := w*64 + (63 - minDist)
			if minType == scanWall {
				if resultX+1 <= x {
					jps.recordCardinalScan(resultX+1, x, y, DirLeft, cost, to)
				}
				return GridCoord{X: -1, Y: -1}
			}
			jps.recordCardinalScan(resultX, x, y, DirLeft, cost, to)
			return GridCoord{X: resultX, Y: y}
		}

		// No event in this word — continue without per-word pathmap writes.
	}
	// Fell off grid edge.
	if x >= 0 {
		jps.recordCardinalScan(0, x, y, DirLeft, cost, to)
	}
	return GridCoord{X: -1, Y: -1}
}

// Note: intentionally duplicated for performance. Keep synchronized with other jumpBit* functions.
//
// jumpBitDown scans downward from (x,y) using column bitboards.
func (jps *JPS) jumpBitDown(g *Grid, x, y int, to GridCoord, l GridLayer, cost int) GridCoord {
	bg := &jps.bitGrid
	bg.ensureCol(g, x, l)
	hasLeft := x > 0
	hasRight := x < bg.numCols-1
	if hasLeft {
		bg.ensureCol(g, x-1, l)
	}
	if hasRight {
		bg.ensureCol(g, x+1, l)
	}

	startWord := y / 64
	startBit := uint(y % 64)

	// Hoist loop-invariant goal computation.
	goalW := to.Y / 64
	goalBit := to.Y % 64
	goalActive := to.X == x && to.Y > y &&
		(goalW > startWord || goalBit >= int(startBit))

	for w := startWord; w < bg.wordsPerCol; w++ {
		col := bg.colWord(x, w)
		var forced uint64
		if hasLeft {
			colL := bg.colWord(x-1, w)
			nextBitL := uint64(0)
			if w+1 < bg.wordsPerCol {
				nextBitL = bg.colWord(x-1, w+1) & 1
			}
			lShifted := (colL >> 1) | (nextBitL << 63)
			forced = colL & ^lShifted
		}
		if hasRight {
			colR := bg.colWord(x+1, w)
			nextBitR := uint64(0)
			if w+1 < bg.wordsPerCol {
				nextBitR = bg.colWord(x+1, w+1) & 1
			}
			rShifted := (colR >> 1) | (nextBitR << 63)
			forced |= colR & ^rShifted
		}

		jumpCandidates := forced & ^col

		var mask uint64
		if w == startWord {
			mask = ^uint64(0) << startBit
		} else {
			mask = ^uint64(0)
		}
		jumpCandidates &= mask
		wallBits := col & mask

		wallDist := 64
		if wallBits != 0 {
			wallDist = bits.TrailingZeros64(wallBits)
		}
		jumpDist := 64
		if jumpCandidates != 0 {
			jumpDist = bits.TrailingZeros64(jumpCandidates)
		}

		goalDist := 64
		if goalActive && goalW == w {
			goalDist = goalBit
		}

		minDist := wallDist
		minType := scanWall
		if jumpDist < minDist {
			minDist = jumpDist
			minType = scanJump
		}
		if goalDist < minDist {
			minDist = goalDist
			minType = scanGoal
		}

		if minDist < 64 {
			resultY := w*64 + minDist
			if minType == scanWall {
				if resultY-1 >= y {
					jps.recordCardinalScan(resultY-1, y, x, DirDown, cost, to)
				}
				return GridCoord{X: -1, Y: -1}
			}
			jps.recordCardinalScan(resultY, y, x, DirDown, cost, to)
			return GridCoord{X: x, Y: resultY}
		}

		// No event in this word — continue without per-word pathmap writes.
	}
	// Fell off grid edge.
	endY := bg.numRows - 1
	if endY >= y {
		jps.recordCardinalScan(endY, y, x, DirDown, cost, to)
	}
	return GridCoord{X: -1, Y: -1}
}

// Note: intentionally duplicated for performance. Keep synchronized with other jumpBit* functions.
//
// jumpBitUp scans upward from (x,y) using column bitboards.
func (jps *JPS) jumpBitUp(g *Grid, x, y int, to GridCoord, l GridLayer, cost int) GridCoord {
	bg := &jps.bitGrid
	bg.ensureCol(g, x, l)
	hasLeft := x > 0
	hasRight := x < bg.numCols-1
	if hasLeft {
		bg.ensureCol(g, x-1, l)
	}
	if hasRight {
		bg.ensureCol(g, x+1, l)
	}

	goalOnCol := to.X == x && to.Y < y
	startWord := y / 64
	startBit := uint(y % 64)

	// Hoist loop-invariant goal computation.
	goalW := to.Y / 64
	goalDistVal := 63 - (to.Y % 64)

	for w := startWord; w >= 0; w-- {
		col := bg.colWord(x, w)
		var forced uint64
		if hasLeft {
			colL := bg.colWord(x-1, w)
			prevBitL := uint64(0)
			if w > 0 {
				prevBitL = (bg.colWord(x-1, w-1) >> 63) & 1
			}
			lShifted := (colL << 1) | prevBitL
			forced = colL & ^lShifted
		}
		if hasRight {
			colR := bg.colWord(x+1, w)
			prevBitR := uint64(0)
			if w > 0 {
				prevBitR = (bg.colWord(x+1, w-1) >> 63) & 1
			}
			rShifted := (colR << 1) | prevBitR
			forced |= colR & ^rShifted
		}

		jumpCandidates := forced & ^col

		var mask uint64
		if w == startWord {
			mask = (1 << (startBit + 1)) - 1
		} else {
			mask = ^uint64(0)
		}
		jumpCandidates &= mask
		wallBits := col & mask

		wallDist := 64
		if wallBits != 0 {
			wallDist = bits.LeadingZeros64(wallBits)
		}
		jumpDist := 64
		if jumpCandidates != 0 {
			jumpDist = bits.LeadingZeros64(jumpCandidates)
		}

		goalDist := 64
		if goalOnCol && goalW == w {
			goalDist = goalDistVal
		}

		minDist := wallDist
		minType := scanWall
		if jumpDist < minDist {
			minDist = jumpDist
			minType = scanJump
		}
		if goalDist < minDist {
			minDist = goalDist
			minType = scanGoal
		}

		if minDist < 64 {
			resultY := w*64 + (63 - minDist)
			if minType == scanWall {
				if resultY+1 <= y {
					jps.recordCardinalScan(resultY+1, y, x, DirUp, cost, to)
				}
				return GridCoord{X: -1, Y: -1}
			}
			jps.recordCardinalScan(resultY, y, x, DirUp, cost, to)
			return GridCoord{X: x, Y: resultY}
		}

		// No event in this word — continue without per-word pathmap writes.
	}
	// Fell off grid edge.
	if y >= 0 {
		jps.recordCardinalScan(0, y, x, DirUp, cost, to)
	}
	return GridCoord{X: -1, Y: -1}
}

// recordCardinalScan records pathmap for all cells between start and end
// (inclusive) along a cardinal direction, and updates fallback using the
// analytically closest cell on the segment rather than per-cell chebyshev.
//
// For horizontal scans fixedAxis is Y; for vertical scans fixedAxis is X.
func (jps *JPS) recordCardinalScan(scanEnd, scanStart, fixedAxis int, dir Direction, startCost int, to GridCoord) {
	pm := jps.pathmap
	numCols := pm.numCols
	gen := pm.current
	genDir := (gen << dirShift) | uint32(dir)

	switch dir {
	case DirRight:
		k := uint(fixedAxis*numCols + scanStart)
		for cx := scanStart; cx <= scanEnd; cx++ {
			if pm.data[k]>>dirShift != gen {
				pm.data[k] = genDir
			}
			k++
		}
		bestX := clamp(to.X, scanStart, scanEnd)
		jps.updateFallback(GridCoord{X: bestX, Y: fixedAxis}, startCost+(bestX-scanStart), to)

	case DirLeft:
		k := uint(fixedAxis*numCols + scanStart)
		for cx := scanStart; cx >= scanEnd; cx-- {
			if pm.data[k]>>dirShift != gen {
				pm.data[k] = genDir
			}
			k--
		}
		bestX := clamp(to.X, scanEnd, scanStart)
		jps.updateFallback(GridCoord{X: bestX, Y: fixedAxis}, startCost+(scanStart-bestX), to)

	case DirDown:
		k := uint(scanStart*numCols + fixedAxis)
		for cy := scanStart; cy <= scanEnd; cy++ {
			if pm.data[k]>>dirShift != gen {
				pm.data[k] = genDir
			}
			k += uint(numCols)
		}
		bestY := clamp(to.Y, scanStart, scanEnd)
		jps.updateFallback(GridCoord{X: fixedAxis, Y: bestY}, startCost+(bestY-scanStart), to)

	case DirUp:
		k := uint(scanStart*numCols + fixedAxis)
		for cy := scanStart; cy >= scanEnd; cy-- {
			if pm.data[k]>>dirShift != gen {
				pm.data[k] = genDir
			}
			k -= uint(numCols)
		}
		bestY := clamp(to.Y, scanEnd, scanStart)
		jps.updateFallback(GridCoord{X: fixedAxis, Y: bestY}, startCost+(scanStart-bestY), to)
	}
}

func clamp(v, lo, hi int) int {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}

// jumpDiagPrune scans along a diagonal direction with intermediate jump point pruning.
// Instead of returning intermediate diagonal jump points, it directly pushes
// cardinal sub-jump points to the frontier (reducing heap operations).
func (jps *JPS) jumpDiagPrune(g *Grid, pos GridCoord, dir Direction, to GridCoord, l GridLayer, cost int) {
	perps := diagonalPerps[dir]
	d1, d2 := perps[0], perps[1]
	for {
		jps.updateFallback(pos, cost, to)

		if hasDiagonalForced(g, pos, dir, l) {
			// Real jump point with its own forced neighbors.
			jps.pushJumpPoint(pos, cost, to, dir)
			return
		}

		// Cardinal sub-jumps: push sub-jump points directly (pruning optimization).
		jp1 := jps.jumpCardBit(g, pos, d1, to, l, cost)
		jp2 := jps.jumpCardBit(g, pos, d2, to, l, cost)
		if jp1.X >= 0 || jp2.X >= 0 {
			// Found cardinal sub-jump(s). Push them and the current diagonal
			// position (needed as waypoint for path reconstruction).
			jps.pushJumpPoint(pos, cost, to, dir)
			if jp1.X >= 0 {
				jps.pushJumpPoint(jp1, cost+intabs(jp1.X-pos.X)+intabs(jp1.Y-pos.Y), to, d1)
			}
			if jp2.X >= 0 {
				jps.pushJumpPoint(jp2, cost+intabs(jp2.X-pos.X)+intabs(jp2.Y-pos.Y), to, d2)
			}
			return
		}

		next := pos.Move(dir)
		if g.GetCellCost(next, l) == 0 {
			return
		}
		pos = next
		cost++
		jps.pathmap.setIfAbsent(jps.pathmap.packCoord(pos), dir)
		if pos == to {
			jps.pushJumpPoint(pos, cost, to, dir)
			return
		}
	}
}

// jpsPrunedDirs returns the valid neighbor directions for a JPS node,
// given the direction from its parent. Corner cutting is allowed.
func jpsPrunedDirs(g *Grid, pos GridCoord, parentDir Direction, l GridLayer, out *[8]Direction) int {
	n := 0

	if isDiagonalDir(parentDir) {
		perps := diagonalPerps[parentDir]
		d1, d2 := perps[0], perps[1]
		fc := &diagForcedChecks[parentDir]
		out[n] = parentDir
		n++
		out[n] = d1
		n++
		out[n] = d2
		n++
		if g.GetCellCost(pos.Move(fc.block1), l) == 0 {
			out[n] = fc.force1
			n++
		}
		if g.GetCellCost(pos.Move(fc.block2), l) == 0 {
			out[n] = fc.force2
			n++
		}
	} else {
		out[n] = parentDir
		n++
		perps := cardinalPerps[parentDir]
		p1, p2 := perps[0], perps[1]
		if g.GetCellCost(pos.Move(p1), l) == 0 {
			out[n] = combineDirs(parentDir, p1)
			n++
			out[n] = p1
			n++
		}
		if g.GetCellCost(pos.Move(p2), l) == 0 {
			out[n] = combineDirs(parentDir, p2)
			n++
			out[n] = p2
			n++
		}
	}

	return n
}

// diagForcedCheck stores precomputed directions for forced neighbor detection.
// For a diagonal direction, block1/block2 are the two perpendicular directions to check,
// and force1/force2 are the corresponding forced diagonal neighbors.
type diagForcedCheck struct {
	block1, force1, block2, force2 Direction
}

var diagForcedChecks = [...]diagForcedCheck{
	{DirRight, DirUpRight, DirDown, DirDownLeft}, // DirUpLeft=0
	{DirNone, DirNone, DirNone, DirNone},         // DirUp=1
	{DirLeft, DirUpLeft, DirDown, DirDownRight},  // DirUpRight=2
	{DirNone, DirNone, DirNone, DirNone},         // DirLeft=3
	{DirNone, DirNone, DirNone, DirNone},         // DirRight=4
	{DirRight, DirDownRight, DirUp, DirUpLeft},   // DirDownLeft=5
	{DirNone, DirNone, DirNone, DirNone},         // DirDown=6
	{DirLeft, DirDownLeft, DirUp, DirUpRight},    // DirDownRight=7
	{DirNone, DirNone, DirNone, DirNone},         // DirNone=8
}

// hasDiagonalForced checks if a position has forced neighbors
// when approached from a diagonal direction.
func hasDiagonalForced(g *Grid, pos GridCoord, dir Direction, l GridLayer) bool {
	fc := &diagForcedChecks[dir]
	if g.GetCellCost(pos.Move(fc.block1), l) == 0 {
		if g.GetCellCost(pos.Move(fc.force1), l) != 0 {
			return true
		}
	}
	if g.GetCellCost(pos.Move(fc.block2), l) == 0 {
		if g.GetCellCost(pos.Move(fc.force2), l) != 0 {
			return true
		}
	}
	return false
}

func isDiagonalDir(d Direction) bool {
	return d == DirUpLeft || d == DirUpRight || d == DirDownLeft || d == DirDownRight
}

// diagonalPerps stores the two cardinal components of each diagonal direction.
var diagonalPerps = [...][2]Direction{
	{DirLeft, DirUp},    // DirUpLeft=0
	{DirNone, DirNone},  // DirUp=1
	{DirRight, DirUp},   // DirUpRight=2
	{DirNone, DirNone},  // DirLeft=3
	{DirNone, DirNone},  // DirRight=4
	{DirLeft, DirDown},  // DirDownLeft=5
	{DirNone, DirNone},  // DirDown=6
	{DirRight, DirDown}, // DirDownRight=7
	{DirNone, DirNone},  // DirNone=8
}

// cardinalPerps stores perpendicular directions for each cardinal direction.
// Indexed by Direction; non-cardinal entries are {DirNone, DirNone}.
var cardinalPerps = [...][2]Direction{
	{DirNone, DirNone},  // DirUpLeft=0
	{DirLeft, DirRight}, // DirUp=1
	{DirNone, DirNone},  // DirUpRight=2
	{DirUp, DirDown},    // DirLeft=3
	{DirUp, DirDown},    // DirRight=4
	{DirNone, DirNone},  // DirDownLeft=5
	{DirLeft, DirRight}, // DirDown=6
	{DirNone, DirNone},  // DirDownRight=7
	{DirNone, DirNone},  // DirNone=8
}

// combineDirs returns the diagonal direction formed by two cardinal directions.
// For example, combineDirs(DirUp, DirLeft) returns DirUpLeft.
func combineDirs(a, b Direction) Direction {
	switch a {
	case DirUp:
		if b == DirLeft {
			return DirUpLeft
		}
		return DirUpRight
	case DirDown:
		if b == DirLeft {
			return DirDownLeft
		}
		return DirDownRight
	case DirLeft:
		if b == DirUp {
			return DirUpLeft
		}
		return DirDownLeft
	default: // DirRight
		if b == DirUp {
			return DirUpRight
		}
		return DirDownRight
	}
}

func chebyshev(x1, y1, x2, y2 int) int {
	dx := intabs(x1 - x2)
	dy := intabs(y1 - y2)
	if dx > dy {
		return dx
	}
	return dy
}
