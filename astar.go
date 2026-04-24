package pathing

// AStar implements an A* search pathfinding algorithm.
// You must use NewAStar() function to obtain an instance of this type.
//
// AStar is a bit slower than GreedyBFS, but its results can be more optimal.
// It also supports a proper weight/cost based pathfinding.
//
// Once created, you should re-use it to build paths.
// Do not throw the instance away after building the path once.
//
// Cost scaling in diagonal mode:
// When Diagonal is true, g-values (and therefore BuildPathResult.Cost) are
// stored in integer micro-units where a cardinal step costs 10 and a diagonal
// step costs 14 (≈10·√2). This matches DistOctile's 10:14 scale, making the
// heuristic admissible and consistent, which in turn enables a monotone radix
// heap for the frontier. In cardinal-only mode (Diagonal=false) the raw tile
// cost is used directly with DistManhattan, which is already consistent.
type AStar struct {
	frontier *radixHeap[pathCoord]
	costmap  *coordMap
	pathmap  *pathDirMap
	diagonal bool
}

type AStarConfig struct {
	// NumCols and NumRows are size hints for the AStar constructor.
	// Grid.NumCols() and Grid.NumRows() methods will come in handy to initialize these.
	// If you keep them at 0, the max amount of the working space will be allocated.
	// It's like a size hint: the constructor may allocate a smaller working area
	// if the grids you're going operate on are small.
	NumCols uint
	NumRows uint

	// Diagonal enables 8-directional (diagonal) movement.
	// When false (the default), only the 4 cardinal directions are used.
	Diagonal bool
}

// Diagonal-mode step multipliers matching DistOctile (10*max + 4*min).
const (
	cardinalMul = 10
	diagonalMul = 14
)

// diagonalStepMul gives the per-direction cost multiplier (applied to
// GetCellCost) in diagonal mode, indexed by position inside neighborDiagonal.
// neighborDiagonal order: UL, U, UR, L, R, DL, D, DR.
var diagonalStepMul = [...]uint32{
	diagonalMul, // UL
	cardinalMul, // U
	diagonalMul, // UR
	cardinalMul, // L
	cardinalMul, // R
	diagonalMul, // DL
	cardinalMul, // D
	diagonalMul, // DR
}

// NewAStar creates a ready-to-use AStar object.
func NewAStar(config AStarConfig) *AStar {
	if config.NumCols == 0 {
		config.NumCols = gridMapSide
	}
	if config.NumRows == 0 {
		config.NumRows = gridMapSide
	}

	coordMapCols := int(config.NumCols)
	coordMapRows := int(config.NumRows)

	return &AStar{
		pathmap:  newPathDirMap(coordMapCols, coordMapRows),
		costmap:  newCoordMap(coordMapCols, coordMapRows),
		frontier: newRadixHeap[pathCoord](),
		diagonal: config.Diagonal,
	}
}

// BuildPath attempts to find a path between the two coordinates.
// It will use a provided Grid in combination with a GridLayer.
// The Grid is expected to store the tile tags and the GridLayer is
// used to interpret these tags.
func (astar *AStar) BuildPath(g *Grid, from, to GridCoord, l GridLayer) BuildPathResult {
	if from == to {
		return BuildPathResult{Finish: to}
	}
	if astar.diagonal {
		return astar.buildPathDiagonal(g, from, to, l)
	}
	return astar.buildPathCardinal(g, from, to, l)
}

// buildPathCardinal handles 4-directional A* using dist manhattan with raw
// tile costs (already a consistent heuristic for uniform non-negative costs).
func (astar *AStar) buildPathCardinal(g *Grid, from, to GridCoord, l GridLayer) (result BuildPathResult) {
	frontier := astar.frontier
	frontier.Reset()
	pathmap := astar.pathmap
	pathmap.Reset()
	costmap := astar.costmap
	costmap.Reset()

	costmap.Set(costmap.packCoord(from), 0)
	frontier.Push(0, pathCoord{Coord: from})

	var fallbackCoord GridCoord
	var fallbackDist, fallbackCost int
	fallbackSet := false
	foundPath := false

	for !frontier.IsEmpty() {
		current := frontier.Pop()
		currentKey := costmap.packCoord(current.Coord)
		currentCost, _ := costmap.Get(currentKey)
		if current.Cost != int32(currentCost) {
			continue
		}

		if current.Coord == to {
			result.Steps = constructPath(from, to, pathmap)
			result.Finish = to
			result.Cost = int(current.Cost)
			foundPath = true
			break
		}

		// Fallback candidate: closest cell visited so far (cardinal manhattan distance).
		dist := to.DistManhattan(current.Coord)
		cost := int(current.Cost)
		if !fallbackSet || dist < fallbackDist || (dist == fallbackDist && cost < fallbackCost) {
			fallbackCoord = current.Coord
			fallbackDist = dist
			fallbackCost = cost
			fallbackSet = true
		}

		for _, nb := range &neighborCardinal {
			next := current.Coord.Add(nb.GridCoord)
			nextCellCost := g.GetCellCost(next, l)
			if nextCellCost == 0 {
				continue
			}
			newNextCost := currentCost + uint32(nextCellCost)
			k := costmap.packCoord(next)
			oldNextCost, ok := costmap.Get(k)
			if ok && newNextCost >= oldNextCost {
				continue
			}
			costmap.Set(k, newNextCost)
			priority := int32(newNextCost) + int32(to.DistManhattan(next))
			frontier.Push(priority, pathCoord{
				Coord: next,
				Cost:  int32(newNextCost),
			})
			pathmap.Set(k, nb.Direction)
		}
	}

	if !foundPath {
		result.Steps = constructPath(from, fallbackCoord, pathmap)
		result.Finish = fallbackCoord
		result.Cost = fallbackCost
		result.Partial = true
	}
	return result
}

// buildPathDiagonal handles 8-directional A* using dist octile with scaled g
// (cardinal steps cost 10·cellCost, diagonal steps cost 14·cellCost). This
// keeps h admissible+consistent so the radix heap invariant (monotone pops)
// holds. BuildPathResult.Cost is therefore reported in these micro-units.
func (astar *AStar) buildPathDiagonal(g *Grid, from, to GridCoord, l GridLayer) (result BuildPathResult) {
	frontier := astar.frontier
	frontier.Reset()
	pathmap := astar.pathmap
	pathmap.Reset()
	costmap := astar.costmap
	costmap.Reset()

	costmap.Set(costmap.packCoord(from), 0)
	frontier.Push(0, pathCoord{Coord: from})

	var fallbackCoord GridCoord
	var fallbackDist, fallbackCost int
	fallbackSet := false
	foundPath := false

	for !frontier.IsEmpty() {
		current := frontier.Pop()
		currentKey := costmap.packCoord(current.Coord)
		currentCost, _ := costmap.Get(currentKey)
		if current.Cost != int32(currentCost) {
			continue
		}

		if current.Coord == to {
			result.Steps = constructPath(from, to, pathmap)
			result.Finish = to
			result.Cost = int(current.Cost / 10)
			foundPath = true
			break
		}

		// Octile fallback distance (scaled, matches DistOctile).
		dist := to.DistOctile(current.Coord)
		cost := int(current.Cost)
		if !fallbackSet || dist < fallbackDist || (dist == fallbackDist && cost < fallbackCost) {
			fallbackCoord = current.Coord
			fallbackDist = dist
			fallbackCost = cost
			fallbackSet = true
		}

		for _, nb := range &neighborDiagonal {
			next := current.Coord.Add(nb.GridCoord)
			nextCellCost := g.GetCellCost(next, l)
			if nextCellCost == 0 {
				continue
			}
			newNextCost := currentCost + diagonalStepMul[nb.Direction]*uint32(nextCellCost)
			k := costmap.packCoord(next)
			oldNextCost, ok := costmap.Get(k)
			if ok && newNextCost >= oldNextCost {
				continue
			}
			costmap.Set(k, newNextCost)
			priority := int32(newNextCost) + int32(to.DistOctile(next))
			frontier.Push(priority, pathCoord{
				Coord: next,
				Cost:  int32(newNextCost),
			})
			pathmap.Set(k, nb.Direction)
		}
	}

	if !foundPath {
		result.Steps = constructPath(from, fallbackCoord, pathmap)
		result.Finish = fallbackCoord
		result.Cost = fallbackCost / 10
		result.Partial = true
	}
	return result
}
