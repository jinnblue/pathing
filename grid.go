package pathing

import (
	"math"
	"slices"
	"strings"
	"unsafe"
)

// GridLayer is a tile-to-cost mapper.
// Every Grid cell has a tile tag value ranging from 0 to 7 (3 bits).
// Layers are used to turn that tag value into an actual traversal cost.
//
// Although you can construct a GridLayer value yourself,
// it's much easier to use a MakeGridLayer() function.
//
// A byte value of 0 means "the cell can't be traversed".
// A value higher than 0 means "traversing this cell costs X points".
// The pathfinding algorithms will respect that value when finding a solution.
//
// In the simplest situations just use 0 (no path) and 1 (can traverse).
type GridLayer [2]uint64

type LayerCost [8]uint8

func costsMapping(costs LayerCost) uint64 {
	return uint64(costs[0]) | uint64(costs[1])<<8 | uint64(costs[2])<<16 | uint64(costs[3])<<24 |
		uint64(costs[4])<<32 | uint64(costs[5])<<40 | uint64(costs[6])<<48 | uint64(costs[7])<<56
}

// MakeGridLayer is a GridLayer constructor function.
// It uses a temporary array to fill the result layer.
//
// The array represents a mapping from a tile tag (the key)
// to a traversal cost (the value).
//
// If you want blocked (occupied) tiles to be traversible,
// see MakeGridLayerWithBlocked constructor.
func MakeGridLayer(costs LayerCost) GridLayer {
	return GridLayer{costsMapping(costs), 0}
}

// MakeGridLayerWithBlocked is like MakeGridLayer, but allows
// a custom movement cost per a blocked tile.
// Setting a movement cost of 0 will keep that cell impossible to move through.
func MakeGridLayerWithBlocked(costs LayerCost, blocked LayerCost) (l GridLayer) {
	tileMapping := costsMapping(costs)
	blockedMapping := costsMapping(blocked)
	return GridLayer{tileMapping, blockedMapping}
}

// Get maps a given tile tag into a traversal score.
// A tile tag is a value in [0-7] range.
func (l GridLayer) Get(tileTag uint8) uint8 {
	return uint8(l[0] >> (uint64(tileTag) * 8))
}

func (l GridLayer) getFast(tag uint8) uint8 {
	return *(*uint8)(unsafe.Add(unsafe.Pointer(&l), tag))
}

type Grid struct {
	bytes []byte

	numCols uint
	numRows uint

	// mutGen is incremented on every cell mutation (SetCellTile, SetCellIsBlocked).
	// Used by JPS to detect when the cached bitgrid must be invalidated.
	mutGen uint32

	cellWidth  int
	cellHeight int

	fcellWidth      float64
	fcellHeight     float64
	fcellHalfWidth  float64
	fcellHalfHeight float64
}

// GridConfig is a NewGrid() function parameter.
// See field comments for more details.
type GridConfig struct {
	// WorldWidth and WorldHeight specify the modelled world size.
	// These are used in combination with cell sizes to map positions
	// and grid coordinates.
	// The world size is specified in pixels.
	// Although the positions are expected to be a pair of float64,
	// the world size is a pair of uints, because sizes like 200.5 make no sense.
	WorldWidth  uint
	WorldHeight uint

	// CellWidth and CellHeight specify the grid cell size.
	// If the world size is 320x320 and the cell size is 32x32,
	// that would mean that there are 10x10 (100) cells in total.
	//
	// If left unset (0), the default size will be used (32x32).
	CellWidth  uint
	CellHeight uint

	// DefaultTile controls the default grid fill.
	// Only the 3 lower bits matter as a tile tag value can't exceed a value of 7 (0b0111).
	// This value is a minor option, but it can be used to populate the grid
	// with the most common tile.
	// Although it does fill the grid in an optimized way, it's mostly a convenience option
	// to make the initialization easier.
	DefaultTile uint8
}

const (
	tileMask     uint8 = 0b1111
	tileBlockBit uint8 = 0b1000
	tileTagMask  uint8 = 0b0111
	tileTagShift uint  = 4
	tileTagCount uint  = 8 / tileTagShift
)

// NewGrid creates a Grid object.
// See GridConfig comment to learn more.
func NewGrid(config GridConfig) *Grid {
	if config.CellWidth == 0 {
		config.CellWidth = 32
	}
	if config.CellHeight == 0 {
		config.CellHeight = 32
	}

	g := &Grid{
		cellWidth:  int(config.CellWidth),
		cellHeight: int(config.CellHeight),

		fcellWidth:  float64(config.CellWidth),
		fcellHeight: float64(config.CellHeight),
	}

	g.fcellHalfWidth = float64(config.CellWidth / 2)
	g.fcellHalfHeight = float64(config.CellHeight / 2)

	g.numCols = config.WorldWidth / config.CellWidth
	g.numRows = config.WorldHeight / config.CellHeight

	numCells := g.numCols * g.numRows
	numBytes := numCells / tileTagCount
	if numCells%tileTagCount != 0 {
		numBytes++
	}
	b := make([]byte, numBytes)

	defaultTileTag := config.DefaultTile
	defaultTileTag &= tileTagMask
	if defaultTileTag != 0 {
		v := defaultTileTag | (defaultTileTag << tileTagShift)
		for i := range b {
			b[i] = v
		}
	}

	g.bytes = b

	return g
}

// Clone returns a deep copy of the Grid.
func (g *Grid) Clone() *Grid {
	clone := *g
	clone.bytes = slices.Clone(g.bytes)
	return &clone
}

// NumCols returns the number of columns this grid has.
func (g *Grid) NumCols() int { return int(g.numCols) }

// NumRows returns the number of rows this grid has.
func (g *Grid) NumRows() int { return int(g.numRows) }

// SetCellTile assigns the tile tag for the given cell coordinate.
//
// You usually do not want to change the tile types after the grid
// is filled, but if your map is dynamic, it is OK to do so
// as it is an O(1) operation.
//
// For dynamic info like "tile is blocked" use the separate bit
// accessible through some of the APIs (e.g. SetCellIsBlocked, GetCellTile2, GetCellIsBlocked)
func (g *Grid) SetCellTile(c GridCoord, tileTag uint8) {
	i := uint(c.Y)*g.numCols + uint(c.X)
	byteIndex := i / tileTagCount
	if byteIndex < uint(len(g.bytes)) {
		shift := (i % tileTagCount) * uint(tileTagShift)
		b := g.bytes[byteIndex]
		b &^= tileTagMask << shift            // Clear the four data bits
		b |= (tileTag & tileTagMask) << shift // Mix it with provided bits
		g.bytes[byteIndex] = b
		g.mutGen++
	}
}

// SetCellIsBlocked writes to a special tile "blocked" bit (0 or 1).
// You can read that bit with GetCellIsBlocked, it is also reported with GetCellTile2.
// Blocked cells usually can not be traversed, but it can be changed with the layer
// with a dedicated blocked tile map.
func (g *Grid) SetCellIsBlocked(c GridCoord, blocked bool) {
	bit := uint8(0)
	if blocked {
		bit = tileBlockBit
	}
	i := uint(c.Y)*g.numCols + uint(c.X)
	byteIndex := i / tileTagCount
	if byteIndex < uint(len(g.bytes)) {
		shift := (i % tileTagCount) * uint(tileTagShift)
		b := g.bytes[byteIndex]
		b &^= tileBlockBit << shift // Clear the bit
		b |= bit << shift           // Mix it in
		g.bytes[byteIndex] = b
		g.mutGen++
	}
}

// GetCellTile returns the cell tile tag.
// This operation is only useful for the Grid debugging as
// for the pathfinding tasks you would want to use GetCellCost() method instead.
//
// An out-of-bounds access returns 0.
//
// This function never reports whether a tile is blocked or not,
// it only returns the tag associated with it.
func (g *Grid) GetCellTile(c GridCoord) uint8 {
	x := uint(c.X)
	y := uint(c.Y)
	if x >= g.numCols || y >= g.numRows {
		return 0
	}
	i := y*g.numCols + x
	byteIndex := i / tileTagCount
	shift := (i % tileTagCount) * uint(tileTagShift)
	return (g.bytes[byteIndex] >> shift) & tileTagMask
}

// GetCellTile2 is like GetCellTile, but also reports
// whether the tile was marked as blocked (occupied).
//
// An out-of-bounds access is not marked as occupied.
func (g *Grid) GetCellTile2(c GridCoord) (uint8, bool) {
	x := uint(c.X)
	y := uint(c.Y)
	if x >= g.numCols || y >= g.numRows {
		return 0, false
	}
	i := y*g.numCols + x
	byteIndex := i / tileTagCount
	shift := (i % tileTagCount) * uint(tileTagShift)
	bits := g.bytes[byteIndex] >> shift
	return bits & tileTagMask, bits&tileBlockBit != 0
}

// GetCellIsBlocked reports whether the given cell was marked as blocked.
// By default, no cells are marked as such, until SetCellIsBlocked is called.
//
// An out-of-bounds cell is never blocked.
func (g *Grid) GetCellIsBlocked(c GridCoord) bool {
	x := uint(c.X)
	y := uint(c.Y)
	if x >= g.numCols || y >= g.numRows {
		return false
	}
	i := y*g.numCols + x
	byteIndex := i / tileTagCount
	shift := (i % tileTagCount) * uint(tileTagShift)
	return (g.bytes[byteIndex]>>shift)&tileBlockBit != 0
}

// GetCellCost returns a travelling cost for a given cell as specified in the layer.
// The return value interpreted as this: 0 is a blocked path while any other value
// is a travelling cost.
//
// An out-of-bounds access returns 0 (interpreted as blocked).
func (g *Grid) GetCellCost(c GridCoord, l GridLayer) uint8 {
	x := uint(c.X)
	y := uint(c.Y)
	if x >= g.numCols || y >= g.numRows {
		// Consider out of bound cells as blocked.
		return 0
	}
	return g.getCellCost(x, y, l)
}

func (g *Grid) getCellCost(x, y uint, l GridLayer) uint8 {
	i := y*g.numCols + x
	byteIndex := i / tileTagCount
	shift := (i % tileTagCount) * uint(tileTagShift)
	tileTag := (g.bytes[byteIndex] >> shift) & tileMask
	return l.getFast(tileTag)
}

// AlignPos is an easy way to center the world position inside a grid cell.
// For instance, with a cell size of 32x32, a {10,10} pos would become {16,16}.
func (g *Grid) AlignPos(x, y float64) (float64, float64) {
	return g.CoordToPos(g.PosToCoord(x, y))
}

// PosToCoord converts a world position into a grid coord.
func (g *Grid) PosToCoord(x, y float64) GridCoord {
	return GridCoord{
		X: int(x) / g.cellWidth,
		Y: int(y) / g.cellHeight,
	}
}

// CoordToPos converts a grid coord into a world position.
func (g *Grid) CoordToPos(c GridCoord) (float64, float64) {
	x := (float64(c.X) * g.fcellWidth) + g.fcellHalfWidth
	y := (float64(c.Y) * g.fcellHeight) + g.fcellHalfHeight
	return x, y
}

// PackCoord returns a packed version of a grid coordinate.
// It can be useful to get an efficient map key.
// A packed coordinate can later be unpacked with UnpackCoord() method.
func (g *Grid) PackCoord(c GridCoord) uint32 {
	return uint32(c.X) | uint32(c.Y<<16)
}

// UnpackCoord takes a packed coord and returns its unpacked version.
func (g *Grid) UnpackCoord(v uint32) GridCoord {
	u32 := uint32(v)
	x := int(u32 & 0xffff)
	y := int(u32 >> 16)
	return GridCoord{X: x, Y: y}
}

const (
	// gridPathBytes controls the max number of steps a GridPath can hold.
	// Each byte stores 2 direction values (4 bits each), so the max path
	// length is gridPathBytes*2. 512 bytes = 1024 steps, enough for a
	// 512x512 grid in both 4-directional and 8-directional (diagonal) modes.
	gridPathBytes  = 512
	gridPathMaxLen = gridPathBytes * 2

	// gridMapSide is the default working-area size used when NumCols/NumRows
	// are not provided to NewAStar or NewGreedyBFS.
	gridMapSide = 114
)

// GridCoord represents a grid-local coordinate.
// You can translate it to a world coordinate using a grid.
//
// If the grid cell size is 32x32, then this table can explain the mapping:
//
//   - pos{0, 0}   => coord{0, 0}
//   - pos{16, 16} => coord{0, 0}
//   - pos{20, 20} => coord{0, 0}
//   - pos{35, 10} => coord{1, 0}
//   - pos{50, 50} => coord{1, 1}
//   - pos{90, 90} => coord{2, 2}
type GridCoord struct {
	X int
	Y int
}

// IsZero reports whether the coord is {0, 0}.
func (c GridCoord) IsZero() bool {
	return c.X == 0 && c.Y == 0
}

// Add performs a + operation and returns the result coordinate.
func (c GridCoord) Add(other GridCoord) GridCoord {
	return GridCoord{X: c.X + other.X, Y: c.Y + other.Y}
}

// Sub performs a - operation and returns the result coordinate.
func (c GridCoord) Sub(other GridCoord) GridCoord {
	return GridCoord{X: c.X - other.X, Y: c.Y - other.Y}
}

// dirOffset maps Direction to (dx, dy) offsets.
var dirOffset = [...]GridCoord{
	{-1, -1}, // DirUpLeft=0
	{0, -1},  // DirUp=1
	{1, -1},  // DirUpRight=2
	{-1, 0},  // DirLeft=3
	{1, 0},   // DirRight=4
	{-1, 1},  // DirDownLeft=5
	{0, 1},   // DirDown=6
	{1, 1},   // DirDownRight=7
	{0, 0},   // DirNone=8
}

func (c GridCoord) reversedMove(d Direction) GridCoord {
	do := dirOffset[d]
	return GridCoord{X: c.X - do.X, Y: c.Y - do.Y}
}

// Move translates the coordinate one step towards the direction.
//
// Note that the coordinates are not validated.
// It's possible to get an out-of-bounds coordinate that
// will not belong to a Grid.
//
//   - {2,2}.Move(DirLeft) would give {1,2}
//   - {2,2}.Move(DirDown) would give {2,3}
//   - {2,2}.Move(DirDownRight) would give {3,3}
func (c GridCoord) Move(d Direction) GridCoord {
	do := dirOffset[d]
	return GridCoord{X: c.X + do.X, Y: c.Y + do.Y}
}

// DistManhattan finds a Manhattan distance between the two coordinates.
func (c GridCoord) DistManhattan(other GridCoord) int {
	return intabs(c.X-other.X) + intabs(c.Y-other.Y)
}

// DistOctile finds a Octile distance between the two coordinates.
// If diagonal movement equal linear movement, should use Chebyshev distance.
func (c GridCoord) DistOctile(other GridCoord) int {
	dx := c.X - other.X
	dy := c.Y - other.Y
	if dx < 0 {
		dx = -dx
	}
	if dy < 0 {
		dy = -dy
	}
	return 4*min(dx, dy) + 10*max(dx, dy)
}

// Direction is a simple enumeration of axial and diagonal movement directions.
type Direction int

const (
	DirUpLeft Direction = iota
	DirUp
	DirUpRight
	DirLeft
	DirRight
	DirDownLeft
	DirDown
	DirDownRight
	DirNone // A special sentinel value
)

func (d Direction) String() string {
	switch d {
	case DirUpLeft:
		return "UpLeft"
	case DirUp:
		return "Up"
	case DirUpRight:
		return "UpRight"
	case DirLeft:
		return "Left"
	case DirRight:
		return "Right"
	case DirDownLeft:
		return "DownLeft"
	case DirDown:
		return "Down"
	case DirDownRight:
		return "DownRight"
	default:
		return "None"
	}
}

// dirOffset maps Direction to (dx, dy) offsets.
var dirReverse = [...]Direction{
	DirDownRight, // DirUpLeft=0
	DirDown,      // DirUp=1
	DirDownLeft,  // DirUpRight=2
	DirRight,     // DirLeft=3
	DirLeft,      // DirRight=4
	DirUpRight,   // DirDownLeft=5
	DirUp,        // DirDown=6
	DirUpLeft,    // DirDownRight=7
	DirNone,      // DirNone=8
}

// Reversed returns an opposite direction.
// For instance, DirRight would become DirLeft.
func (d Direction) Reversed() Direction {
	return dirReverse[d]
}

type coordMap struct {
	elems   []coordMapElem
	gen     uint32
	numRows int
	numCols int
}

type coordMapElem struct {
	value uint32
	gen   uint32
}

func newCoordMap(numCols, numRows int) *coordMap {
	size := numRows * numCols
	return &coordMap{
		elems:   make([]coordMapElem, size),
		gen:     1,
		numRows: numRows,
		numCols: numCols,
	}
}

func (m *coordMap) Contains(k uint) bool {
	if k < uint(len(m.elems)) {
		return m.elems[k].gen == m.gen
	}
	return false
}

func (m *coordMap) Get(k uint) (uint32, bool) {
	if k < uint(len(m.elems)) {
		el := m.elems[k]
		if el.gen == m.gen {
			return el.value, true
		}
	}
	return 0, false
}

func (m *coordMap) Set(k uint, v uint32) {
	if k < uint(len(m.elems)) {
		m.elems[k] = coordMapElem{value: v, gen: m.gen}
	}
}

func (m *coordMap) Reset() {
	if m.gen == math.MaxUint32 {
		// For most users, this will never happen.
		// But to be safe, we need to handle this correctly.
		// m.gen will be 1, element gen will be 0.
		m.clear()
	} else {
		m.gen++
	}
}

// clear does a real array data reset.
// m.gen becomes 1.
// Every element gen becomes 0.
// This is identical to the initial array state.
//
//go:noinline - called on a cold path, therefore it should not be inlined.
func (m *coordMap) clear() {
	m.gen = 1
	clear(m.elems)
}

func (s *coordMap) packCoord(c GridCoord) uint {
	return uint((c.Y * s.numCols) + c.X)
}

// pathDirMap stores per-cell direction data with generation-based reset.
// Each entry packs generation (bits 4-31) and direction (bits 0-3) into a single uint32,
// reducing memory and improving cache locality vs separate gen+dirs arrays.
type pathDirMap struct {
	data    []uint32 // packed: (generation << 4) | direction
	current uint32   // current generation; only lower 28 bits used
	numRows int
	numCols int
}

const pathDirMapMaxGen = (1 << 28) - 1

func newPathDirMap(numCols, numRows int) *pathDirMap {
	size := numRows * numCols
	return &pathDirMap{
		data:    make([]uint32, size),
		current: 1,
		numRows: numRows,
		numCols: numCols,
	}
}

func (m *pathDirMap) packCoord(c GridCoord) uint {
	return uint((c.Y * m.numCols) + c.X)
}

func (m *pathDirMap) Contains(k uint) bool {
	if k < uint(len(m.data)) {
		return m.data[k]>>dirShift == m.current
	}
	return false
}

func (m *pathDirMap) Get(k uint) (Direction, bool) {
	if k < uint(len(m.data)) {
		v := m.data[k]
		if v>>dirShift == m.current {
			return Direction(v & dirMask), true
		}
	}
	return DirNone, false
}

func (m *pathDirMap) Set(k uint, d Direction) {
	if k < uint(len(m.data)) {
		m.data[k] = (m.current << dirShift) | uint32(d)
	}
}

// setIfAbsent writes d at key k only if k is not already set in this generation.
// Skips bounds check — caller must ensure k is in range.
func (m *pathDirMap) setIfAbsent(k uint, d Direction) {
	if m.data[k]>>dirShift != m.current {
		m.data[k] = (m.current << dirShift) | uint32(d)
	}
}

func (m *pathDirMap) Reset() {
	if m.current >= pathDirMapMaxGen {
		m.clear()
	} else {
		m.current++
	}
}

//go:noinline - called on a cold path, therefore it should not be inlined.
func (m *pathDirMap) clear() {
	m.current = 1
	clear(m.data)
}

func intabs(x int) int {
	if x < 0 {
		return -x
	}
	return x
}

const (
	dirShift = 4
	dirCount = 8 / dirShift // 2 directions per byte
	dirMask  = 0b1111
)

type neighborGrid struct {
	Direction
	GridCoord
}

// neighborCardinal lists the 4 cardinal directions in raster-scan order (Up, Left, Right, Down).
var neighborCardinal = [...]neighborGrid{
	{DirUp, GridCoord{X: 0, Y: -1}},
	{DirLeft, GridCoord{X: -1, Y: 0}},
	{DirRight, GridCoord{X: 1, Y: 0}},
	{DirDown, GridCoord{X: 0, Y: 1}},
}

// neighborDiagonal lists all 8 movement directions in cache-friendly raster-scan order
// (top-to-bottom, left-to-right). Grid cells are laid out row-major.
var neighborDiagonal = [...]neighborGrid{
	{DirUpLeft, GridCoord{X: -1, Y: -1}},
	{DirUp, GridCoord{X: 0, Y: -1}},
	{DirUpRight, GridCoord{X: 1, Y: -1}},
	{DirLeft, GridCoord{X: -1, Y: 0}},
	{DirRight, GridCoord{X: 1, Y: 0}},
	{DirDownLeft, GridCoord{X: -1, Y: 1}},
	{DirDown, GridCoord{X: 0, Y: 1}},
	{DirDownRight, GridCoord{X: 1, Y: 1}},
}

// GridPath represents a constructed path from point A to point B.
//
// Instead of storing the actual coordinates, it stores deltas in a form of step directions.
//
// You get from point A to point B by taking the steps into the directions
// specified by the path.
// The path object is essentialy an iterator.
//
// The path can be copied by simply assigning it, it has a value semantics.
// You want to pass it around as a value 90% of time,
// but if you want some function to be able to affect the iterator state,
// pass it by the pointer.
type GridPath struct {
	bytes [gridPathBytes]byte
	len   uint16
	pos   uint16
}

// MakeGridPath construct a path from the given set of steps.
func MakeGridPath(steps ...Direction) GridPath {
	var result GridPath
	for i := len(steps) - 1; i >= 0; i-- {
		result.push(steps[i])
	}
	result.Rewind()
	return result
}

// String returns a debug-print version of the path.
// It's not intended to be used a fast path-to-string method.
func (p GridPath) String() string {
	parts := make([]string, 0, p.len)
	prevPos := p.pos // Restore the pos later
	p.Rewind()
	for p.HasNext() {
		parts = append(parts, p.Next().String())
	}
	p.pos = prevPos
	return "{" + strings.Join(parts, ",") + "}"
}

// Len returns the path length.
// It's not affected by the iterator state; the result is always
// a total path length regardless of the progress.
func (p *GridPath) Len() int {
	return int(p.len)
}

// HasNext reports whether there are more steps inside this path.
// Use Next() to extract the next path segment if there are any.
func (p *GridPath) HasNext() bool {
	return p.pos != 0
}

// Rewind resets the iterator and allows you to traverse it again.
func (p *GridPath) Rewind() {
	p.pos = p.len
}

// Peek returns the next path step without advancing the iterator.
func (p *GridPath) Peek() Direction {
	return p.get(p.pos - 1)
}

// Next returns the next path step and advances the iterator.
func (p *GridPath) Next() Direction {
	d := p.Peek()
	p.pos--
	return d
}

// Skip consumes n next path steps.
func (p *GridPath) Skip(n uint16) {
	p.pos -= n
}

// Peek2 is like Peek(), but it returns two next steps instead of just one.
func (p *GridPath) Peek2() (Direction, Direction) {
	// If p.pos is 1, p.pos-2 overflows to 255.
	// byteIndex will not be inside len(p.bytes), so
	// p.get(p.pos-2) will return DirNone as it should.
	// No need to check for that condition here explicitely.
	return p.get(p.pos - 1), p.get(p.pos - 2)
}

func (p *GridPath) push(dir Direction) {
	i := p.pos
	p.pos++
	p.len++
	byteIndex := i / dirCount
	bitShift := byte((i % dirCount) * dirShift)
	if int(byteIndex) < len(p.bytes) {
		b := p.bytes[byteIndex]
		b &^= dirMask << bitShift
		b |= (byte(dir) & dirMask) << bitShift
		p.bytes[byteIndex] = b
	}
}

func (p *GridPath) get(i uint16) Direction {
	byteIndex := i / dirCount
	bitShift := byte((i % dirCount) * dirShift)
	if int(byteIndex) < len(p.bytes) {
		return Direction((p.bytes[byteIndex] >> bitShift) & dirMask)
	}
	return DirNone
}

func constructPath(from, to GridCoord, pathmap *pathDirMap) GridPath {
	// We walk from the finish point towards the start.
	// The directions are pushed in that order and would lead
	// to a reversed path, but since GridPath does its iteration
	// in reversed order itself, we don't need to do any
	// post-build reversal here.
	var result GridPath
	pos := to
	for {
		d, _ := pathmap.Get(pathmap.packCoord(pos))
		if pos == from {
			break
		}
		result.push(d)
		pos = pos.reversedMove(d)
	}
	return result
}

// BuildPathResult is a BuildPath() method return value.
type BuildPathResult struct {
	// Steps is an actual path that was constructed.
	Steps GridPath

	// Finish is where the constructed path ends.
	// It's mostly needed in case of a partial result,
	// since you can build another path from this coord right away.
	Finish GridCoord

	// Cost is a path final movement cost.
	// For GreedyBFS this equals the path step count.
	// For AStar this is the accumulated tile-layer cost.
	Cost int

	// Whether this is a partial path result.
	// This happens if the destination can't be reached
	// or if it's too far away.
	Partial bool
}
