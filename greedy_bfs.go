package pathing

// GreedyBFS implements a greedy best-first search pathfinding algorithm.
// You must use NewGreedyBFS() function to obtain an instance of this type.
//
// GreedyBFS is a faster pathfinder with a lower memory costs as compared to an AStar.
// It can't handle different movements costs though, so it will treat any non-zero
// value returned by GridLayer identically.
//
// Once created, you should re-use it to build paths.
// Do not throw the instance away after building the path once.
type GreedyBFS struct {
	pqueue     *fixedPriorityQueue[weightedGridCoord]
	coordSlice []weightedGridCoord
	pathmap    *pathDirMap
	diagonal   bool
}

type weightedGridCoord struct {
	Coord  GridCoord
	Weight int
}

type GreedyBFSConfig struct {
	// NumCols and NumRows are size hints for the GreedyBFS constructor.
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

// NewGreedyBFS creates a ready-to-use GreedyBFS object.
func NewGreedyBFS(config GreedyBFSConfig) *GreedyBFS {
	if config.NumCols == 0 {
		config.NumCols = gridMapSide
	}
	if config.NumRows == 0 {
		config.NumRows = gridMapSide
	}

	coordMapCols := int(config.NumCols)
	coordMapRows := int(config.NumRows)

	bfs := &GreedyBFS{
		pqueue:     newFixedPriorityQueue[weightedGridCoord](),
		pathmap:    newPathDirMap(coordMapCols, coordMapRows),
		coordSlice: make([]weightedGridCoord, 0, 40),
		diagonal:   config.Diagonal,
	}

	return bfs
}

// BuildPath attempts to find a path between the two coordinates.
// It will use a provided Grid in combination with a GridLayer.
// The Grid is expected to store the tile tags and the GridLayer is
// used to interpret these tags.
//
// GreedyBFS treats 0 as "blocked" and any other value as "passable".
// If you need a cost-based pathfinding, use AStar instead.
func (bfs *GreedyBFS) BuildPath(g *Grid, from, to GridCoord, l GridLayer) BuildPathResult {
	var result BuildPathResult
	if from == to {
		result.Finish = to
		return result
	}

	frontier := bfs.pqueue
	frontier.Reset()

	hotFrontier := bfs.coordSlice[:0]
	hotFrontier = append(hotFrontier, weightedGridCoord{Coord: from})

	pathmap := bfs.pathmap
	pathmap.Reset()

	offsets := neighborOffsets[:4]
	if bfs.diagonal {
		offsets = neighborOffsets[:]
	}

	distFunc := GridCoord.Dist
	if bfs.diagonal {
		distFunc = chebyshevDist
	}

	var fallbackCoord GridCoord
	var fallbackDist, fallbackWeight int
	fallbackSet := false
	foundPath := false
	for len(hotFrontier) != 0 || !frontier.IsEmpty() {
		var current weightedGridCoord
		if len(hotFrontier) != 0 {
			current = hotFrontier[len(hotFrontier)-1]
			hotFrontier = hotFrontier[:len(hotFrontier)-1]
		} else {
			current = frontier.Pop()
		}

		if current.Coord == to {
			result.Steps = constructPath(from, to, pathmap)
			result.Finish = to
			result.Cost = result.Steps.Len()
			foundPath = true
			break
		}

		dist := distFunc(to, current.Coord)
		if !fallbackSet || dist < fallbackDist || (dist == fallbackDist && current.Weight < fallbackWeight) {
			fallbackCoord = current.Coord
			fallbackDist = dist
			fallbackWeight = current.Weight
			fallbackSet = true
		}

		for dir, offset := range offsets {
			next := current.Coord.Add(offset)
			if g.GetCellCost(next, l) == 0 {
				continue
			}
			pathmapKey := pathmap.packCoord(next)
			if pathmap.Contains(pathmapKey) {
				continue
			}
			pathmap.Set(pathmapKey, Direction(dir))
			nextDist := distFunc(to, next)
			nextWeighted := weightedGridCoord{
				Coord: next,
				// This is used to determine the out-of-scope coordinates.
				// It's not a distance score; therefore, we're not using nextDist here.
				Weight: current.Weight + 1,
			}
			if nextDist < dist {
				hotFrontier = append(hotFrontier, nextWeighted)
			} else {
				frontier.Push(nextDist, nextWeighted)
			}
		}
	}

	if !foundPath {
		result.Steps = constructPath(from, fallbackCoord, pathmap)
		result.Finish = fallbackCoord
		result.Cost = result.Steps.Len()
		result.Partial = true
	}

	// In case if that slice was growing due to appends,
	// save that extra capacity for later.
	bfs.coordSlice = hotFrontier[:0]

	return result
}
