package pathing

// AStar implements an A* search pathfinding algorithm.
// You must use NewAStar() function to obtain an instance of this type.
//
// AStar is a bit slower than GreedyBFS, but its results can be more optimal.
// It also supports a proper weight/cost based pathfinding.
//
// Once created, you should re-use it to build paths.
// Do not throw the instance away after building the path once.
type AStar struct {
	frontier *minheap[astarCoord]
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

type astarCoord struct {
	Coord  GridCoord
	Weight int32
	Cost   int32
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

	astar := &AStar{
		frontier: newMinheap[astarCoord](32),
		pathmap:  newPathDirMap(coordMapCols, coordMapRows),
		costmap:  newCoordMap(coordMapCols, coordMapRows),
		diagonal: config.Diagonal,
	}

	return astar
}

// BuildPath attempts to find a path between the two coordinates.
// It will use a provided Grid in combination with a GridLayer.
// The Grid is expected to store the tile tags and the GridLayer is
// used to interpret these tags.
func (astar *AStar) BuildPath(g *Grid, from, to GridCoord, l GridLayer) BuildPathResult {
	var result BuildPathResult
	if from == to {
		result.Finish = to
		return result
	}

	frontier := astar.frontier
	frontier.Reset()

	pathmap := astar.pathmap
	pathmap.Reset()

	costmap := astar.costmap
	costmap.Reset()

	frontier.Push(0, astarCoord{Coord: from})

	neighbors := neighborCardinal[:]
	if astar.diagonal {
		neighbors = neighborDiagonal[:]
	}

	distFunc := GridCoord.DistManhattan
	if astar.diagonal {
		distFunc = GridCoord.DistOctile
	}

	var fallbackCoord GridCoord
	var fallbackDist, fallbackCost int
	fallbackSet := false
	foundPath := false
	for !frontier.IsEmpty() {
		current := frontier.Pop()
		currentKey := costmap.packCoord(current.Coord)
		currentCost, ok := costmap.Get(currentKey)
		if ok && current.Cost != int32(currentCost) {
			continue
		}
		if !ok {
			currentCost = uint32(current.Cost)
		}

		if current.Coord == to {
			result.Steps = constructPath(from, to, pathmap)
			result.Finish = to
			result.Cost = int(current.Cost)
			foundPath = true
			break
		}

		dist := distFunc(to, current.Coord)
		cost := int(current.Cost)
		if !fallbackSet || dist < fallbackDist || (dist == fallbackDist && cost < fallbackCost) {
			fallbackCoord = current.Coord
			fallbackDist = dist
			fallbackCost = cost
			fallbackSet = true
		}

		for _, nb := range neighbors {
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
			priority := newNextCost + uint32(distFunc(to, next))
			nextWeighted := astarCoord{
				Coord:  next,
				Cost:   int32(newNextCost),
				Weight: int32(current.Weight + 1),
			}
			frontier.Push(int(priority), nextWeighted)
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
