package bench

import (
	"github.com/quasilyte/pathing"
)

type quasilytePathingJPSTester struct {
	tc *testCase

	grid *pathing.Grid
	jps  *pathing.JPS
}

func newQuasilytePathingJPSTester() *quasilytePathingJPSTester {
	return &quasilytePathingJPSTester{}
}

func (t *quasilytePathingJPSTester) Init(tc *testCase) {
	t.tc = tc
	t.jps = pathing.NewJPS(pathing.JPSConfig{
		NumCols: uint(tc.numCols),
		NumRows: uint(tc.numRows),
	})
	width := tc.cellWidth * tc.numCols
	height := tc.cellHeight * tc.numRows
	t.grid = pathing.NewGrid(pathing.GridConfig{
		WorldWidth:  uint(width),
		WorldHeight: uint(height),
	})
	fillPathingGrid(t.grid, tc)
}

func (t *quasilytePathingJPSTester) BuildPath() (pathing.GridPath, gridCoord) {
	from := pathing.GridCoord{X: t.tc.start.X, Y: t.tc.start.Y}
	to := pathing.GridCoord{X: t.tc.finish.X, Y: t.tc.finish.Y}
	result := t.jps.BuildPath(t.grid, from, to, pathingLayer)
	return result.Steps, gridCoord{X: result.Finish.X, Y: result.Finish.Y}
}
