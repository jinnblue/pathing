package bench

import (
	"github.com/kelindar/tile"
)

type kelindarTileTester struct {
	tc *testCase

	grid *tile.Grid[uint8]
}

func newKelindarTileTester() *kelindarTileTester {
	return &kelindarTileTester{}
}

func (t *kelindarTileTester) Init(tc *testCase) {
	t.tc = tc
	t.grid = tile.NewGridOf[uint8](int16(tc.numCols), int16(tc.numRows))

	for y, row := range tc.layout {
		for x, col := range row {
			if col == 'x' {
				t.grid.WriteAt(int16(x), int16(y), tile.Value(0))
			} else {
				t.grid.WriteAt(int16(x), int16(y), tile.Value(1))
			}
		}
	}
}

func (t *kelindarTileTester) BuildPath() ([]tile.Point, gridCoord) {
	from := tile.Point{X: int16(t.tc.start.X), Y: int16(t.tc.start.Y)}
	to := tile.Point{X: int16(t.tc.finish.X), Y: int16(t.tc.finish.Y)}
	result, _, _ := t.grid.Path(from, to, func(v tile.Value) uint16 {
		// The simplest mapping: the tile value is its cost.
		return uint16(v)
	})
	last := result[len(result)-1]
	finish := gridCoord{X: int(last.X), Y: int(last.Y)}

	return result, finish
}
