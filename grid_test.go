package pathing

import (
	"fmt"
	"math/rand/v2"
	"reflect"
	"strings"
	"testing"
)

type testPos struct {
	X float64
	Y float64
}

func (p testPos) XY() (float64, float64) {
	return p.X, p.Y
}

func TestGridLayerBlocked(t *testing.T) {
	tests := []struct {
		data  [2]LayerCost
		index int
		want  uint8
	}{
		{
			data:  [2]LayerCost{{1: 11}, {2: 22}},
			index: 1,
			want:  11,
		},
		{
			data:  [2]LayerCost{{1: 11}, {2: 22}},
			index: 0,
			want:  0,
		},
		{
			data:  [2]LayerCost{{1: 11}, {2: 22}},
			index: 2 | (1 << 3),
			want:  22,
		},
		{
			data:  [2]LayerCost{{1: 11}, {2: 22}},
			index: 1 | (1 << 3),
			want:  0,
		},
		{
			data:  [2]LayerCost{{1: 11}, {}},
			index: 2 | (1 << 3),
			want:  0,
		},
		{
			data:  [2]LayerCost{{1: 11}, {}},
			index: 1 | (1 << 3),
			want:  0,
		},
	}

	for _, test := range tests {
		l := MakeGridLayerWithBlocked(test.data[0], test.data[1])
		want := test.want
		have := l.getFast(uint8(test.index))
		if want != have {
			t.Fatalf("layer %v getFast(%d %04b):\nhave: %v\nwant: %v", test.data, test.index, test.index, have, want)
		}
	}
}

func TestGridLayer(t *testing.T) {
	tests := []LayerCost{
		{0, 0, 0, 0, 0, 0, 1, 0},
		{0, 0, 0, 1, 0, 0, 2, 0},
		{1, 0, 0, 0, 0, 0, 3, 0},
		{1, 1, 1, 1, 0, 0, 4, 0},
		{10, 0, 10, 0, 0, 0, 0, 0xff},
		{1, 2, 3, 4, 0, 0, 0, 0},
		{4, 3, 2, 1, 0, 0, 0, 0},
		{0xff, 0xff, 0xff, 0xff, 0, 0xff, 0, 0},
		{100, 0xff, 0xff, 100, 0, 0, 0xfe, 0xfa},
		{24, 53, 21, 99, 0, 0, 0, 0},
		{99, 145, 9, 0, 0, 0, 0, 0},
		{10, 0, 20, 30, 0, 0, 0, 0},
	}

	for _, test := range tests {
		l := MakeGridLayer(test)
		for i := uint8(0); i <= 7; i++ {
			want := test[i]
			have := l.Get(i)
			if want != have {
				t.Fatalf("(%v).Get(%d): have %v, want %v", test, i, have, want)
			}
			have2 := l.getFast(i)
			if want != have2 {
				t.Fatalf("(%v).getFast(%d): have %v, want %v", test, i, have, want)
			}
		}
	}
}

func TestEmptyGrid(t *testing.T) {
	p := NewGrid(GridConfig{
		WorldWidth:  0,
		WorldHeight: 0,
		CellWidth:   32,
		CellHeight:  32,
	})
	cols := p.NumCols()
	rows := p.NumRows()
	if rows != 0 || cols != 0 {
		t.Fatalf("expected [0,0] size, got [%d,%d]", cols, rows)
	}

	positions := []testPos{
		{X: 0, Y: 0},
		{X: 98, Y: 0},
		{X: 0, Y: 98},
		{X: -98, Y: 0},
		{X: 0, Y: -98},
		{X: 2045, Y: 3525},
		{X: -2045, Y: -3525},
	}

	l := MakeGridLayer(LayerCost{1, 0, 1, 1, 0, 0, 0, 0})
	for _, pos := range positions {
		if p.GetCellCost(p.PosToCoord(pos.XY()), l) != 0 {
			t.Fatalf("empty grid reported %v as free", pos)
		}
		if p.GetCellIsBlocked(p.PosToCoord(pos.XY())) {
			t.Fatalf("empty grid reported %v as blocked", pos)
		}
	}
}

func TestGridOutOfBounds(t *testing.T) {
	p := NewGrid(GridConfig{
		WorldWidth:  48 * 4,
		WorldHeight: 48 * 4,
		CellWidth:   48,
		CellHeight:  48,
	})
	cols := p.NumCols()
	rows := p.NumRows()
	if rows != 4 || cols != 4 {
		t.Fatalf("expected [4,4] size, got [%d,%d]", cols, rows)
	}

	coords := []GridCoord{
		{X: 0, Y: -1},
		{X: -1, Y: -1},
		{X: -1, Y: 0},
		{X: -40, Y: -40},

		{X: 4, Y: 0},
		{X: 5, Y: 0},
		{X: 50, Y: 0},
		{X: 0, Y: 4},
		{X: 0, Y: 5},
		{X: 0, Y: 50},
		{X: 4, Y: 4},
		{X: 5, Y: 5},
		{X: 50, Y: 50},

		{X: 2, Y: 10},
		{X: 3, Y: 10},
		{X: 10, Y: 2},
		{X: 10, Y: 3},
		{X: 2, Y: -10},
		{X: 3, Y: -10},
		{X: -10, Y: 2},
		{X: -10, Y: 3},
	}

	l := MakeGridLayer(LayerCost{1, 0, 1, 1, 0, 0, 0, 0})
	for _, coord := range coords {
		if p.GetCellCost(coord, l) != 0 {
			t.Fatalf("grid reported out-of-bounds %v as free", coord)
		}
		if p.GetCellIsBlocked(coord) {
			t.Fatalf("grid reported out-of-bounds %v as blocked", coord)
		}
	}
}

func TestRandFillGrid(t *testing.T) {
	for try := 0; try < 15; try++ {
		p := NewGrid(GridConfig{
			WorldWidth:  10 * 32,
			WorldHeight: 10 * 32,
		})

		rng := rand.New(rand.NewPCG(1772248727283, 0))
		layers := make([][]uint8, 10)
		for i := range layers {
			layers[i] = make([]uint8, 10)
			for j := range layers[i] {
				layers[i][j] = uint8(rng.IntN(4))
			}
		}

		values := LayerCost{10, 0, 20, 30, 0, 0, 0, 0}
		values2 := LayerCost{0, 1, 2, 3, 0, 0, 0, 0}
		l := MakeGridLayer(values)
		l2 := MakeGridLayer(values2)
		// Pre-set some tiles to a blocked state.
		blockedCells := make(map[GridCoord]bool)
		for y := 0; y < 10; y++ {
			for x := 0; x < 10; x++ {
				c := GridCoord{X: x, Y: y}
				if rng.Float64() <= 0.1 {
					p.SetCellIsBlocked(c, true)
					blockedCells[c] = true
				}
			}
		}
		// Runs are meant to test that setting individual cells
		// does not invalidate other cells in the process.
		for run := 0; run < 4; run++ {
			for y := 0; y < 10; y++ {
				for x := 0; x < 10; x++ {
					c := GridCoord{X: x, Y: y}
					tag := layers[y][x]
					p.SetCellTile(c, tag)
					v := p.GetCellCost(c, l)
					if blockedCells[c] {
						if v != 0 {
							t.Fatalf("grid[%d][%d] blocked cell cost is not 0 (%v)", y, x, v)
						}
					} else {
						if v != values[tag] {
							t.Fatalf("grid[%d][%d] value mismatch: have %v, want %v", y, x, v, values[tag])
						}
					}
					v2 := p.GetCellCost(c, l2)
					if blockedCells[c] {
						if v2 != 0 {
							t.Fatalf("grid[%d][%d] blocked2 cell cost is not 0 (%v)", y, x, v2)
						}
					} else {
						if v2 != values2[tag] {
							t.Fatalf("grid[%d][%d] value2 mismatch: have %v, want %v", y, x, v2, values2[tag])
						}
					}
					tag2, blocked := p.GetCellTile2(c)
					if tag2 != tag {
						t.Fatalf("grid[%d][%d] GetCellTile2 tag mismatch: have %v, want %v", y, x, tag2, tag)
					}
					wantBlocked := blockedCells[c]
					if blocked != wantBlocked {
						t.Fatalf("grid[%d][%d] GetCellTile2 blocked mismatch: have %v, want %v", y, x, blocked, wantBlocked)
					}
					blocked2 := p.GetCellIsBlocked(c)
					if blocked2 != wantBlocked {
						t.Fatalf("grid[%d][%d] GetCellIsBlocked blocked mismatch: have %v, want %v", y, x, blocked2, wantBlocked)
					}
					tag3 := p.GetCellTile(c)
					if tag3 != tag {
						t.Fatalf("grid[%d][%d] GetCellTile tag mismatch: have %v, want %v", y, x, tag3, tag)
					}
				}
			}
		}
	}
}

func TestGridValueChange(t *testing.T) {
	p := NewGrid(GridConfig{
		WorldWidth:  4 * 64,
		WorldHeight: 4 * 64,
		CellWidth:   64,
		CellHeight:  64,
	})
	layerValues := LayerCost{1, 0, 5, 10, 0, 0, 0, 0}
	l := MakeGridLayer(layerValues)
	coord := GridCoord{X: 1, Y: 1}

	probes := []uint8{
		3,
		3,
		0,
		1,
		3,
		2,
		1,
		0,
		3,
		3,
		0,
		0,
		1,
		0,
	}

	for _, probe := range probes {
		want := layerValues[probe]
		p.SetCellTile(coord, probe)
		if have := p.GetCellCost(coord, l); have != want {
			t.Fatalf("SetCellTile(%v, %v): have %v, want %v", coord, probe, have, want)
		}
	}
}

func TestSmallGrid(t *testing.T) {
	p := NewGrid(GridConfig{
		WorldWidth:  9 * 32,
		WorldHeight: 6 * 32,
	})

	numCols := p.NumCols()
	numRows := p.NumRows()
	if numCols != 9 || numRows != 6 {
		t.Fatalf("expected [9,6] size, got [%d,%d]", numCols, numRows)
	}

	values := LayerCost{10, 0, 20, 30, 0, 0, 0, 0}
	l := MakeGridLayer(values)
	numCells := numCols * numRows
	for y := 0; y < numRows; y++ {
		for x := 0; x < numCols; x++ {
			c := GridCoord{X: x, Y: y}
			if p.GetCellCost(c, l) != 10 {
				t.Fatalf("empty grid (size %d) reports in-bounds %v as marked", numCells, c)
			}
		}
	}

	for y := 0; y < numRows; y++ {
		for x := 0; x < numCols; x++ {
			c := GridCoord{X: x, Y: y}
			tag := uint8((y*numCols + x) % 4)
			p.SetCellTile(c, tag)
		}
	}

	for y := 0; y < numRows; y++ {
		for x := 0; x < numCols; x++ {
			c := GridCoord{X: x, Y: y}
			tag := uint8((y*numCols + x) % 4)
			v := values[tag]
			if actual := p.GetCellCost(c, l); actual != v {
				t.Fatalf("expected %v value to be %v, got %v", c, v, actual)
			}
		}
	}
}

func TestGrid(t *testing.T) {
	p := NewGrid(GridConfig{
		WorldWidth:  1856,
		WorldHeight: 1856,
	})

	tests := []GridCoord{
		{X: 0, Y: 0},
		{X: 1, Y: 0},
		{X: 0, Y: 1},
		{X: 1, Y: 1},
		{X: 4, Y: 0},
		{X: 0, Y: 4},
		{X: 8, Y: 0},
		{X: 0, Y: 8},
		{X: 9, Y: 0},
		{X: 0, Y: 9},
		{X: 9, Y: 9},
		{X: 30, Y: 31},
		{X: 31, Y: 30},
		{X: 0, Y: 14},
		{X: 14, Y: 0},
	}

	l := MakeGridLayer(LayerCost{0, 1, 2, 3, 0, 0, 0, 0})
	for i, test := range tests {
		if p.GetCellCost(test, l) != 0 {
			t.Fatalf("GetCellCost(%d, %d) returned true before it was set", test.X, test.Y)
		}
		p.SetCellTile(test, 1)
		if p.GetCellCost(test, l) != 1 {
			t.Fatalf("GetCellCost(%d, %d) returned false after it was set", test.X, test.Y)
		}
		for j := i + 1; j < len(tests); j++ {
			otherTest := tests[j]
			if p.GetCellCost(otherTest, l) != 0 {
				t.Fatalf("unrelated GetCellCost(%d, %d) returned true after (%d, %d) was set", otherTest.X, otherTest.Y, test.X, test.Y)
			}
		}
	}
}

func testParsePath(s string) GridPath {
	s = s[1 : len(s)-1] // Drop "{}"
	if s == "" {
		return GridPath{}
	}
	var directions []Direction
	for _, part := range strings.Split(s, ",") {
		switch part {
		case "Right":
			directions = append(directions, DirRight)
		case "Down":
			directions = append(directions, DirDown)
		case "Left":
			directions = append(directions, DirLeft)
		case "Up":
			directions = append(directions, DirUp)
		default:
			panic("unexpected part: " + part)
		}
	}
	return MakeGridPath(directions...)
}

func TestGridPathString(t *testing.T) {
	tests := []string{
		"{}",
		"{Left}",
		"{Left,Right}",
		"{Right,Left}",
		"{Down,Down,Down,Up}",
		"{Left,Right,Up,Down}",
		"{Left,Right,Right,Right,Left}",
		"{Up,Up,Down,Down,Left,Left,Right,Right,Down,Down}",
		"{Up,Up,Down,Down,Left,Left,Right,Right,Down,Down,Down,Left,Up,Right}",
		"{Left,Left,Left,Left,Left,Left,Left,Left,Left,Left,Left,Left,Left,Left}",
		"{Left,Left,Left,Left,Left,Left,Left,Left,Left,Left,Left,Left,Left,Right}",
		"{Up,Left,Left,Left,Left,Left,Left,Left,Left,Left,Left,Left,Left,Left}",
	}

	for _, test := range tests {
		p := testParsePath(test)
		if p.String() != test {
			t.Fatalf("results mismatched:\nhave: %q\nwant: %q", p.String(), test)
		}
	}
}

func TestGridPath(t *testing.T) {
	tests := [][]Direction{
		{},
		{DirLeft},
		{DirDown},
		{DirLeft, DirRight, DirUp},
		{DirLeft, DirLeft, DirLeft},
		{DirDown, DirDown, DirDown},
		{DirDown, DirUp, DirLeft, DirRight, DirLeft, DirRight},
		{DirDown, DirLeft, DirLeft, DirLeft, DirLeft, DirDown},
		{DirRight, DirRight, DirRight, DirRight, DirRight, DirRight, DirRight},
		{DirDown, DirRight, DirRight, DirDown, DirRight, DirUp, DirRight, DirLeft},
	}

	for i, directions := range tests {
		p := MakeGridPath(directions...)
		reconstructed := []Direction{}
		for p.HasNext() {
			reconstructed = append(reconstructed, p.Next())
		}
		if !reflect.DeepEqual(directions, reconstructed) {
			t.Fatalf("test%d paths mismatch", i)
		}
	}

	r := rand.New(rand.NewPCG(1772248727283, 0))
	for i := 0; i < 100; i++ {
		size := r.IntN(20) + 10
		directions := []Direction{}
		for j := 0; j < size; j++ {
			d := r.IntN(4)
			directions = append(directions, Direction(d))
		}
		p := MakeGridPath(directions...)
		reconstructed := []Direction{}
		for p.HasNext() {
			reconstructed = append(reconstructed, p.Next())
		}
		if !reflect.DeepEqual(directions, reconstructed) {
			t.Fatalf("test%d paths mismatch", i)
		}

		p.Rewind()
		reconstructed = reconstructed[:0]
		for p.HasNext() {
			reconstructed = append(reconstructed, p.Next())
		}
		if !reflect.DeepEqual(directions, reconstructed) {
			t.Fatalf("test%d paths mismatch", i)
		}
	}
}

func BenchmarkMakeGridLayer(b *testing.B) {
	vals := LayerCost{1, 0, 2, 3, 4, 5, 6, 7}
	b.ResetTimer()
	var l GridLayer
	for i := 0; i < b.N; i++ {
		l = MakeGridLayer(vals)
	}
	want := GridLayer{uint64(506097522914230273), uint64(0)}
	if l != want {
		b.Fail()
	}
}

func BenchmarkMakeGridLayerByFor(b *testing.B) {
	vals := LayerCost{1, 0, 2, 3, 4, 5, 6, 7}
	b.ResetTimer()
	var l GridLayer
	for i := 0; i < b.N; i++ {
		l = MakeGridLayerByFor(vals)
	}
	want := GridLayer{uint64(506097522914230273), uint64(0)}
	if l != want {
		b.Fail()
	}
}

func MakeGridLayerByFor(costs LayerCost) GridLayer {
	return MakeGridLayerWithBlockedByFor(costs, LayerCost{})
}

func MakeGridLayerWithBlockedByFor(costs LayerCost, blocked LayerCost) (l GridLayer) {
	tileMapping := uint64(0)
	for i := range costs {
		tileMapping |= uint64(costs[i]) << (i * 8)
	}
	blockedMapping := uint64(0)
	for i := range blocked {
		blockedMapping |= uint64(blocked[i]) << (i * 8)
	}
	return GridLayer{tileMapping, blockedMapping}
}

func BenchmarkPathgridGetCellCost(b *testing.B) {
	p := NewGrid(GridConfig{WorldWidth: 1856, WorldHeight: 1856})
	l := MakeGridLayer(LayerCost{1, 0, 2, 3, 0, 0, 0, 0})
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		p.GetCellCost(GridCoord{14, 5}, l)
	}
}

func BenchmarkPathgridSetCellTile(b *testing.B) {
	p := NewGrid(GridConfig{WorldWidth: 1856, WorldHeight: 1856})
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		p.SetCellTile(GridCoord{14, 5}, 1)
	}
}

func BenchmarkCoordMapReset(b *testing.B) {
	sizes := []int{32, 96}
	for i := range sizes {
		size := sizes[i]
		b.Run(fmt.Sprintf("size%d", size), func(b *testing.B) {
			m := newCoordMap(size, size)
			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				m.Reset()
			}
		})
	}
}

func BenchmarkCoordMapSet(b *testing.B) {
	m := newCoordMap(8, 8)
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		m.Reset()
		for j := 0; j < 8; j++ {
			m.Set(uint(j), uint32(DirUp))
		}
	}
}

func BenchmarkCoordMapGet(b *testing.B) {
	m := newCoordMap(8, 8)
	for j := 0; j < 8; j++ {
		m.Set(uint(j), uint32(DirUp))
	}
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		for j := 0; j < 8; j++ {
			_, _ = m.Get(uint(j))
		}
	}
}

func TestEmptyCoordMap(t *testing.T) {
	m := newCoordMap(0, 0)

	coords := []GridCoord{
		{0, 0},
		{0, 1},
		{1, 0},
		{1, 1},

		{0, -1},
		{-1, 0},
		{-1, -1},

		{0, 10},
		{10, 0},
		{10, 10},

		{100, 100},
		{-100, -100},
	}

	for _, coord := range coords {
		if v, ok := m.Get(m.packCoord(coord)); v != 0 || ok {
			t.Fatalf("empty coord map returns invalid result for %v", coord)
		}
		if m.Contains(m.packCoord(coord)) {
			t.Fatalf("empty coord map returns invalid result for %v", coord)
		}
	}
}

func TestCoordMap(t *testing.T) {
	m := newCoordMap(32, 32)

	coords := []GridCoord{
		{0, 0},
		{0, 1},
		{1, 0},
		{1, 1},

		{0, 10},
		{10, 0},
		{10, 10},
		{10, 30},

		{31, 31},
	}

	for i, coord := range coords {
		if v, ok := m.Get(m.packCoord(coord)); v != 0 || ok {
			t.Fatalf("Get(%v) expected to give 0 before insertion", coord)
		}
		if m.Contains(m.packCoord(coord)) {
			t.Fatalf("Contains(%v) expected to give false before insertion", coord)
		}
		dir := Direction(i % 4)
		m.Set(m.packCoord(coord), uint32(dir))
		for j := 0; j < 3; j++ {
			if v, ok := m.Get(m.packCoord(coord)); !ok || Direction(v) != dir {
				t.Fatalf("Get(%v) gives %s, expected %s", coord, Direction(v), dir)
			}
		}
		dir = Direction(3 - (i % 4))
		m.Set(m.packCoord(coord), uint32(dir))
		for j := 0; j < 3; j++ {
			if v, ok := m.Get(m.packCoord(coord)); !ok || Direction(v) != dir {
				t.Fatalf("Get(%v) gives %s, expected %s", coord, Direction(v), dir)
			}
		}
		for _, otherCoord := range coords[i:] {
			if coord == otherCoord {
				continue
			}
			if m.Contains(m.packCoord(otherCoord)) {
				t.Fatalf("unrelated Contains(%v) after Set(%v) reports true", otherCoord, coord)
			}
		}
	}
}
