package pathing_test

import (
	"bytes"
	"fmt"
	"strings"
	"testing"

	"github.com/quasilyte/pathing"
)

func Example() {
	// Grid is a "map" that stores cell info.
	const cellSize = 40
	g := pathing.NewGrid(pathing.GridConfig{
		// A 5x5 map.
		WorldWidth:  5 * cellSize,
		WorldHeight: 5 * cellSize,
		CellWidth:   cellSize,
		CellHeight:  cellSize,
	})

	// We'll use Greedy BFS pathfinder.
	bfs := pathing.NewGreedyBFS(pathing.GreedyBFSConfig{
		NumCols: uint(g.NumCols()),
		NumRows: uint(g.NumRows()),
	})

	// Tile kinds are needed to interpret the cell values.
	// Let's define some.
	const (
		tilePlain = iota
		tileForest
		tileMountain
	)

	// Grid map cells contain "tile tags"; these are basically
	// a tile enum values that should fit the 3 bits (max 8 tags per Grid).
	// The default tag is 0 (tilePlain).
	// Let's add some forests and mountains.
	//
	// The result map layout will look like this:
	// m m m m m | [m] - mountain
	// m   f   m | [f] - forest
	// m   f   m | [ ] - plain
	// m       m
	// m m m m m
	g.SetCellTile(pathing.GridCoord{X: 2, Y: 1}, tileForest)
	g.SetCellTile(pathing.GridCoord{X: 2, Y: 2}, tileForest)
	for y := 0; y < g.NumRows(); y++ {
		for x := 0; x < g.NumCols(); x++ {
			if !(y == 0 || y == g.NumRows()-1 || x == 0 || x == g.NumCols()-1) {
				continue
			}
			g.SetCellTile(pathing.GridCoord{X: x, Y: y}, tileMountain)
		}
	}

	// Now we need to tell the pathfinding library how to interpret
	// these tiles. For instance, which tiles are passable and not.
	// We do that by using layers. I'll define two layers here
	// to show you how it's possible to interpret the grid differently
	// depending on the layer.
	normalLayer := pathing.MakeGridLayer(pathing.LayerCost{
		tilePlain:    1, // passable
		tileMountain: 0, // not passable
		tileForest:   0, // not passable
	})
	flyingLayer := pathing.MakeGridLayer(pathing.LayerCost{
		tilePlain:    1,
		tileMountain: 1,
		tileForest:   1,
	})

	// Our map with markers will look like this:
	// . . m . . | [m] - mountain
	// . A f B . | [f] - forest
	// . . f . . | [.] - plain
	// . . . . . | [A] - start, [B] - finish
	startPos := pathing.GridCoord{X: 1, Y: 1}
	finishPos := pathing.GridCoord{X: 3, Y: 1}

	// Let's build a normal path first, for a non-flying unit.
	p := bfs.BuildPath(g, startPos, finishPos, normalLayer)

	fmt.Println(p.Steps.String(), "- normal layer path")

	// You can iterate the path.
	for p.Steps.HasNext() {
		fmt.Println("> step:", p.Steps.Next())
	}

	// A flying unit can go in a straight line.
	p = bfs.BuildPath(g, startPos, finishPos, flyingLayer)
	fmt.Println(p.Steps.String(), "- flying layer path")

	// A path building result has some extra information bits you might be interested in.
	// Usually, you only need the Steps part, so you can pass it around instead of the
	// entire result object
	fmt.Println(p.Finish, p.Partial)

	// You can also toggle a "blocked" path bit to make it impossible
	// to be traversed (unless a layer with blocked tile costs is used).
	g.SetCellIsBlocked(pathing.GridCoord{X: 2, Y: 1}, true)

	// This blocked our closest route for the flying unit.
	// Note that it is still known to be a forest tile.
	// m m m m m
	// m A X B m
	// m   f   m
	// m       m
	// m m m m m

	p = bfs.BuildPath(g, startPos, finishPos, flyingLayer)
	fmt.Println(p.Steps.String(), "- after blocking a tile")

	// Output:
	// {Down,Down,Right,Right,Up,Up} - normal layer path
	// > step: Down
	// > step: Down
	// > step: Right
	// > step: Right
	// > step: Up
	// > step: Up
	// {Right,Right} - flying layer path
	// {3 1} false
	// {Up,Right,Right,Down} - after blocking a tile
}

func TestGridMaps(t *testing.T) {
	tests := [][]string{
		{
			"A.............x...............................x.....x.....x....",
			"..............x.......x......xxxxxxxxxx.......x..x..x..x..x....",
			"....xxxxxxxxxxx.......x...............x.......x..x..x..x..x....",
			"......................x...............x..........x.....x......B",
		},

		{
			"A.............x..........x................x.x..x...x.x...x.....x....",
			"..............x........x.x....xxxxxxxxxx..x.xxxx...x.xx..x..x..x....",
			"....xxxxxxxxxxx...xx...x.x.............x..x.x..x...x.xx..x..x..x....",
			"..................xx...x.x.............x..x.x..x.....xx.....x......B",
		},
	}

	l := pathing.MakeGridLayer(pathing.LayerCost{1, 0, 1, 1, 0, 0, 0, 0})
	for i, test := range tests {
		parsed := testParseGrid(t, test)
		for row := 0; row < parsed.numRows; row++ {
			for col := 0; col < parsed.numCols; col++ {
				marker := test[row][col]
				cell := pathing.GridCoord{X: col, Y: row}
				switch marker {
				case 'x':
					if parsed.grid.GetCellCost(cell, l) != 0 {
						t.Fatalf("test%d: x cell is reported as free", i)
					}
				case '.', ' ':
					if parsed.grid.GetCellCost(cell, l) != 1 {
						t.Fatalf("test%d: empty/free cell is reported as marked", i)
					}
				}
			}
		}
	}
}

func TestPartialFallbackNearGoal(t *testing.T) {
	baseMap := []string{
		"xxxxxxxxxx",
		"xA......$x",
		"x......xxx",
		"xxxxxxxxxx",
	}
	// With simple min-distance fallback, the closest reachable cell to the
	// blocked goal $ at (8,1) is (7,1) — distance 1.
	wantFinish := pathing.GridCoord{X: 7, Y: 1}
	layer := pathing.MakeGridLayer(pathing.LayerCost{1, 0, 1, 1, 0, 0, 0, 0})

	builders := []struct {
		name        string
		constructor func(cols, rows uint) pathBuilder
	}{
		{
			name: "astar",
			constructor: func(cols, rows uint) pathBuilder {
				return pathing.NewAStar(pathing.AStarConfig{NumCols: cols, NumRows: rows})
			},
		},
		{
			name: "greedy_bfs",
			constructor: func(cols, rows uint) pathBuilder {
				return pathing.NewGreedyBFS(pathing.GreedyBFSConfig{NumCols: cols, NumRows: rows})
			},
		},
		{
			name: "jps",
			constructor: func(cols, rows uint) pathBuilder {
				return pathing.NewJPS(pathing.JPSConfig{NumCols: cols, NumRows: rows})
			},
		},
	}

	offsets := []pathing.GridCoord{
		{},
		{X: 150, Y: 150},
	}

	for _, builder := range builders {
		for _, offset := range offsets {
			t.Run(fmt.Sprintf("%s_offset_%d_%d", builder.name, offset.X, offset.Y), func(t *testing.T) {
				m := padBlockedMap(baseMap, offset.X, offset.Y)
				parsed := testParseGrid(t, m)
				parsed.grid.SetCellIsBlocked(parsed.dest, true)

				result := builder.constructor(uint(parsed.numCols), uint(parsed.numRows)).BuildPath(parsed.grid, parsed.start, parsed.dest, layer)
				if !result.Partial {
					t.Fatalf("expected partial result, got full path to %v", result.Finish)
				}

				expectedFinish := wantFinish.Add(offset)
				if result.Finish != expectedFinish {
					t.Fatalf("unexpected fallback finish: have %v, want %v", result.Finish, expectedFinish)
				}

				pos := parsed.start
				steps := result.Steps
				for steps.HasNext() {
					pos = pos.Move(steps.Next())
				}
				if pos != result.Finish {
					t.Fatalf("path end mismatch: finish=%v, path ends at %v", result.Finish, pos)
				}
				if parsed.dest.DistManhattan(result.Finish) != 1 {
					t.Fatalf("fallback ended at unexpected goal distance: have %d, want 1", parsed.dest.DistManhattan(result.Finish))
				}
			})
		}
	}
}

func padBlockedMap(m []string, left, top int) []string {
	if left == 0 && top == 0 {
		copied := make([]string, len(m))
		copy(copied, m)
		return copied
	}

	width := len(m[0]) + left
	row := strings.Repeat("x", width)
	result := make([]string, 0, top+len(m))
	for i := 0; i < top; i++ {
		result = append(result, row)
	}
	for _, line := range m {
		result = append(result, strings.Repeat("x", left)+line)
	}
	return result
}

type pathBuilder interface {
	BuildPath(g *pathing.Grid, from, to pathing.GridCoord, l pathing.GridLayer) pathing.BuildPathResult
}

type testGrid struct {
	start    pathing.GridCoord
	dest     pathing.GridCoord
	grid     *pathing.Grid
	pathLen  int
	numCols  int
	numRows  int
	haveRows [][]byte
}

func runPathfindTest(t *testing.T, test pathfindTestCase, constructor func(uint, uint) pathBuilder) {
	t.Helper()

	runTestOnce := func(t *testing.T, test pathfindTestCase, m []string, parseResult testGrid, impl pathBuilder, grid *pathing.Grid) {
		t.Helper()

		l := test.layer
		if l == (pathing.GridLayer{}) {
			l = pathing.MakeGridLayer(pathing.LayerCost{1, 0, 2, 3, 0, 0, 0, 0})
		}

		result := impl.BuildPath(grid, parseResult.start, parseResult.dest, l)
		path := result.Steps

		haveRows := make([][]byte, len(parseResult.haveRows))
		for i, row := range parseResult.haveRows {
			haveRows[i] = make([]byte, len(row))
			copy(haveRows[i], row)
		}

		pos := parseResult.start
		pathLen := 0
		for path.HasNext() {
			pathLen++
			d := path.Next()
			pos = pos.Move(d)
			marker := haveRows[pos.Y][pos.X]
			switch marker {
			case 'A':
				haveRows[pos.Y][pos.X] = 'A'
			case 'B':
				haveRows[pos.Y][pos.X] = '$'
			case ' ':
				t.Fatal("visited one cell more than once")
			case '.':
				haveRows[pos.Y][pos.X] = ' '
			case 'o':
				haveRows[pos.Y][pos.X] = 'O'
			case 'w':
				haveRows[pos.Y][pos.X] = 'W'
			case 'O', 'W':
				haveRows[pos.Y][pos.X] = marker
			default:
				panic(fmt.Sprintf("unexpected %c marker", marker))
			}
		}

		have := string(bytes.Join(haveRows, []byte("\n")))
		want := strings.Join(m, "\n")

		haveCost := result.Cost
		wantCost := test.cost
		if wantCost == 0 {
			wantCost = result.Steps.Len()
		}
		if haveCost != wantCost {
			t.Fatalf("costs mismatch\nmap:\n%s\n\nhave (l=%d c=%d):\n%s\n\nwant (l=%d c=%d):\n%s",
				strings.Join(m, "\n"), pathLen, haveCost, have, parseResult.pathLen, wantCost, want)
		}

		if have != want {
			t.Fatalf("paths mismatch\nmap:\n%s\n\nhave (l=%d c=%d):\n%s\n\nwant (l=%d):\n%s",
				strings.Join(m, "\n"), pathLen, result.Cost, have, parseResult.pathLen, want)
		}

		wantPartial := test.partial
		havePartial := pos != parseResult.dest && result.Partial
		if havePartial != wantPartial {
			t.Fatalf("partial flag mismatch\nmap:\n%s\nhave: %v\nwant: %v", strings.Join(m, "\n"), havePartial, wantPartial)
		}
	}

	runTestCase := func(t *testing.T, test pathfindTestCase, offset, offset2 pathing.GridCoord) {
		t.Helper()

		m := make([]string, len(test.path))
		copy(m, test.path)
		if offset.X != 0 {
			for y := range m {
				m[y] = strings.Repeat("x", offset.X) + m[y]
			}
		}
		if offset2.X != 0 {
			for y := range m {
				m[y] = m[y] + strings.Repeat("x", offset2.X)
			}
		}
		if offset.Y != 0 {
			row := strings.Repeat("x", len(m[0]))
			extraRows := make([]string, offset.Y)
			for i := range extraRows {
				extraRows[i] = row
			}
			m = append(extraRows, m...)
		}
		if offset2.Y != 0 {
			row := strings.Repeat("x", len(m[0]))
			for i := 0; i < offset2.Y; i++ {
				m = append(m, row)
			}
		}

		parseResult := testParseGrid(t, m)
		impl := constructor(uint(parseResult.numCols), uint(parseResult.numRows))
		grid := parseResult.grid

		for i := 0; i < 5; i++ {
			runTestOnce(t, test, m, parseResult, impl, grid)
		}
	}

	t.Run(test.name, func(t *testing.T) {
		runTestCase(t, test, pathing.GridCoord{}, pathing.GridCoord{})
	})
	if t.Failed() {
		return
	}

	t.Run(test.name+"with_offset_x", func(t *testing.T) {
		runTestCase(t, test, pathing.GridCoord{X: 8}, pathing.GridCoord{})
	})
	t.Run(test.name+"with_offset_x2", func(t *testing.T) {
		runTestCase(t, test, pathing.GridCoord{X: 500}, pathing.GridCoord{})
	})
	t.Run(test.name+"with_offset_y", func(t *testing.T) {
		runTestCase(t, test, pathing.GridCoord{Y: 24}, pathing.GridCoord{})
	})
	t.Run(test.name+"with_offset_y2", func(t *testing.T) {
		runTestCase(t, test, pathing.GridCoord{Y: 600}, pathing.GridCoord{})
	})
	t.Run(test.name+"with_offset_xy", func(t *testing.T) {
		runTestCase(t, test, pathing.GridCoord{X: 32, Y: 120}, pathing.GridCoord{})
	})
	t.Run(test.name+"with_offset_xy2", func(t *testing.T) {
		runTestCase(t, test, pathing.GridCoord{X: 64, Y: 32}, pathing.GridCoord{})
	})
	t.Run(test.name+"with_offset_xy3", func(t *testing.T) {
		runTestCase(t, test, pathing.GridCoord{X: 150, Y: 150}, pathing.GridCoord{})
	})

	t.Run(test.name+"with_offset2", func(t *testing.T) {
		runTestCase(t, test, pathing.GridCoord{X: 8, Y: 8}, pathing.GridCoord{X: 8, Y: 8})
	})
	t.Run(test.name+"with_offset2_2", func(t *testing.T) {
		runTestCase(t, test, pathing.GridCoord{}, pathing.GridCoord{X: 150, Y: 150})
	})
	t.Run(test.name+"with_offset2_x", func(t *testing.T) {
		runTestCase(t, test, pathing.GridCoord{X: 150}, pathing.GridCoord{X: 150})
	})
	t.Run(test.name+"with_offset2_y", func(t *testing.T) {
		runTestCase(t, test, pathing.GridCoord{Y: 150}, pathing.GridCoord{Y: 150})
	})
}

func testParseGrid(tb testing.TB, m []string) testGrid {
	tb.Helper()

	numCols := len(m[0])
	numRows := len(m)

	grid := pathing.NewGrid(pathing.GridConfig{
		WorldWidth:  32 * uint(numCols),
		WorldHeight: 32 * uint(numRows),
	})

	pathLen := 0
	var startPos pathing.GridCoord
	var destPos pathing.GridCoord
	haveRows := make([][]byte, numRows)
	for row := 0; row < numRows; row++ {
		haveRows[row] = make([]byte, numCols)
		for col := 0; col < numCols; col++ {
			marker := m[row][col]
			cell := pathing.GridCoord{X: col, Y: row}
			haveRows[row][col] = marker
			switch marker {
			case 'x':
				grid.SetCellTile(cell, 1)
			case 'o', 'O':
				grid.SetCellTile(cell, 2)
			case 'c': // Blocked o
				grid.SetCellTile(cell, 2)
				grid.SetCellIsBlocked(cell, true)
			case 'w', 'W':
				grid.SetCellTile(cell, 3)
			case 'm': // Blocked w
				grid.SetCellTile(cell, 3)
				grid.SetCellIsBlocked(cell, true)
			case 'A':
				startPos = cell
			case 'B', '$':
				if marker == '$' {
					pathLen++
				}
				destPos = cell
				haveRows[row][col] = 'B'
			case '~': // Blocked ' '
				grid.SetCellIsBlocked(cell, true)
			case ' ':
				pathLen++
				haveRows[row][col] = '.'
			}
		}
	}

	return testGrid{
		pathLen:  pathLen,
		start:    startPos,
		dest:     destPos,
		numRows:  numRows,
		numCols:  numCols,
		haveRows: haveRows,
		grid:     grid,
	}
}

type pathfindTestCase struct {
	name    string
	path    []string
	cost    int
	layer   pathing.GridLayer
	partial bool
	bench   bool
}
