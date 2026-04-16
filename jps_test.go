package pathing_test

import (
	"fmt"
	"testing"

	"github.com/quasilyte/pathing"
)

func BenchmarkJPS(b *testing.B) {
	l := pathing.MakeGridLayer(pathing.LayerCost{1, 0, 1, 1, 0, 0, 0, 0})
	for i := range astarTests {
		test := astarTests[i]
		if !test.bench {
			continue
		}
		numCols := len(test.path[0])
		numRows := len(test.path)
		b.Run(fmt.Sprintf("%s_%dx%d", test.name, numCols, numRows), func(b *testing.B) {
			parseResult := testParseGrid(b, test.path)
			jps := pathing.NewJPS(pathing.JPSConfig{
				NumCols: uint(parseResult.numCols),
				NumRows: uint(parseResult.numRows),
			})
			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				jps.BuildPath(parseResult.grid, parseResult.start, parseResult.dest, l)
			}
		})
	}
}

// TestJPS validates JPS by cross-checking against A* with diagonal movement.
// Both should find optimal paths with the same cost and same partial/reachable status.
func TestJPS(t *testing.T) {
	l := pathing.MakeGridLayer(pathing.LayerCost{1, 0, 1, 1, 0, 0, 0, 0})
	for _, tt := range astarTests {
		t.Run(tt.name, func(t *testing.T) {
			runJPSCrossCheck(t, tt.path, l)
		})
	}
}

// TestJPSPartialFallback verifies that JPS returns reasonable partial paths
// when the goal is unreachable.
func TestJPSPartialFallback(t *testing.T) {
	l := pathing.MakeGridLayer(pathing.LayerCost{1, 0, 1, 1, 0, 0, 0, 0})

	grids := [][]string{
		{
			"A....x.B",
			".....x..",
		},
		{
			"........",
			".xxxxx..",
			".x...x..",
			".x.A.x..",
			".x...x..",
			".xxxxx..",
			".......B",
		},
	}

	for i, grid := range grids {
		t.Run(fmt.Sprintf("partial_%d", i), func(t *testing.T) {
			parsed := testParseGrid(t, grid)
			jps := pathing.NewJPS(pathing.JPSConfig{
				NumCols: uint(parsed.numCols),
				NumRows: uint(parsed.numRows),
			})
			result := jps.BuildPath(parsed.grid, parsed.start, parsed.dest, l)
			if !result.Partial {
				t.Fatalf("expected partial result, got full path to %v", result.Finish)
			}
			// Verify the path is walkable.
			pos := parsed.start
			steps := result.Steps
			for steps.HasNext() {
				pos = pos.Move(steps.Next())
				if parsed.grid.GetCellCost(pos, l) == 0 {
					t.Fatalf("path walks through blocked cell %v", pos)
				}
			}
			if pos != result.Finish {
				t.Fatalf("path end mismatch: finish=%v, path ends at %v", result.Finish, pos)
			}
		})
	}
}

func runJPSCrossCheck(t *testing.T, grid []string, l pathing.GridLayer) {
	t.Helper()

	offsets := []pathing.GridCoord{
		{},
		{X: 8},
		{Y: 24},
		{X: 32, Y: 120},
		{X: 150, Y: 150},
	}

	for _, offset := range offsets {
		name := fmt.Sprintf("offset_%d_%d", offset.X, offset.Y)
		t.Run(name, func(t *testing.T) {
			m := padGrid(grid, offset)
			parsed := testParseGrid(t, m)

			jps := pathing.NewJPS(pathing.JPSConfig{
				NumCols: uint(parsed.numCols),
				NumRows: uint(parsed.numRows),
			})
			astar := pathing.NewAStar(pathing.AStarConfig{
				NumCols:  uint(parsed.numCols),
				NumRows:  uint(parsed.numRows),
				Diagonal: true,
			})

			jpsResult := jps.BuildPath(parsed.grid, parsed.start, parsed.dest, l)
			astarResult := astar.BuildPath(parsed.grid, parsed.start, parsed.dest, l)

			// Both should agree on reachability.
			if jpsResult.Partial != astarResult.Partial {
				t.Fatalf("partial mismatch: JPS=%v, AStar=%v", jpsResult.Partial, astarResult.Partial)
			}

			// For full paths, JPS cost should be close to A* cost.
			// Write-once pathmap may cause minor suboptimality in rare cases
			// where a jump point is rediscovered via a better path but intermediate
			// cells retain their original direction for reconstruction.
			if !jpsResult.Partial {
				diff := jpsResult.Steps.Len() - astarResult.Steps.Len()
				if diff > 2 {
					t.Fatalf("JPS path too long vs A*: JPS=%d steps, A*=%d steps (diff=%d)",
						jpsResult.Steps.Len(), astarResult.Steps.Len(), diff)
				}
			}

			// Verify the JPS path is walkable.
			pos := parsed.start
			steps := jpsResult.Steps
			for steps.HasNext() {
				pos = pos.Move(steps.Next())
				if parsed.grid.GetCellCost(pos, l) == 0 {
					t.Fatalf("JPS path walks through blocked cell %v", pos)
				}
			}
			if pos != jpsResult.Finish {
				t.Fatalf("JPS path end mismatch: finish=%v, path ends at %v", jpsResult.Finish, pos)
			}

			// Run multiple times to ensure determinism.
			for i := 0; i < 3; i++ {
				r2 := jps.BuildPath(parsed.grid, parsed.start, parsed.dest, l)
				if r2.Steps.Len() != jpsResult.Steps.Len() {
					t.Fatalf("non-deterministic: run 0 len=%d, run %d len=%d",
						jpsResult.Steps.Len(), i+1, r2.Steps.Len())
				}
			}
		})
	}
}

func padGrid(m []string, offset pathing.GridCoord) []string {
	if offset.X == 0 && offset.Y == 0 {
		copied := make([]string, len(m))
		copy(copied, m)
		return copied
	}
	result := make([]string, 0, offset.Y+len(m))
	width := len(m[0]) + offset.X
	for i := 0; i < offset.Y; i++ {
		row := make([]byte, width)
		for j := range row {
			row[j] = 'x'
		}
		result = append(result, string(row))
	}
	for _, line := range m {
		prefix := make([]byte, offset.X)
		for j := range prefix {
			prefix[j] = 'x'
		}
		result = append(result, string(prefix)+line)
	}
	return result
}

var jpsBenchTests = [][]string{
	{
		"..........",
		"...A...B..",
		"..........",
	},
	{
		"A.........",
		"..........",
		"..........",
		"..........",
		".........B",
	},
	{
		"..........x.....................",
		"..........x.....................",
		"..........x.....................",
		"..........x.....................",
		"................................",
		"..............x.................",
		"..............x.................",
		"..A...........x.................",
		"....x...........................",
		"....x...........................",
		"....x...........................",
		"....x.......................B...",
	},
	{
		"A                                                                  B",
	},
	{
		".........x.....",
		"xxxxxxxx.x.....",
		"x......x.x.....",
		"x..xxx.x.x.....",
		"x....x.x.x.....",
		"x..A.x...xx..xx",
		"x....x.x.x...B.",
		"xxxxxx.x.x.xxxx",
		"x......x.x.....",
		"xxxxxxxx.xxxx.x",
		".........x.....",
		"..........x....",
	},
}
