package bench

// Benchmarks using Moving AI Lab map/scenario files.
// Map format: https://www.movingai.com/benchmarks/formats.html
//
// The testdata directory contains:
//   - ht_0_hightown_n.map      – 514×514 octile grid
//   - ht_0_hightown_n.map.scen – 1400 start/goal pairs ordered by path length
//
// Benchmarks use 8-directional (diagonal) movement, matching the octile movement
// model used by the Moving AI benchmark suite.
//
// Each benchmark op = one pathfind call.  Scenarios are partitioned into three
// equal thirds (short / medium / long) so callers can compare how algorithm
// performance scales with path length.

import (
	"bufio"
	"fmt"
	"os"
	"strconv"
	"strings"
	"testing"

	"github.com/quasilyte/pathing"
)

// BenchmarkMovingAIAStar benchmarks A* on the ht_0_hightown_n map using three
// scenario sub-sets of roughly equal size:
//
//   - short  – first third  (easiest, shortest paths)
//   - medium – middle third
//   - long   – last third   (hardest, longest paths)
func BenchmarkMovingAIAStar(b *testing.B) {
	n := len(htScenarios)
	t := n / 3
	b.Run("short", func(b *testing.B) { runMovingAIAStar(b, htScenSlice(0, t)) })
	b.Run("medium", func(b *testing.B) { runMovingAIAStar(b, htScenSlice(t, 2*t)) })
	b.Run("long", func(b *testing.B) { runMovingAIAStar(b, htScenSlice(2*t, n)) })
}

// BenchmarkMovingAIBFS benchmarks Greedy BFS on the same map and scenario
// sub-sets as BenchmarkMovingAIAStar.
func BenchmarkMovingAIBFS(b *testing.B) {
	n := len(htScenarios)
	t := n / 3
	b.Run("short", func(b *testing.B) { runMovingAIBFS(b, htScenSlice(0, t)) })
	b.Run("medium", func(b *testing.B) { runMovingAIBFS(b, htScenSlice(t, 2*t)) })
	b.Run("long", func(b *testing.B) { runMovingAIBFS(b, htScenSlice(2*t, n)) })
}

// movingAIScenario holds one start/goal pair from a .scen file.
type movingAIScenario struct {
	startX, startY int
	goalX, goalY   int
	cost           float64
}

// loadMovingAIGrid parses a Moving AI .map file and returns a pathing.Grid
// together with the grid dimensions (cols, rows).
// Passable tile characters: '.', 'G', 'S', 'W'.
// All other characters (e.g. '@', 'T', 'O') are treated as walls (tile tag 1).
func loadMovingAIGrid(mapPath string) (*pathing.Grid, int, int, error) {
	f, err := os.Open(mapPath)
	if err != nil {
		return nil, 0, 0, err
	}
	defer f.Close()

	var cols, rows int
	var mapRows []string

	scanner := bufio.NewScanner(f)
	scanner.Buffer(make([]byte, 2*1024*1024), 2*1024*1024)

	inMap := false
	for scanner.Scan() {
		line := scanner.Text()
		if inMap {
			mapRows = append(mapRows, line)
			continue
		}
		if line == "map" {
			inMap = true
			continue
		}
		if fields := strings.Fields(line); len(fields) == 2 {
			switch fields[0] {
			case "width":
				cols, _ = strconv.Atoi(fields[1])
			case "height":
				rows, _ = strconv.Atoi(fields[1])
			}
		}
	}
	if err := scanner.Err(); err != nil {
		return nil, 0, 0, err
	}

	// Use 1×1 cells so that grid coordinates match map coordinates directly.
	g := pathing.NewGrid(pathing.GridConfig{
		WorldWidth:  uint(cols),
		WorldHeight: uint(rows),
		CellWidth:   1,
		CellHeight:  1,
	})

	for y, row := range mapRows {
		for x := 0; x < len(row); x++ {
			ch := row[x]
			passable := ch == '.' || ch == 'G' || ch == 'S' || ch == 'W'
			if !passable {
				g.SetCellTile(pathing.GridCoord{X: x, Y: y}, 1) // tile tag 1 → cost 0 (blocked)
			}
		}
	}

	return g, cols, rows, nil
}

// loadMovingAIScenarios parses a Moving AI .map.scen file and returns all
// start/goal pairs.  The file is tab-separated with columns:
//
//	bucket  map  map_w  map_h  start_x  start_y  goal_x  goal_y  optimal_len
func loadMovingAIScenarios(scenPath string) ([]movingAIScenario, error) {
	f, err := os.Open(scenPath)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	var scenarios []movingAIScenario
	scanner := bufio.NewScanner(f)
	for scanner.Scan() {
		line := scanner.Text()
		if line == "" || strings.HasPrefix(line, "version") {
			continue
		}
		fields := strings.Split(line, "\t")
		if len(fields) < 9 {
			continue
		}
		sx, _ := strconv.Atoi(fields[4])
		sy, _ := strconv.Atoi(fields[5])
		gx, _ := strconv.Atoi(fields[6])
		gy, _ := strconv.Atoi(fields[7])
		cost, _ := strconv.ParseFloat(fields[8], 64)
		scenarios = append(scenarios, movingAIScenario{
			startX: sx, startY: sy,
			goalX: gx, goalY: gy,
			cost: cost,
		})
	}
	return scenarios, scanner.Err()
}

// Package-level state loaded once for all Moving AI benchmarks.
var (
	htGrid      *pathing.Grid
	htGridCols  int
	htGridRows  int
	htScenarios []movingAIScenario
)

func init() {
	var err error
	htGrid, htGridCols, htGridRows, err = loadMovingAIGrid("testdata/ht_0_hightown_n.map")
	if err != nil {
		panic(fmt.Sprintf("movingai: failed to load map: %v", err))
	}
	htScenarios, err = loadMovingAIScenarios("testdata/ht_0_hightown_n.map.scen")
	if err != nil {
		panic(fmt.Sprintf("movingai: failed to load scenarios: %v", err))
	}
	if len(htScenarios) == 0 {
		panic("movingai: scenario file is empty")
	}
}

// htLayer reuses the same layer definition as the rest of the bench suite:
// tile tag 0 → cost 1 (passable), tag 1 → cost 0 (blocked).
//
// pathingLayer is already declared in quasilyte_pathing_bfs_test.go;
// reference it directly instead of redefining.

func runMovingAIAStar(b *testing.B, scenarios []movingAIScenario) {
	b.Helper()
	astar := pathing.NewAStar(pathing.AStarConfig{
		NumCols:  uint(htGridCols),
		NumRows:  uint(htGridRows),
		Diagonal: true,
	})
	b.ResetTimer()
	costs := 0
	baseCosts := float64(0)
	for i := 0; i < b.N; i++ {
		sc := scenarios[i%len(scenarios)]
		from := pathing.GridCoord{X: sc.startX, Y: sc.startY}
		to := pathing.GridCoord{X: sc.goalX, Y: sc.goalY}
		ret := astar.BuildPath(htGrid, from, to, pathingLayer)
		if ret.Finish != to {
			b.Fail()
		}
		costs += ret.Cost
		baseCosts += sc.cost

	}
	b.ReportMetric(float64(costs)/float64(b.N), "costs/op")
	b.ReportMetric(baseCosts/float64(b.N), "base_costs/op")
}

func runMovingAIBFS(b *testing.B, scenarios []movingAIScenario) {
	b.Helper()
	bfs := pathing.NewGreedyBFS(pathing.GreedyBFSConfig{
		NumCols:  uint(htGridCols),
		NumRows:  uint(htGridRows),
		Diagonal: true,
	})
	b.ResetTimer()
	costs := 0
	baseCosts := float64(0)
	for i := 0; i < b.N; i++ {
		sc := scenarios[i%len(scenarios)]
		from := pathing.GridCoord{X: sc.startX, Y: sc.startY}
		to := pathing.GridCoord{X: sc.goalX, Y: sc.goalY}
		ret := bfs.BuildPath(htGrid, from, to, pathingLayer)
		if ret.Finish != to {
			b.Fail()
		}
		costs += ret.Cost
		baseCosts += sc.cost
	}
	b.ReportMetric(float64(costs)/float64(b.N), "costs/op")
	b.ReportMetric(baseCosts/float64(b.N), "base_costs/op")
}

// htScenSlice returns the [lo, hi) slice of the loaded scenarios.
func htScenSlice(lo, hi int) []movingAIScenario { return htScenarios[lo:hi] }
