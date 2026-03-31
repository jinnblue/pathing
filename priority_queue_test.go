package pathing

import (
	"container/heap"
	"image"
	"math/rand/v2"
	"sort"
	"strings"
	"testing"
)

type priorityQueuer[T any] interface {
	Push(priority int, value T)
	Pop() (x T)
	IsEmpty() bool
	Reset()
}

type pqKind int

const (
	pqMinHeap pqKind = iota
	pqFixedPriorityQueue
)

func newPQueue[T any](k pqKind) priorityQueuer[T] {
	switch k {
	case pqMinHeap:
		return newMinheap[T](0)
	case pqFixedPriorityQueue:
		return newFixedPriorityQueue[T]()
	default:
		panic("unknown priority queue kind")
	}
}

func ensureEmpty[T any](t *testing.T, iq priorityQueuer[T]) {
	t.Helper()

	if !iq.IsEmpty() {
		t.Fatal("queue is not empty")
	}

	if q, ok := iq.(*fixedPriorityQueue[T]); ok {
		if q.mask != 0 {
			t.Fatal("queue is not empty")
		}
		for i, b := range &q.buckets {
			if len(b) != 0 {
				t.Fatalf("buckets[%d] is not empty", i)
			}
		}
	}
}

func TestPriorityQueue(t *testing.T) {
	for k := pqMinHeap; k <= pqFixedPriorityQueue; k++ {
		q := newPQueue[int](k)
		testIntPriorityQueue(t, q)
	}
}

func testIntPriorityQueue(t *testing.T, pqueue priorityQueuer[int]) {
	tests := []struct {
		input  string
		output string
	}{
		{"a", "a"},
		{"aaa", "aaa"},
		{"aa", "aa"},
		{"ba", "ab"},
		{"baa", "aab"},
		{"ab", "ab"},
		{"aaba", "aaab"},
		{"abcd", "abcd"},
		{"dcab", "abcd"},
		{"aaaaaaa", "aaaaaaa"},
		{"aaaaaab", "aaaaaab"},
		{"baaaaaa", "aaaaaab"},
		{"baaaaab", "aaaaabb"},
		{"aaaaaaaaaaaaaaa", "aaaaaaaaaaaaaaa"},
		{"ababababab", "aaaaabbbbb"},
		{"abcabcabcabc", "aaaabbbbcccc"},
		{"abcdabcdabcdabcd", "aaaabbbbccccdddd"},
		{"aabbccddaabbccdd", "aaaabbbbccccdddd"},
	}

	for _, test := range tests {
		pqueue.Reset()
		for i := 0; i < len(test.input); i++ {
			b := test.input[i]
			priority := int(b) - 'a'
			pqueue.Push(priority, int(b))
		}
		var output strings.Builder
		for !pqueue.IsEmpty() {
			output.WriteByte(byte(pqueue.Pop()))
		}
		ensureEmpty(t, pqueue)
		if output.String() != test.output {
			t.Fatalf("input=%q:\nhave: %q\nwant: %q", test.input, output.String(), test.output)
		}
	}

	{ // ensureEmpty
		q := newFixedPriorityQueue[int]()
		ensureEmpty(t, q)
		for i := 0; i < 50; i++ {
			q.Push(i, i)
		}
		for i := 0; i < 50; i++ {
			result := q.Pop()
			if i != result {
				t.Fatal("invalid result in push+pop pair")
			}
		}
		ensureEmpty(t, q)
	}

	for i := 0; i < 64; i++ {
		r := rand.New(rand.NewPCG(1753682265044, 0))
		var values []int
		num := r.IntN(96) + 6
		for i := 0; i < num; i++ {
			maxValue := 63 // fixedPriorityQueue supports priorities 0..63
			minValue := 0
			v := rand.IntN(maxValue-minValue) + minValue
			values = append(values, v)
		}
		sortedValues := make([]int, len(values))
		copy(sortedValues, values)
		sort.SliceStable(sortedValues, func(i, j int) bool {
			return sortedValues[i] < sortedValues[j]
		})
		var q fixedPriorityQueue[int]
		for _, x := range values {
			q.Push(x, x)
		}
		for _, x := range sortedValues {
			y := q.Pop()
			if x != y {
				t.Fatal("invalid result in push+pop pair")
			}
		}
		ensureEmpty(t, &q)
	}
}

const elements_num = 1 << 8

func BenchmarkMinheapPushPop(b *testing.B) {
	r := rand.New(rand.NewPCG(1753682265044, 0))
	pq := newMinheap[image.Point](32)

	var it image.Point
	for i := 0; i < b.N; i++ {
		for range elements_num {
			pq.Push(r.IntN(elements_num), image.Point{X: r.IntN(1024), Y: r.IntN(1024)})
		}
		for !pq.IsEmpty() {
			it = pq.Pop()
		}
	}
	b.Log(it)
}

func BenchmarkFixedPriorityQueuePushPop(b *testing.B) {
	r := rand.New(rand.NewPCG(1753682265044, 0))
	pq := newFixedPriorityQueue[image.Point]()

	var it image.Point
	for i := 0; i < b.N; i++ {
		for range elements_num {
			pq.Push(r.IntN(elements_num), image.Point{X: r.IntN(1024), Y: r.IntN(1024)})
		}
		for !pq.IsEmpty() {
			it = pq.Pop()
		}
	}
	b.Log(it)
}

func BenchmarkHeapPushPop(b *testing.B) {
	r := rand.New(rand.NewPCG(1753682265044, 0))
	pq := &heapPriorityQueue[image.Point]{}
	heap.Init(pq)

	var it item[image.Point]
	for i := 0; i < b.N; i++ {
		for range elements_num {
			heap.Push(pq, item[image.Point]{
				score: -float64(r.IntN(elements_num)),
				value: image.Point{X: r.IntN(1024), Y: r.IntN(1024)},
			})
		}
		for pq.Len() > 0 {
			it = heap.Pop(pq).(item[image.Point])
		}
	}
	b.Log(it)
}

// An item is something we manage in a priority queue.
type item[T any] struct {
	value T       // The value of the item; arbitrary.
	score float64 // The priority of the item in the queue.
}

// A heapPriorityQueue implements heap.Interface and holds items.
type heapPriorityQueue[T any] []item[T]

func (pq heapPriorityQueue[T]) Len() int { return len(pq) }

func (pq heapPriorityQueue[T]) Less(i, j int) bool {
	return pq[i].score < pq[j].score
}

func (pq heapPriorityQueue[T]) Swap(i, j int) {
	pq[i], pq[j] = pq[j], pq[i]
}

func (pq *heapPriorityQueue[T]) Push(x any) {
	*pq = append(*pq, x.(item[T]))
}

func (pq *heapPriorityQueue[T]) Pop() any {
	old := *pq
	n := len(old)
	it := old[n-1]
	*pq = old[0 : n-1]
	return it
}

func TestFixedPriorityQueueClampsPriorityAboveLastBucket(t *testing.T) {
	q := newFixedPriorityQueue[int]()
	q.Push(3, 3)
	q.Push(128, 128)
	q.Push(64, 64)

	if got := q.Pop(); got != 3 {
		t.Fatalf("first pop = %d, want 3", got)
	}
	if got := q.Pop(); got != 64 {
		t.Fatalf("second pop = %d, want 64", got)
	}
	if got := q.Pop(); got != 128 {
		t.Fatalf("third pop = %d, want 128", got)
	}
	if !q.IsEmpty() {
		t.Fatal("queue should be empty")
	}
}

func TestFixedPriorityQueueClampsNegativePriorityToZero(t *testing.T) {
	q := newFixedPriorityQueue[int]()
	q.Push(-10, -10)
	q.Push(2, 2)

	if got := q.Pop(); got != -10 {
		t.Fatalf("first pop = %d, want -10", got)
	}
	if got := q.Pop(); got != 2 {
		t.Fatalf("second pop = %d, want 2", got)
	}
}
