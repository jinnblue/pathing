package pathing

import (
	"math/bits"
)

// pathCoord is the frontier element shared by AStar and JPS.
type pathCoord struct {
	Coord GridCoord
	Cost  int32
}

// radixHeap is a generic monotone integer-priority queue.
//
// Correctness relies on the caller using a consistent heuristic so that
// f-values popped from the frontier are non-decreasing.  Push'd priorities
// must be >= the last-popped priority (or 0 before the first pop).
//
// Bucket i (i>=1) holds items whose priority XOR-differs from the current
// min in bit i; bucket 0 holds items with priority == min.  When bucket 0
// empties, we find the lowest non-empty bucket, update min to its smallest
// priority, and redistribute that bucket.  Each item is redistributed at
// most O(log maxPriority) times.  33 buckets suffice for any non-negative
// int32 priority.
const radixHeapBuckets = 33

type radixHeapElem[T any] struct {
	priority int32
	value    T
}

type radixHeap[T any] struct {
	buckets [radixHeapBuckets][]radixHeapElem[T]
	min     int32
	count   int
}

func newRadixHeap[T any]() *radixHeap[T] {
	h := &radixHeap[T]{}
	// Pre-size bucket 0 which sees the most churn (same-priority items).
	h.buckets[0] = make([]radixHeapElem[T], 0, 32)
	return h
}

func (h *radixHeap[T]) IsEmpty() bool { return h.count == 0 }

func (h *radixHeap[T]) Reset() {
	for i := range h.buckets {
		if len(h.buckets[i]) > 0 {
			h.buckets[i] = h.buckets[i][:0]
		}
	}
	h.min = 0
	h.count = 0
}

func (h *radixHeap[T]) bucketIndex(p int32) int {
	if p == h.min {
		return 0
	}
	return bits.Len32(uint32(p ^ h.min))
}

func (h *radixHeap[T]) Push(priority int32, v T) {
	b := h.bucketIndex(priority)
	h.buckets[b] = append(h.buckets[b], radixHeapElem[T]{priority: priority, value: v})
	h.count++
}

func (h *radixHeap[T]) Pop() T {
	// Fast path: bucket 0 holds items with priority == min.
	if n := len(h.buckets[0]); n > 0 {
		e := h.buckets[0][n-1]
		h.buckets[0] = h.buckets[0][:n-1]
		h.count--
		return e.value
	}
	// Find lowest non-empty bucket.
	i := 1
	for len(h.buckets[i]) == 0 {
		i++
	}
	// Locate new min priority in that bucket.
	bucket := h.buckets[i]
	newMin := bucket[0].priority
	for j := 1; j < len(bucket); j++ {
		if bucket[j].priority < newMin {
			newMin = bucket[j].priority
		}
	}
	h.min = newMin
	// Redistribute bucket into lower buckets.
	for _, e := range bucket {
		b := h.bucketIndex(e.priority)
		h.buckets[b] = append(h.buckets[b], e)
	}
	h.buckets[i] = bucket[:0]
	// One or more items with priority == newMin are now in bucket 0.
	n := len(h.buckets[0])
	e := h.buckets[0][n-1]
	h.buckets[0] = h.buckets[0][:n-1]
	h.count--
	return e.value
}

type minheap[T any] struct {
	elems []minheapElem[T]
}

type minheapElem[T any] struct {
	Value    T
	Priority int
}

func newMinheap[T any](size int) *minheap[T] {
	h := &minheap[T]{
		elems: make([]minheapElem[T], 0, size),
	}
	return h
}

func (h *minheap[T]) IsEmpty() bool {
	return len(h.elems) == 0
}

func (h *minheap[T]) Reset() {
	h.elems = h.elems[:0]
}

func (q *minheap[T]) Push(priority int, value T) {
	// Add new element to the end of the slice
	q.elems = append(q.elems, minheapElem[T]{
		Priority: priority,
		Value:    value,
	})

	// Percolate up operation, maintain a min-heap
	elems := q.elems
	i := uint(len(elems) - 1) // Index of new element
	for i > 0 {
		parent := (i - 1) / 2 // Index of parent node
		// If current node is root (i=0), or its priority is not less than parent, stop
		if elems[i].Priority >= elems[parent].Priority {
			break
		}
		// Swap current node and parent node
		elems[i], elems[parent] = elems[parent], elems[i]
		// Continue checking up
		i = parent
	}
}

func (q *minheap[T]) Pop() T {
	if len(q.elems) == 0 {
		var zero T
		return zero
	}

	// Swap the top element with the last element
	elems := q.elems
	size := len(elems) - 1
	elems[0], elems[size] = elems[size], elems[0]

	// Percolate down operation, maintain a min-heap
	i := 0
	for {
		j := 2*i + 1 // Index of left child node
		if j >= size {
			break // If no child node, exit
		}
		// Find the child node with higher priority (smaller value)
		if j2 := j + 1; j2 < size && elems[j2].Priority < elems[j].Priority {
			j = j2
		}
		// If current node's priority is not higher than child node, stop
		if elems[i].Priority < elems[j].Priority {
			break
		}
		// Swap current node and child node
		elems[i], elems[j] = elems[j], elems[i]
		// Continue checking down
		i = j
	}

	// Remove and return the original top element
	value := elems[size].Value
	q.elems = elems[:size]
	return value
}

// fixedPriorityQueue is a limit priority(64) queue based on a fixed hash table.
type fixedPriorityQueue[T any] struct {
	buckets [64][]T
	heads   [64]int // read index per bucket for FIFO ordering
	mask    uint64
}

func newFixedPriorityQueue[T any]() *fixedPriorityQueue[T] {
	h := &fixedPriorityQueue[T]{}
	for i := range &h.buckets {
		// Start with some small capacity for every bucket.
		h.buckets[i] = make([]T, 0, 4)
	}
	return h
}

func (q *fixedPriorityQueue[T]) Reset() {
	buckets := &q.buckets

	// Reslice storage slices back.
	// To avoid traversing all len(q.buckets),
	// we have some offset to skip uninteresting (already empty) buckets.
	// We also stop when mask is 0 meaning all remaining buckets are empty too.
	// In other words, it would only touch slices between min and max non-empty priorities.
	mask := q.mask
	offset := uint(bits.TrailingZeros64(mask))
	mask >>= offset
	i := offset
	for mask != 0 {
		if i < uint(len(buckets)) {
			buckets[i] = buckets[i][:0]
			q.heads[i] = 0
		}
		mask >>= 1
		i++
	}

	q.mask = 0
}

func (q *fixedPriorityQueue[T]) IsEmpty() bool {
	return q.mask == 0
}

func (q *fixedPriorityQueue[T]) Push(priority int, value T) {
	if priority < 0 {
		priority = 0
	} else if priority >= len(q.buckets) {
		priority = len(q.buckets) - 1
	}
	i := uint(priority)
	q.buckets[i] = append(q.buckets[i], value)
	q.mask |= 1 << i
}

func (q *fixedPriorityQueue[T]) Pop() (x T) {
	buckets := &q.buckets

	// Using uints here and explicit len check to avoid the implicitly inserted bound check.
	// extremely quickly calculate the number of trailing zeros in the binary representation of q.mask;
	// the result is exactly the position of the lowest bit set to 1 (index)
	// Example: ...0101000 represents non-empty buckets for priorities 3 and 5,
	// TrailingZeros64 returns 3, which is the index of the lowest priority.
	i := uint(bits.TrailingZeros64(q.mask))
	if i < uint(len(buckets)) {
		head := q.heads[i]
		e := buckets[i][head]
		head++
		if head >= len(buckets[i]) {
			// Bucket fully consumed; reslice and reset head.
			buckets[i] = buckets[i][:0]
			q.heads[i] = 0
			q.mask &^= 1 << i
		} else {
			q.heads[i] = head
		}
		return e
	}

	// A queue is empty?
	return x
}
