# Finger-Based Sequential Marking for GenShen Young Generation

## Background: G1's Two-Level Finger Mechanism

G1's concurrent marking uses a two-level finger to drive bitmap-based linear scanning:

### Global Finger (`G1ConcurrentMark::_finger`)
- Region-aligned pointer, initialized to `heap.start()`
- Workers atomically CAS-advance it to claim regions
- Defines the **grey/black boundary** for the entire heap:
  - Objects below global finger: either black (scanned) or explicitly grey (on mark stack)
  - Objects above global finger: **implicitly grey** — will be found by bitmap scan later

### Task-Local Finger (`G1CMTask::_finger`)
- Per-worker scanning cursor within a claimed region
- Advances from `region->bottom()` to `region->top_at_mark_start()` (TAMS)
- Uses `get_next_marked_addr()` to find next marked object in bitmap

### Decision Logic (`is_below_finger`)
When a new reference is discovered during object scanning:

```
if (obj < local_finger)         → PUSH (already scanned past it in current region)
if (local_finger <= obj < TAMS) → SKIP (bitmap scan will find it in current region)
if (TAMS <= obj < global_finger)→ PUSH (in already-claimed/scanned region)
if (obj >= global_finger)       → SKIP (in unclaimed region, bitmap scan will find it)
```

Key benefit: objects above the finger are **never pushed to the mark stack**, dramatically
reducing queue pressure. Only objects in already-scanned territory need explicit queuing.

### G1 Marking Loop Structure
```
do {
    if (has_current_region) {
        // Linear bitmap scan: finger -> TAMS
        bitmap.iterate(&closure, MemRegion(finger, region_limit));
        // Each marked object found:
        //   1. Advance local finger
        //   2. Scan object (push discovered refs via make_reference_grey)
        //   3. Partially drain local queue and global stack
    }
    // Claim next region
    while (!out_of_regions) {
        claimed = cm->claim_region();  // Advances global finger atomically
        if (claimed) setup_for_region(claimed);
    }
} while (has_current_region);

// Drain remaining queue + work stealing
drain_local_queue();
drain_global_stack();
while (steal(entry)) { scan(entry); drain_all(); }
```

## Design: Adapting Finger-Based Marking for GenShen

### Key Difference from G1
G1's concurrent marking scans **all regions** (whole heap). GenShen's young concurrent
marking scans **only young-generation regions**, which are scattered across the heap.

### Solution: Sorted Young Region Array + Global Finger Index

Instead of sweeping a global finger across the entire heap address space, we maintain
a **sorted array of young region pointers** and use an **index into this array** as
the global finger.

```
Young regions sorted by address:
  [0] region@0x1000  [1] region@0x3000  [2] region@0x7000  [3] region@0xA000
                                          ^
                                    global_finger_index = 2
                                    (regions 0,1 already claimed)
```

The `is_below_finger` check becomes:
- Compare the target object's region index against the global finger index
- If target is in a region before the global finger index → PUSH
- If target is in a region after the global finger index → SKIP (implicit grey)
- If target is in the current region → compare with local finger

### Data Structures

**In `ShenandoahMark` (or a new helper class):**
```cpp
// Sorted array of young regions participating in this marking cycle
ShenandoahHeapRegion** _finger_regions;      // Array sorted by address
size_t                 _finger_region_count;  // Number of young regions
volatile size_t        _finger_claim_index;   // Global finger (atomically advanced)
```

**Per-worker state (passed through mark_loop_work):**
```cpp
ShenandoahHeapRegion*  _curr_region;    // Currently claimed region
HeapWord*              _local_finger;   // Local scanning cursor within region
HeapWord*              _region_limit;   // TAMS of current region
```

### Modified mark_loop_work

The new marking loop follows G1's structure:

```
Phase 1: Process pre-existing queues (unchanged from current code)

Phase 2: Finger-based bitmap scanning
  do {
      if (curr_region != nullptr) {
          // Scan bitmap linearly: local_finger -> region_limit (TAMS)
          while (local_finger < region_limit) {
              addr = get_next_marked_addr(local_finger, region_limit);
              if (addr >= region_limit) break;
              local_finger = addr;
              scan_object(addr);  // Uses modified mark_ref with finger check
              local_finger = addr + obj->size();
              // Interleave: drain SATB buffers + partial queue drain
          }
          curr_region = nullptr;  // Done with this region
      }
      // Claim next region
      claimed = claim_next_finger_region();
      if (claimed) setup_finger_region(claimed);
  } while (curr_region != nullptr);

Phase 3: Drain remaining queue + SATB + work stealing (similar to current)
```

### Modified mark_ref (is_below_finger logic)

```cpp
void mark_ref_with_finger(queue, mark_context, weak, obj,
                          curr_region, local_finger, global_finger_index) {
    marked = mark_context->mark_{strong|weak}(obj);
    if (!marked) return;  // Already marked

    HeapWord* obj_addr = cast_from_oop<HeapWord*>(obj);

    // Check if object is in a young region that needs explicit queuing
    if (is_below_finger(obj_addr, curr_region, local_finger, global_finger_index)) {
        queue->push(ShenandoahMarkTask(obj, ...));
    }
    // Otherwise: object is above the finger, bitmap scan will find it later
}
```

### is_below_finger for GenShen

```cpp
bool is_below_finger(HeapWord* obj_addr,
                     ShenandoahHeapRegion* curr_region,
                     HeapWord* local_finger,
                     size_t global_finger_index) {
    ShenandoahHeapRegion* obj_region = heap->heap_region_containing(obj_addr);

    // If object is in current region, use local finger
    if (obj_region == curr_region) {
        return obj_addr < local_finger;
        // obj at or above local_finger: bitmap scan will find it
    }

    // If object is in a young region, check against global finger
    if (obj_region->is_young()) {
        size_t obj_region_finger_idx = find_finger_index(obj_region);
        return obj_region_finger_idx < global_finger_index;
        // Below global finger: region already claimed/scanned, must push
        // At or above global finger: will be claimed later, skip push
    }

    // Object is in old region — always push (cross-gen reference)
    return true;
}
```

### Region Claiming

```cpp
ShenandoahHeapRegion* claim_next_finger_region() {
    size_t idx = Atomic::fetch_then_add(&_finger_claim_index, 1);
    if (idx < _finger_region_count) {
        return _finger_regions[idx];
    }
    return nullptr;  // All regions claimed
}
```

### Initialization (at mark start)

Before marking begins, build the sorted region array:
```cpp
void init_finger_regions() {
    // Collect all young regions
    for (size_t i = 0; i < heap->num_regions(); i++) {
        ShenandoahHeapRegion* r = heap->get_region(i);
        if (r->is_young() && r->is_affiliated()) {
            _finger_regions[count++] = r;
        }
    }
    // Already sorted by index (= address order)
    _finger_region_count = count;
    _finger_claim_index = 0;
}
```

### Handling Cross-Generation and Non-Young References

When `mark_ref` discovers a reference:
1. **Target in young region above finger** → just mark bitmap, don't push
2. **Target in young region below finger** → mark bitmap + push to queue
3. **Target in old region** → goes to `old_queue` (unchanged behavior)
4. **Target allocated after TAMS** → implicitly live (unchanged)

### SATB Buffer Handling

SATB buffers contain objects snapshotted by the write barrier. These must always be
processed regardless of finger position. In the finger-based loop, SATB draining happens:
1. Before each region scan iteration
2. After finishing all regions (final drain)
3. Objects from SATB are marked and pushed to queue if below finger (same logic)
