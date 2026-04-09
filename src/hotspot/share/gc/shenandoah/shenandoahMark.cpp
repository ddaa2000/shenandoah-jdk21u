/*
 * Copyright (c) 2021, 2022, Red Hat, Inc. All rights reserved.
 * Copyright Amazon.com Inc. or its affiliates. All Rights Reserved.
 * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
 *
 * This code is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 only, as
 * published by the Free Software Foundation.
 *
 * This code is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * version 2 for more details (a copy is included in the LICENSE file that
 * accompanied this code).
 *
 * You should have received a copy of the GNU General Public License version
 * 2 along with this work; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Please contact Oracle, 500 Oracle Parkway, Redwood Shores, CA 94065 USA
 * or visit www.oracle.com if you need additional information or have any
 * questions.
 *
 */


#include "precompiled.hpp"

#include "gc/shenandoah/shenandoahBarrierSet.hpp"
#include "gc/shenandoah/shenandoahClosures.inline.hpp"
#include "gc/shenandoah/shenandoahGeneration.hpp"
#include "gc/shenandoah/shenandoahHeapRegion.hpp"
#include "gc/shenandoah/shenandoahMark.inline.hpp"
#include "gc/shenandoah/shenandoahMarkingContext.inline.hpp"
#include "gc/shenandoah/shenandoahOopClosures.inline.hpp"
#include "gc/shenandoah/shenandoahReferenceProcessor.hpp"
#include "gc/shenandoah/shenandoahTaskqueue.inline.hpp"
#include "gc/shenandoah/shenandoahUtils.hpp"
#include "gc/shenandoah/shenandoahVerifier.hpp"
#include "logging/log.hpp"

void ShenandoahMark::start_mark() {
  if (!CodeCache::is_gc_marking_cycle_active()) {
    CodeCache::on_gc_marking_cycle_start();
  }
}

void ShenandoahMark::end_mark() {
  // Unlike other GCs, we do not arm the nmethods
  // when marking terminates.
  if (!ShenandoahHeap::heap()->is_concurrent_old_mark_in_progress()) {
    CodeCache::on_gc_marking_cycle_finish();
  }
}

ShenandoahMarkRefsSuperClosure::ShenandoahMarkRefsSuperClosure(ShenandoahObjToScanQueue* q,  ShenandoahReferenceProcessor* rp, ShenandoahObjToScanQueue* old_q) :
  MetadataVisitingOopIterateClosure(rp),
  _queue(q),
  _old_queue(old_q),
  _mark_context(ShenandoahHeap::heap()->marking_context()),
  _weak(false)
{ }

ShenandoahMark::ShenandoahMark(ShenandoahGeneration* generation) :
  _generation(generation),
  _task_queues(generation->task_queues()),
  _old_gen_task_queues(generation->old_gen_task_queues()),
  _finger_regions(nullptr),
  _finger_region_count(0),
  _finger_claim_index(0),
  _region_to_finger_index(nullptr) {
}

// Finger-aware closure constructor
ShenandoahMarkRefsFingerSuperClosure::ShenandoahMarkRefsFingerSuperClosure(
    ShenandoahObjToScanQueue* q, ShenandoahReferenceProcessor* rp,
    ShenandoahObjToScanQueue* old_q,
    ShenandoahMark* mark, ShenandoahFingerTask* finger_task) :
  ShenandoahMarkRefsSuperClosure(q, rp, old_q),
  _mark(mark),
  _finger_task(finger_task) {
}

// ShenandoahFingerTask methods
void ShenandoahFingerTask::setup_for_region(ShenandoahHeapRegion* r, ShenandoahMarkingContext* ctx) {
  _curr_region = r;
  _local_finger = r->bottom();
  _region_limit = ctx->top_at_mark_start(r);
}

void ShenandoahFingerTask::giveup_current_region() {
  _curr_region = nullptr;
  _local_finger = nullptr;
  _region_limit = nullptr;
}

// Build sorted array of young regions for finger-based marking
void ShenandoahMark::init_finger_regions() {
  ShenandoahHeap* heap = ShenandoahHeap::heap();
  size_t num_regions = heap->num_regions();
  ShenandoahMarkingContext* ctx = heap->marking_context();

  // Allocate arrays
  _finger_regions = NEW_C_HEAP_ARRAY(ShenandoahHeapRegion*, num_regions, mtGC);
  _region_to_finger_index = NEW_C_HEAP_ARRAY(size_t, num_regions, mtGC);

  // Initialize lookup table to SIZE_MAX (not in finger set)
  for (size_t i = 0; i < num_regions; i++) {
    _region_to_finger_index[i] = SIZE_MAX;
  }

  // Collect young regions that have content to scan (TAMS > bottom).
  // Regions are already in address order since we iterate by index.
  size_t count = 0;
  for (size_t i = 0; i < num_regions; i++) {
    ShenandoahHeapRegion* r = heap->get_region(i);
    if (r->is_young() && r->is_affiliated()) {
      HeapWord* tams = ctx->top_at_mark_start(r);
      if (tams > r->bottom()) {
        _finger_regions[count] = r;
        _region_to_finger_index[i] = count;
        count++;
      }
    }
  }

  _finger_region_count = count;
  _finger_claim_index = 0;

  log_info(gc, marking)("Finger marking: %zu young regions to scan", count);
}

void ShenandoahMark::destroy_finger_regions() {
  if (_finger_regions != nullptr) {
    FREE_C_HEAP_ARRAY(ShenandoahHeapRegion*, _finger_regions);
    _finger_regions = nullptr;
  }
  if (_region_to_finger_index != nullptr) {
    FREE_C_HEAP_ARRAY(size_t, _region_to_finger_index);
    _region_to_finger_index = nullptr;
  }
  _finger_region_count = 0;
  _finger_claim_index = 0;
}

ShenandoahHeapRegion* ShenandoahMark::claim_next_finger_region() {
  size_t idx = Atomic::fetch_then_add(&_finger_claim_index, (size_t)1);
  if (idx < _finger_region_count) {
    return _finger_regions[idx];
  }
  return nullptr;
}

template <ShenandoahGenerationType GENERATION, bool CANCELLABLE, StringDedupMode STRING_DEDUP>
void ShenandoahMark::mark_loop_prework(uint w, TaskTerminator *t, ShenandoahReferenceProcessor *rp, StringDedup::Requests* const req, bool update_refs) {
  ShenandoahObjToScanQueue* q = get_queue(w);
  ShenandoahObjToScanQueue* old_q = get_old_queue(w);

  ShenandoahHeap* const heap = ShenandoahHeap::heap();
  ShenandoahLiveData* ld = heap->get_liveness_cache(w);

  // Use finger-based marking for young generation when enabled and applicable.
  // Finger marking is only for concurrent marking (not update_refs / STW modes)
  // and only for the YOUNG generation where regions are scattered.
  if (use_finger_marking() && GENERATION == YOUNG && !update_refs) {
    using Closure = ShenandoahMarkRefsFingerClosure<GENERATION>;
    ShenandoahFingerTask finger_task;
    Closure cl(q, rp, old_q, this, &finger_task);
    mark_loop_work_finger<Closure, GENERATION, CANCELLABLE, STRING_DEDUP>(&cl, ld, w, t, req);
  } else if (update_refs) {
    using Closure = ShenandoahMarkUpdateRefsClosure<GENERATION>;
    Closure cl(q, rp, old_q);
    mark_loop_work<Closure, GENERATION, CANCELLABLE, STRING_DEDUP>(&cl, ld, w, t, req);
  } else {
    using Closure = ShenandoahMarkRefsClosure<GENERATION>;
    Closure cl(q, rp, old_q);
    mark_loop_work<Closure, GENERATION, CANCELLABLE, STRING_DEDUP>(&cl, ld, w, t, req);
  }

  heap->flush_liveness_cache(w);
}

template<bool CANCELLABLE, StringDedupMode STRING_DEDUP>
void ShenandoahMark::mark_loop(uint worker_id, TaskTerminator* terminator, ShenandoahReferenceProcessor *rp,
                               ShenandoahGenerationType generation, StringDedup::Requests* const req) {
  bool update_refs = ShenandoahHeap::heap()->has_forwarded_objects();
  switch (generation) {
    case YOUNG:
      mark_loop_prework<YOUNG, CANCELLABLE, STRING_DEDUP>(worker_id, terminator, rp, req, update_refs);
      break;
    case OLD:
      // Old generation collection only performs marking, it should not update references.
      mark_loop_prework<OLD, CANCELLABLE, STRING_DEDUP>(worker_id, terminator, rp, req, false);
      break;
    case GLOBAL:
      mark_loop_prework<GLOBAL, CANCELLABLE, STRING_DEDUP>(worker_id, terminator, rp, req, update_refs);
      break;
    case NON_GEN:
      mark_loop_prework<NON_GEN, CANCELLABLE, STRING_DEDUP>(worker_id, terminator, rp, req, update_refs);
      break;
    default:
      ShouldNotReachHere();
      break;
  }
}

void ShenandoahMark::mark_loop(uint worker_id, TaskTerminator* terminator, ShenandoahReferenceProcessor *rp,
                               ShenandoahGenerationType generation, bool cancellable, StringDedupMode dedup_mode, StringDedup::Requests* const req) {
  if (cancellable) {
    switch(dedup_mode) {
      case NO_DEDUP:
        mark_loop<true, NO_DEDUP>(worker_id, terminator, rp, generation, req);
        break;
      case ENQUEUE_DEDUP:
        mark_loop<true, ENQUEUE_DEDUP>(worker_id, terminator, rp, generation, req);
        break;
      case ALWAYS_DEDUP:
        mark_loop<true, ALWAYS_DEDUP>(worker_id, terminator, rp, generation, req);
        break;
    }
  } else {
    switch(dedup_mode) {
      case NO_DEDUP:
        mark_loop<false, NO_DEDUP>(worker_id, terminator, rp, generation, req);
        break;
      case ENQUEUE_DEDUP:
        mark_loop<false, ENQUEUE_DEDUP>(worker_id, terminator, rp, generation, req);
        break;
      case ALWAYS_DEDUP:
        mark_loop<false, ALWAYS_DEDUP>(worker_id, terminator, rp, generation, req);
        break;
    }
  }
}

template <class T, ShenandoahGenerationType GENERATION, bool CANCELLABLE, StringDedupMode STRING_DEDUP>
void ShenandoahMark::mark_loop_work(T* cl, ShenandoahLiveData* live_data, uint worker_id, TaskTerminator *terminator, StringDedup::Requests* const req) {
  uintx stride = ShenandoahMarkLoopStride;

  ShenandoahHeap* heap = ShenandoahHeap::heap();
  ShenandoahObjToScanQueueSet* queues = task_queues();
  ShenandoahObjToScanQueue* q;
  ShenandoahMarkTask t;

  // Do not use active_generation() : we must use the gc_generation() set by
  // ShenandoahGCScope on the ControllerThread's stack; no safepoint may
  // intervene to update active_generation, so we can't
  // shenandoah_assert_generations_reconciled() here.
  assert(heap->gc_generation()->type() == GENERATION, "Sanity: %d != %d", heap->gc_generation()->type(), GENERATION);
  heap->gc_generation()->ref_processor()->set_mark_closure(worker_id, cl);

  /*
   * Process outstanding queues, if any.
   *
   * There can be more queues than workers. To deal with the imbalance, we claim
   * extra queues first. Since marking can push new tasks into the queue associated
   * with this worker id, we come back to process this queue in the normal loop.
   */
  assert(queues->get_reserved() == heap->workers()->active_workers(),
         "Need to reserve proper number of queues: reserved: %u, active: %u", queues->get_reserved(), heap->workers()->active_workers());

  q = queues->claim_next();
  while (q != nullptr) {
    if (CANCELLABLE && heap->check_cancelled_gc_and_yield()) {
      return;
    }

    for (uint i = 0; i < stride; i++) {
      if (q->pop(t)) {
        do_task<T, GENERATION, STRING_DEDUP>(q, cl, live_data, req, &t, worker_id);
      } else {
        assert(q->is_empty(), "Must be empty");
        q = queues->claim_next();
        break;
      }
    }
  }
  q = get_queue(worker_id);
  ShenandoahObjToScanQueue* old_q = get_old_queue(worker_id);

  ShenandoahSATBBufferClosure<GENERATION> drain_satb(q, old_q);
  SATBMarkQueueSet& satb_mq_set = ShenandoahBarrierSet::satb_mark_queue_set();

  /*
   * Normal marking loop:
   */
  while (true) {
    if (CANCELLABLE && heap->check_cancelled_gc_and_yield()) {
      return;
    }
    while (satb_mq_set.completed_buffers_num() > 0) {
      satb_mq_set.apply_closure_to_completed_buffer(&drain_satb);
    }

    uint work = 0;
    for (uint i = 0; i < stride; i++) {
      if (q->pop(t) ||
          queues->steal(worker_id, t)) {
        do_task<T, GENERATION, STRING_DEDUP>(q, cl, live_data, req, &t, worker_id);
        work++;
      } else {
        break;
      }
    }

    if (work == 0) {
      // No work encountered in current stride, try to terminate.
      // Need to leave the STS here otherwise it might block safepoints.
      ShenandoahSuspendibleThreadSetLeaver stsl(CANCELLABLE);
      ShenandoahTerminatorTerminator tt(heap);
      if (terminator->offer_termination(&tt)) return;
    }
  }
}

// Finger-based marking loop: scans the mark bitmap linearly within each claimed
// region, similar to G1's concurrent marking. Objects above the global finger
// are implicitly grey (deferred to bitmap scan) and not pushed to the queue.
//
// Structure follows G1's do_marking_step():
// Phase 1: Drain pre-existing queues (same as regular mark_loop_work)
// Phase 2: Finger-based bitmap scanning — claim regions in address order,
//          scan bitmap linearly within each region, interleave with SATB/queue drain
// Phase 3: Drain remaining queue + SATB + work stealing (termination)
template <class T, ShenandoahGenerationType GENERATION, bool CANCELLABLE, StringDedupMode STRING_DEDUP>
void ShenandoahMark::mark_loop_work_finger(T* cl, ShenandoahLiveData* live_data, uint worker_id, TaskTerminator *terminator, StringDedup::Requests* const req) {
  uintx stride = ShenandoahMarkLoopStride;

  ShenandoahHeap* heap = ShenandoahHeap::heap();
  ShenandoahMarkingContext* mark_context = heap->marking_context();
  ShenandoahObjToScanQueueSet* queues = task_queues();
  ShenandoahObjToScanQueue* q;
  ShenandoahMarkTask t;

  assert(heap->gc_generation()->type() == GENERATION, "Sanity: %d != %d", heap->gc_generation()->type(), GENERATION);
  heap->gc_generation()->ref_processor()->set_mark_closure(worker_id, cl);

  // The closure carries a pointer to the per-worker ShenandoahFingerTask
  // (stack-allocated in mark_loop_prework). We need it for bitmap scanning
  // and the is_below_finger check inside the closure's mark_through_ref_with_finger.
  ShenandoahMarkRefsFingerSuperClosure* finger_cl = static_cast<ShenandoahMarkRefsFingerSuperClosure*>(cl);
  ShenandoahFingerTask* finger_task = finger_cl->finger_task();

  assert(queues->get_reserved() == heap->workers()->active_workers(),
         "Need to reserve proper number of queues: reserved: %u, active: %u", queues->get_reserved(), heap->workers()->active_workers());

  // ---- Phase 1: Process outstanding queues (same as regular) ----
  q = queues->claim_next();
  while (q != nullptr) {
    if (CANCELLABLE && heap->check_cancelled_gc_and_yield()) {
      return;
    }

    for (uint i = 0; i < stride; i++) {
      if (q->pop(t)) {
        do_task<T, GENERATION, STRING_DEDUP>(q, cl, live_data, req, &t, worker_id);
      } else {
        assert(q->is_empty(), "Must be empty");
        q = queues->claim_next();
        break;
      }
    }
  }

  q = get_queue(worker_id);
  ShenandoahObjToScanQueue* old_q = get_old_queue(worker_id);

  ShenandoahSATBBufferClosure<GENERATION> drain_satb(q, old_q);
  SATBMarkQueueSet& satb_mq_set = ShenandoahBarrierSet::satb_mark_queue_set();

  // ---- Phase 2: Finger-based bitmap scanning ----
  // Claim regions in address order and scan the mark bitmap linearly.
  // After scanning each object, partially drain the local queue and SATB buffers.
  // This is analogous to G1's bitmap_closure.do_addr() + drain pattern.

  do {
    if (CANCELLABLE && heap->check_cancelled_gc_and_yield()) {
      return;
    }

    if (finger_task->curr_region() != nullptr) {
      HeapWord* finger = finger_task->local_finger();
      HeapWord* limit = finger_task->region_limit();

      // Scan bitmap linearly within this region: finger -> limit (TAMS)
      while (finger < limit) {
        if (CANCELLABLE && heap->check_cancelled_gc_and_yield()) {
          return;
        }

        HeapWord* addr = mark_context->get_next_marked_addr(finger, limit);
        if (addr >= limit) {
          break;  // No more marked objects in this region
        }

        // Advance local finger to this object (analogous to G1CMBitMapClosure::do_addr)
        finger_task->move_finger_to(addr);

        // Process this object: scan its fields via the finger-aware closure
        oop obj = cast_to_oop(addr);
        shenandoah_assert_not_forwarded(nullptr, obj);
        shenandoah_assert_marked(nullptr, obj);

        // Count liveness
        count_liveness<GENERATION>(live_data, obj, worker_id);

        // Scan the object (this will call the finger closure for discovered refs)
        if (obj->is_instance()) {
          if (ContinuationGCSupport::relativize_stack_chunk(obj)) {
            cl->set_weak(false);
          }
          obj->oop_iterate(cl);
          dedup_string<STRING_DEDUP>(obj, req);
        } else if (obj->is_objArray()) {
          // For arrays, use the chunked processing (pushes chunks to queue)
          do_chunked_array_start<T>(q, cl, obj, false);
        }
        // typeArrays have no oops, skip

        // Advance finger past this object
        finger = addr + obj->size();
        finger_task->move_finger_to(finger);

        // Partially drain local queue and SATB buffers (like G1)
        // This ensures we don't let the queue grow unboundedly during bitmap scan.
        while (satb_mq_set.completed_buffers_num() > 0) {
          satb_mq_set.apply_closure_to_completed_buffer(&drain_satb);
        }
        uint drained = 0;
        for (uint i = 0; i < stride && q->pop(t); i++) {
          do_task<T, GENERATION, STRING_DEDUP>(q, cl, live_data, req, &t, worker_id);
          drained++;
        }
      }

      // Done with this region
      finger_task->giveup_current_region();
    }

    // Try to claim the next region
    ShenandoahHeapRegion* claimed = claim_next_finger_region();
    if (claimed != nullptr) {
      finger_task->setup_for_region(claimed, mark_context);
    }
  } while (finger_task->curr_region() != nullptr);

  // ---- Phase 3: Drain remaining queue + SATB + work stealing ----
  // All regions have been scanned. Now drain everything that was pushed
  // during bitmap scanning and SATB processing.

  // Full SATB drain
  while (satb_mq_set.completed_buffers_num() > 0) {
    satb_mq_set.apply_closure_to_completed_buffer(&drain_satb);
  }

  // Full local queue drain
  while (q->pop(t)) {
    do_task<T, GENERATION, STRING_DEDUP>(q, cl, live_data, req, &t, worker_id);
  }

  // Work stealing loop (same structure as regular mark_loop_work termination)
  while (true) {
    if (CANCELLABLE && heap->check_cancelled_gc_and_yield()) {
      return;
    }
    while (satb_mq_set.completed_buffers_num() > 0) {
      satb_mq_set.apply_closure_to_completed_buffer(&drain_satb);
    }

    uint work = 0;
    for (uint i = 0; i < stride; i++) {
      if (q->pop(t) ||
          queues->steal(worker_id, t)) {
        do_task<T, GENERATION, STRING_DEDUP>(q, cl, live_data, req, &t, worker_id);
        work++;
      } else {
        break;
      }
    }

    if (work == 0) {
      ShenandoahSuspendibleThreadSetLeaver stsl(CANCELLABLE);
      ShenandoahTerminatorTerminator tt(heap);
      if (terminator->offer_termination(&tt)) return;
    }
  }
}
