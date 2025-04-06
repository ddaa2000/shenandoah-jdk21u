/*
 * Copyright (c) 2018, 2019, Red Hat, Inc. All rights reserved.
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


#include "gc/shared/gcCause.hpp"
#include "gc/shenandoah/heuristics/shenandoahHeuristics.hpp"
#include "gc/shenandoah/heuristics/shenandoahSpaceInfo.hpp"
#include "gc/shenandoah/heuristics/shenandoahAdaptiveHeuristics.hpp"
#include "gc/shenandoah/shenandoahCollectionSet.hpp"
#include "gc/shenandoah/shenandoahCollectorPolicy.hpp"
#include "gc/shenandoah/shenandoahFreeSet.hpp"
#include "gc/shenandoah/shenandoahHeap.inline.hpp"
#include "gc/shenandoah/shenandoahHeapRegion.inline.hpp"
#include "logging/log.hpp"
#include "logging/logTag.hpp"
#include "runtime/globals_extension.hpp"
#include "utilities/quickSort.hpp"

// These constants are used to adjust the margin of error for the moving
// average of the allocation rate and cycle time. The units are standard
// deviations.
const double ShenandoahAdaptiveHeuristics::FULL_PENALTY_SD = 0.2;
const double ShenandoahAdaptiveHeuristics::DEGENERATE_PENALTY_SD = 0.1;

// These are used to decide if we want to make any adjustments at all
// at the end of a successful concurrent cycle.
const double ShenandoahAdaptiveHeuristics::LOWEST_EXPECTED_AVAILABLE_AT_END = -0.5;
const double ShenandoahAdaptiveHeuristics::HIGHEST_EXPECTED_AVAILABLE_AT_END = 0.5;

// These values are the confidence interval expressed as standard deviations.
// At the minimum confidence level, there is a 25% chance that the true value of
// the estimate (average cycle time or allocation rate) is not more than
// MINIMUM_CONFIDENCE standard deviations away from our estimate. Similarly, the
// MAXIMUM_CONFIDENCE interval here means there is a one in a thousand chance
// that the true value of our estimate is outside the interval. These are used
// as bounds on the adjustments applied at the outcome of a GC cycle.
const double ShenandoahAdaptiveHeuristics::MINIMUM_CONFIDENCE = 0.319; // 25%
const double ShenandoahAdaptiveHeuristics::MAXIMUM_CONFIDENCE = 3.291; // 99.9%

ShenandoahAdaptiveHeuristics::ShenandoahAdaptiveHeuristics(ShenandoahSpaceInfo* space_info) :
  ShenandoahHeuristics(space_info),
  _copy_bytes_during_gc(0),
  _margin_of_error_sd(ShenandoahAdaptiveInitialConfidence),
  _spike_threshold_sd(ShenandoahAdaptiveInitialSpikeThreshold),
  _last_trigger(OTHER),
  _available(Moving_Average_Samples, ShenandoahAdaptiveDecayFactor) { }

ShenandoahAdaptiveHeuristics::~ShenandoahAdaptiveHeuristics() {}

void ShenandoahAdaptiveHeuristics::choose_collection_set_from_regiondata(ShenandoahCollectionSet* cset,
                                                                         RegionData* data, size_t size,
                                                                         size_t actual_free) {
  size_t garbage_threshold = ShenandoahHeapRegion::region_size_bytes() * ShenandoahGarbageThreshold / 100;

  // The logic for cset selection in adaptive is as follows:
  //
  //   1. We cannot get cset larger than available free space. Otherwise we guarantee OOME
  //      during evacuation, and thus guarantee full GC. In practice, we also want to let
  //      application to allocate something. This is why we limit CSet to some fraction of
  //      available space. In non-overloaded heap, max_cset would contain all plausible candidates
  //      over garbage threshold.
  //
  //   2. We should not get cset too low so that free threshold would not be met right
  //      after the cycle. Otherwise we get back-to-back cycles for no reason if heap is
  //      too fragmented. In non-overloaded non-fragmented heap min_garbage would be around zero.
  //
  // Therefore, we start by sorting the regions by garbage. Then we unconditionally add the best candidates
  // before we meet min_garbage. Then we add all candidates that fit with a garbage threshold before
  // we hit max_cset. When max_cset is hit, we terminate the cset selection. Note that in this scheme,
  // ShenandoahGarbageThreshold is the soft threshold which would be ignored until min_garbage is hit.

  size_t capacity    = _space_info->soft_max_capacity();
  size_t max_cset    = (size_t)((1.0 * capacity / 100 * ShenandoahEvacReserve) / ShenandoahEvacWaste);
  size_t free_target = (capacity * ShenandoahMinFreeThreshold) / 100 + max_cset;
  size_t min_garbage = (free_target > actual_free) ? (free_target - actual_free) : 0;

  log_info(gc, ergo)("Adaptive CSet Selection. Target Free: " SIZE_FORMAT "%s, Actual Free: "
                     SIZE_FORMAT "%s, Max Evacuation: " SIZE_FORMAT "%s, Min Garbage: " SIZE_FORMAT "%s",
                     byte_size_in_proper_unit(free_target), proper_unit_for_byte_size(free_target),
                     byte_size_in_proper_unit(actual_free), proper_unit_for_byte_size(actual_free),
                     byte_size_in_proper_unit(max_cset),    proper_unit_for_byte_size(max_cset),
                     byte_size_in_proper_unit(min_garbage), proper_unit_for_byte_size(min_garbage));

  // Better select garbage-first regions
  QuickSort::sort<RegionData>(data, (int)size, compare_by_garbage, false);

  size_t cur_cset = 0;
  size_t cur_garbage = 0;

  for (size_t idx = 0; idx < size; idx++) {
    ShenandoahHeapRegion* r = data[idx]._region;

    size_t new_cset    = cur_cset + r->get_live_data_bytes();
    size_t new_garbage = cur_garbage + r->garbage();

    if (new_cset > max_cset) {
      break;
    }

    if ((new_garbage < min_garbage) || (r->garbage() > garbage_threshold)) {
      cset->add_region(r);
      cur_cset = new_cset;
      cur_garbage = new_garbage;
    }
  }
}

void ShenandoahAdaptiveHeuristics::record_cycle_start() {
  ShenandoahHeuristics::record_cycle_start();
  _allocation_rate.allocation_counter_reset();
  // _allocation_rate_user.allocation_counter_reset();
}

void ShenandoahAdaptiveHeuristics::record_success_concurrent(bool abbreviated) {
  ShenandoahHeuristics::record_success_concurrent(abbreviated);

  size_t available = _space_info->available();

  double z_score = 0.0;
  double available_sd = _available.sd();
  if (available_sd > 0) {
    double available_avg = _available.avg();
    z_score = (double(available) - available_avg) / available_sd;
    log_debug(gc, ergo)("%s Available: " SIZE_FORMAT " %sB, z-score=%.3f. Average available: %.1f %sB +/- %.1f %sB.",
                        _space_info->name(),
                        byte_size_in_proper_unit(available), proper_unit_for_byte_size(available),
                        z_score,
                        byte_size_in_proper_unit(available_avg), proper_unit_for_byte_size(available_avg),
                        byte_size_in_proper_unit(available_sd), proper_unit_for_byte_size(available_sd));
  }

  _available.add(double(available));

  // In the case when a concurrent GC cycle completes successfully but with an
  // unusually small amount of available memory we will adjust our trigger
  // parameters so that they are more likely to initiate a new cycle.
  // Conversely, when a GC cycle results in an above average amount of available
  // memory, we will adjust the trigger parameters to be less likely to initiate
  // a GC cycle.
  //
  // The z-score we've computed is in no way statistically related to the
  // trigger parameters, but it has the nice property that worse z-scores for
  // available memory indicate making larger adjustments to the trigger
  // parameters. It also results in fewer adjustments as the application
  // stabilizes.
  //
  // In order to avoid making endless and likely unnecessary adjustments to the
  // trigger parameters, the change in available memory (with respect to the
  // average) at the end of a cycle must be beyond these threshold values.
  if (z_score < LOWEST_EXPECTED_AVAILABLE_AT_END ||
      z_score > HIGHEST_EXPECTED_AVAILABLE_AT_END) {
    // The sign is flipped because a negative z-score indicates that the
    // available memory at the end of the cycle is below average. Positive
    // adjustments make the triggers more sensitive (i.e., more likely to fire).
    // The z-score also gives us a measure of just how far below normal. This
    // property allows us to adjust the trigger parameters proportionally.
    //
    // The `100` here is used to attenuate the size of our adjustments. This
    // number was chosen empirically. It also means the adjustments at the end of
    // a concurrent cycle are an order of magnitude smaller than the adjustments
    // made for a degenerated or full GC cycle (which themselves were also
    // chosen empirically).
    adjust_last_trigger_parameters(z_score / -100);
  }
}

void ShenandoahAdaptiveHeuristics::record_success_degenerated() {
  ShenandoahHeuristics::record_success_degenerated();
  // Adjust both trigger's parameters in the case of a degenerated GC because
  // either of them should have triggered earlier to avoid this case.
  adjust_margin_of_error(DEGENERATE_PENALTY_SD);
  adjust_spike_threshold(DEGENERATE_PENALTY_SD);
}

void ShenandoahAdaptiveHeuristics::record_success_full() {
  ShenandoahHeuristics::record_success_full();
  // Adjust both trigger's parameters in the case of a full GC because
  // either of them should have triggered earlier to avoid this case.
  adjust_margin_of_error(FULL_PENALTY_SD);
  adjust_spike_threshold(FULL_PENALTY_SD);
}

static double saturate(double value, double min, double max) {
  return MAX2(MIN2(value, max), min);
}

bool ShenandoahAdaptiveHeuristics::should_start_gc() {
  size_t capacity = _space_info->soft_max_capacity();
  size_t available = _space_info->soft_available();
  size_t allocated = _space_info->bytes_allocated_since_gc_start();

  log_debug(gc)("should_start_gc (%s)? available: " SIZE_FORMAT ", soft_max_capacity: " SIZE_FORMAT
                ", allocated: " SIZE_FORMAT,
                _space_info->name(), available, capacity, allocated);

  // Track allocation rate even if we decide to start a cycle for other reasons.
  double rate = _allocation_rate.sample(allocated);
  // double rate_user = _allocation_rate_user.sample(allocated);
  // log_info(gc)("Sampled Allocation rate: %lf", rate);
  _last_trigger = OTHER;

  size_t min_threshold = min_free_threshold();
  if (available < min_threshold) {
    log_info(gc)("Trigger (%s): Free (" SIZE_FORMAT "%s) is below minimum threshold (" SIZE_FORMAT "%s)", _space_info->name(),
                 byte_size_in_proper_unit(available), proper_unit_for_byte_size(available),
                 byte_size_in_proper_unit(min_threshold), proper_unit_for_byte_size(min_threshold));
    return true;
  }

  // Check if we need to learn a bit about the application
  const size_t max_learn = ShenandoahLearningSteps;
  if (_gc_times_learned < max_learn) {
    size_t init_threshold = capacity / 100 * ShenandoahInitFreeThreshold;
    if (available < init_threshold) {
      log_info(gc)("Trigger (%s): Learning " SIZE_FORMAT " of " SIZE_FORMAT ". Free (" SIZE_FORMAT "%s) is below initial threshold (" SIZE_FORMAT "%s)",
                   _space_info->name(), _gc_times_learned + 1, max_learn,
                   byte_size_in_proper_unit(available), proper_unit_for_byte_size(available),
                   byte_size_in_proper_unit(init_threshold), proper_unit_for_byte_size(init_threshold));
      return true;
    }
  }
  //  Rationale:
  //    The idea is that there is an average allocation rate and there are occasional abnormal bursts (or spikes) of
  //    allocations that exceed the average allocation rate.  What do these spikes look like?
  //
  //    1. At certain phase changes, we may discard large amounts of data and replace it with large numbers of newly
  //       allocated objects.  This "spike" looks more like a phase change.  We were in steady state at M bytes/sec
  //       allocation rate and now we're in a "reinitialization phase" that looks like N bytes/sec.  We need the "spike"
  //       accommodation to give us enough runway to recalibrate our "average allocation rate".
  //
  //   2. The typical workload changes.  "Suddenly", our typical workload of N TPS increases to N+delta TPS.  This means
  //       our average allocation rate needs to be adjusted.  Once again, we need the "spike" accomodation to give us
  //       enough runway to recalibrate our "average allocation rate".
  //
  //    3. Though there is an "average" allocation rate, a given workload's demand for allocation may be very bursty.  We
  //       allocate a bunch of LABs during the 5 ms that follow completion of a GC, then we perform no more allocations for
  //       the next 150 ms.  It seems we want the "spike" to represent the maximum divergence from average within the
  //       period of time between consecutive evaluation of the should_start_gc() service.  Here's the thinking:
  //
  //       a) Between now and the next time I ask whether should_start_gc(), we might experience a spike representing
  //          the anticipated burst of allocations.  If that would put us over budget, then we should start GC immediately.
  //       b) Between now and the anticipated depletion of allocation pool, there may be two or more bursts of allocations.
  //          If there are more than one of these bursts, we can "approximate" that these will be separated by spans of
  //          time with very little or no allocations so the "average" allocation rate should be a suitable approximation
  //          of how this will behave.
  //
  //    For cases 1 and 2, we need to "quickly" recalibrate the average allocation rate whenever we detect a change
  //    in operation mode.  We want some way to decide that the average rate has changed.  Make average allocation rate
  //    computations an independent effort.
  // Check if allocation headroom is still okay. This also factors in:
  //   1. Some space to absorb allocation spikes (ShenandoahAllocSpikeFactor)
  //   2. Accumulated penalties from Degenerated and Full GC
  size_t allocation_headroom = available;

  size_t spike_headroom = capacity / 100 * ShenandoahAllocSpikeFactor;
  size_t penalties      = capacity / 100 * _gc_time_penalties;

  allocation_headroom -= MIN2(allocation_headroom, spike_headroom);
  allocation_headroom -= MIN2(allocation_headroom, penalties);

  double avg_cycle_time = _gc_cycle_time_history->davg() + (_margin_of_error_sd * _gc_cycle_time_history->dsd());
  double avg_alloc_rate = _allocation_rate.upper_bound(_margin_of_error_sd);
  log_debug(gc)("%s: average GC time: %.2f ms, allocation rate: %.0f %s/s",
                _space_info->name(),
          avg_cycle_time * 1000, byte_size_in_proper_unit(avg_alloc_rate), proper_unit_for_byte_size(avg_alloc_rate));
  if (avg_cycle_time > allocation_headroom / avg_alloc_rate) {
    log_info(gc)("Trigger (%s): Average GC time (%.2f ms) is above the time for average allocation rate (%.0f %sB/s)"
                 " to deplete free headroom (" SIZE_FORMAT "%s) (margin of error = %.2f)",
                 _space_info->name(), avg_cycle_time * 1000,
                 byte_size_in_proper_unit(avg_alloc_rate), proper_unit_for_byte_size(avg_alloc_rate),
                 byte_size_in_proper_unit(allocation_headroom), proper_unit_for_byte_size(allocation_headroom),
                 _margin_of_error_sd);
    log_info(gc, ergo)("Free headroom: " SIZE_FORMAT "%s (free) - " SIZE_FORMAT "%s (spike) - " SIZE_FORMAT "%s (penalties) = " SIZE_FORMAT "%s",
                       byte_size_in_proper_unit(available),           proper_unit_for_byte_size(available),
                       byte_size_in_proper_unit(spike_headroom),      proper_unit_for_byte_size(spike_headroom),
                       byte_size_in_proper_unit(penalties),           proper_unit_for_byte_size(penalties),
                       byte_size_in_proper_unit(allocation_headroom), proper_unit_for_byte_size(allocation_headroom));
    _last_trigger = RATE;
    return true;
  }

  bool is_spiking = _allocation_rate.is_spiking(rate, _spike_threshold_sd);
  if (is_spiking && avg_cycle_time > allocation_headroom / rate) {
    log_info(gc)("Trigger (%s): Average GC time (%.2f ms) is above the time for instantaneous allocation rate (%.0f %sB/s) to deplete free headroom (" SIZE_FORMAT "%s) (spike threshold = %.2f)",
                 _space_info->name(), avg_cycle_time * 1000,
                 byte_size_in_proper_unit(rate), proper_unit_for_byte_size(rate),
                 byte_size_in_proper_unit(allocation_headroom), proper_unit_for_byte_size(allocation_headroom),
                 _spike_threshold_sd);
    _last_trigger = SPIKE;
    return true;
  }

  return ShenandoahHeuristics::should_start_gc();
}

void ShenandoahAdaptiveHeuristics::adjust_last_trigger_parameters(double amount) {
  switch (_last_trigger) {
    case RATE:
      adjust_margin_of_error(amount);
      break;
    case SPIKE:
      adjust_spike_threshold(amount);
      break;
    case OTHER:
      // nothing to adjust here.
      break;
    default:
      ShouldNotReachHere();
  }
}

void ShenandoahAdaptiveHeuristics::adjust_margin_of_error(double amount) {
  _margin_of_error_sd = saturate(_margin_of_error_sd + amount, MINIMUM_CONFIDENCE, MAXIMUM_CONFIDENCE);
  log_debug(gc, ergo)("Margin of error now %.2f", _margin_of_error_sd);
}

void ShenandoahAdaptiveHeuristics::adjust_spike_threshold(double amount) {
  _spike_threshold_sd = saturate(_spike_threshold_sd - amount, MINIMUM_CONFIDENCE, MAXIMUM_CONFIDENCE);
  log_debug(gc, ergo)("Spike threshold now: %.2f", _spike_threshold_sd);
}

size_t ShenandoahAdaptiveHeuristics::min_free_threshold() {
  // Note that soft_max_capacity() / 100 * min_free_threshold is smaller than max_capacity() / 100 * min_free_threshold.
  // We want to behave conservatively here, so use max_capacity().  By returning a larger value, we cause the GC to
  // trigger when the remaining amount of free shrinks below the larger threshold.
  return _space_info->max_capacity() / 100 * ShenandoahMinFreeThreshold;
}

ShenandoahAllocationRate::ShenandoahAllocationRate() :
  _last_sample_time(os::elapsedTime()),
  _last_sample_value(0),
  _interval_sec(1.0 / ShenandoahAdaptiveSampleFrequencyHz),
  _rate(int(ShenandoahAdaptiveSampleSizeSeconds * ShenandoahAdaptiveSampleFrequencyHz), ShenandoahAdaptiveDecayFactor),
  _rate_avg(int(ShenandoahAdaptiveSampleSizeSeconds * ShenandoahAdaptiveSampleFrequencyHz), ShenandoahAdaptiveDecayFactor) {
}

double ShenandoahAllocationRate::sample(size_t allocated) {
  double now = os::elapsedTime();
  double rate = 0.0;
  if (now - _last_sample_time > _interval_sec) {
    if (allocated >= _last_sample_value) {
      rate = instantaneous_rate(now, allocated);
      _rate.add(rate);
      _rate_avg.add(_rate.avg());
    }

    _last_sample_time = now;
    _last_sample_value = allocated;
  }
  return rate;
}

double ShenandoahAllocationRate::upper_bound(double sds) const {
  // Here we are using the standard deviation of the computed running
  // average, rather than the standard deviation of the samples that went
  // into the moving average. This is a much more stable value and is tied
  // to the actual statistic in use (moving average over samples of averages).
  return _rate.davg() + (sds * _rate_avg.dsd());
}

void ShenandoahAllocationRate::allocation_counter_reset() {
  _last_sample_time = os::elapsedTime();
  _last_sample_value = 0;
}

bool ShenandoahAllocationRate::is_spiking(double rate, double threshold) const {
  if (rate <= 0.0) {
    return false;
  }

  double sd = _rate.sd();
  if (sd > 0) {
    // There is a small chance that that rate has already been sampled, but it
    // seems not to matter in practice.
    double z_score = (rate - _rate.avg()) / sd;
    if (z_score > threshold) {
      return true;
    }
  }
  return false;
}

double ShenandoahAllocationRate::instantaneous_rate(double time, size_t allocated) const {
  size_t last_value = _last_sample_value;
  double last_time = _last_sample_time;
  size_t allocation_delta = (allocated > last_value) ? (allocated - last_value) : 0;
  double time_delta_sec = time - last_time;
  return (time_delta_sec > 0)  ? (allocation_delta / time_delta_sec) : 0;
}

ShenandoahAllocationRateUser::ShenandoahAllocationRateUser() :
  _last_sample_value(0),
  _interval_sec(1.0 / ShenandoahAdaptiveSampleFrequencyHz),
  _rate(int(ShenandoahAdaptiveSampleSizeSeconds * ShenandoahAdaptiveSampleFrequencyHz), ShenandoahAdaptiveDecayFactor),
  _rate_avg(int(ShenandoahAdaptiveSampleSizeSeconds * ShenandoahAdaptiveSampleFrequencyHz), ShenandoahAdaptiveDecayFactor) {
    // double real_time, user_time, system_time;
    // bool valid = os::getTimesSecs(&real_time, &user_time, &system_time);
    // _last_sample_time = user_time;

    // to record alloc_rate, only consider mutator thread(jthread)
    long user_time = 0, system_time = 0;
    os::get_accum_jthread_time_by_sub(&user_time, &system_time);
    _last_sample_time = (double) user_time / 1000.0;
}

double ShenandoahAllocationRateUser::sample(size_t allocated) {
  // double now = os::elapsedTime();
  // double real_time, now, system_time;
  // bool valid = os::getTimesSecs(&real_time, &now, &system_time);
  long user_time = 0, system_time = 0;
  os::get_accum_jthread_time_by_sub(&user_time, &system_time);
  // size_t thread_exit_elapsed_time = os::thread_exit_elapsed_time();
  // log_info(gc) ("thread_exit_elapsed_time: %lf", (double) thread_exit_elapsed_time / 1000000.0);
  log_info(gc) ("get_accum_jthread_usertime: %lf", (double) user_time / 1000.0);
  // double now = (double) user_time / 1000.0 + (double) thread_exit_elapsed_time / 1000000.0 ;
  double now = (double) user_time / 1000.0;
  double rate = 0.0;
  log_info(gc) ("allocated: %lu, now: %lf, last_sample_time: %lf", allocated, now, _last_sample_time);
  if (now - _last_sample_time > 0) {
    // if (allocated >= _last_sample_value) {
      // rate = instantaneous_rate(now, allocated);
      rate = allocated * 1.0 / (now - _last_sample_time);
      
      // _rate.add(rate);
      // _rate_avg.add(_rate.avg());
    // }
    _last_sample_time = now;
    // _last_sample_value = allocated;
  } else {
    log_info(gc) ("now(%lf) <= last_sample_time(%lf)", now, _last_sample_time);
  }
  return rate;
}

double ShenandoahAllocationRateUser::upper_bound(double sds) const {
  // Here we are using the standard deviation of the computed running
  // average, rather than the standard deviation of the samples that went
  // into the moving average. This is a much more stable value and is tied
  // to the actual statistic in use (moving average over samples of averages).
  return _rate.davg() + (sds * _rate_avg.dsd());
}

void ShenandoahAllocationRateUser::allocation_counter_reset() {
  // double real_time, user_time, system_time;
  // bool valid = os::getTimesSecs(&real_time, &user_time, &system_time);
  // _last_sample_time = user_time;
  long user_time = 0, system_time = 0;
  os::get_accum_jthread_time_by_sub(&user_time, &system_time);
  _last_sample_time = (double) user_time / 1000.0;
  _last_sample_value = 0;
}

bool ShenandoahAllocationRateUser::is_spiking(double rate, double threshold) const {
  if (rate <= 0.0) {
    return false;
  }

  double sd = _rate.sd();
  if (sd > 0) {
    // There is a small chance that that rate has already been sampled, but it
    // seems not to matter in practice.
    double z_score = (rate - _rate.avg()) / sd;
    if (z_score > threshold) {
      return true;
    }
  }
  return false;
}

double ShenandoahAllocationRateUser::instantaneous_rate(double time, size_t allocated) const {
  size_t last_value = _last_sample_value;
  double last_time = _last_sample_time;
  size_t allocation_delta = (allocated > last_value) ? (allocated - last_value) : 0;
  double time_delta_sec = time - last_time;
  return (time_delta_sec > 0)  ? (allocation_delta / time_delta_sec) : 0;
}

void ShenandoahAdaptiveHeuristics::print_info() {
  // double avg_cycle_time = _gc_cycle_time_history->davg() + (_margin_of_error_sd * _gc_cycle_time_history->dsd());
  // log_info(gc)("%s: average GC time: %.2f ms, allocation rate: %.0f %s/s", 
  //               _space_info->name(), avg_cycle_time * 1000, byte_size_in_proper_unit(avg_alloc_rate), proper_unit_for_byte_size(avg_alloc_rate));
  // in milliseconds
  ShenandoahHeap* heap = ShenandoahHeap::heap();
  double gc_cycle_time = heap->copy_wall_time();
  // elapsed_cycle_time() * 1000.0; // ticks
  double gc_cycle_total_time = (heap->copy_user_time() + heap->copy_sys_time()); // njt user
  double gc_cycle_user_time = heap->copy_user_time(); // njt user + sys

  size_t copy_bytes_during_gc = _copy_bytes_during_gc;
  // double avg_alloc_rate = _allocation_rate.upper_bound(_margin_of_error_sd);
  // only mutator thread
  // double avg_alloc_rate_user = _allocation_rate_user.upper_bound(_margin_of_error_sd);
  
  // log_info(gc)("%s: [wall] GC time: %.2f ms, allocation rate: %.0f %s/s; [user] GC time: %.2f ms, allocation rate: %.0f %s/s", 
  //                 _space_info->name(), gc_cycle_time * 1000, byte_size_in_proper_unit(avg_alloc_rate), proper_unit_for_byte_size(avg_alloc_rate), gc_cycle_user_time * 1000, byte_size_in_proper_unit(avg_alloc_rate_user), proper_unit_for_byte_size(avg_alloc_rate_user));
  // log_info(gc) ("%s GC cost per byte:  [User]: %lf", _space_info->name())
  if (copy_bytes_during_gc != 0) {
    log_info(gc) ("copy/expected: %lf", (double) copy_bytes_during_gc / _copy_bytes_expected);
    log_info(gc) ("copy_bytes_during_gc: %lu", copy_bytes_during_gc);
    log_info(gc) ("gc_cycle_user_time: %lfms, gc_cycle_total_time: %lfms, gc_cycle_time: %lfms", gc_cycle_user_time, gc_cycle_total_time, gc_cycle_time);
    log_info(gc) ("[User] cost_per_byte: %lfms; [User+Sys] cost_per_byte: %lfms; [Ticks] cost_per_byte: %lfms", gc_cycle_user_time / copy_bytes_during_gc, gc_cycle_total_time / copy_bytes_during_gc, gc_cycle_time / copy_bytes_during_gc);
  }
}

bool ShenandoahPhaseDependentSeq::enough_samples_to_use_mixed_seq() const {
  return ShenandoahAnalytics::enough_samples_available(&_mixed_seq);
}

ShenandoahPhaseDependentSeq::ShenandoahPhaseDependentSeq(int length) :
  _young_only_seq(length),
  _mixed_seq(length)
{ }

TruncatedSeq* ShenandoahPhaseDependentSeq::seq_raw(bool use_young_only_phase_seq) {
  return use_young_only_phase_seq ? &_young_only_seq : &_mixed_seq;
}

void ShenandoahPhaseDependentSeq::set_initial(double value) {
  _young_only_seq.add(value);
}

void ShenandoahPhaseDependentSeq::add(double value, bool for_young_only_phase) {
  seq_raw(for_young_only_phase)->add(value);
}

double ShenandoahPhaseDependentSeq::predict(const ShenandoahPredictions* predictor, bool use_young_only_phase_seq) const {
  if (use_young_only_phase_seq || !enough_samples_to_use_mixed_seq()) {
    return predictor->predict(&_young_only_seq);
  } else {
    return predictor->predict(&_mixed_seq);
  }
}



static double cost_per_logged_card_ms_defaults[] = {
  0.01, 0.005, 0.005, 0.003, 0.003, 0.002, 0.002, 0.0015
};

// all the same
static double young_card_scan_to_merge_ratio_defaults[] = {
  1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0
};

static double young_only_cost_per_card_scan_ms_defaults[] = {
  0.015, 0.01, 0.01, 0.008, 0.008, 0.0055, 0.0055, 0.005
};

static double cost_per_byte_ms_defaults[] = {
  0.00006, 0.00003, 0.00003, 0.000015, 0.000015, 0.00001, 0.00001, 0.000009
};

// these should be pretty consistent
static double constant_other_time_ms_defaults[] = {
  5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0
};

static double young_other_cost_per_region_ms_defaults[] = {
  0.3, 0.2, 0.2, 0.15, 0.15, 0.12, 0.12, 0.1
};

static double non_young_other_cost_per_region_ms_defaults[] = {
  1.0, 0.7, 0.7, 0.5, 0.5, 0.42, 0.42, 0.30
};

ShenandoahAnalytics::ShenandoahAnalytics(const ShenandoahPredictions* predictor) :
    _predictor(predictor),
    _recent_gc_times_ms(NumPrevPausesForHeuristics),
    _concurrent_mark_remark_times_ms(NumPrevPausesForHeuristics),
    _concurrent_mark_cleanup_times_ms(NumPrevPausesForHeuristics),
    _alloc_rate_ms_seq(TruncatedSeqLength),
    _prev_collection_pause_end_ms(0.0),
    _concurrent_refine_rate_ms_seq(TruncatedSeqLength),
    _dirtied_cards_rate_ms_seq(TruncatedSeqLength),
    _dirtied_cards_in_thread_buffers_seq(TruncatedSeqLength),
    _card_scan_to_merge_ratio_seq(TruncatedSeqLength),
    _cost_per_card_scan_ms_seq(TruncatedSeqLength),
    _cost_per_card_merge_ms_seq(TruncatedSeqLength),
    _cost_per_byte_copied_ms_seq(TruncatedSeqLength),
    _cost_per_card_scan_user_seq(TruncatedSeqLength),
    _cost_per_card_merge_cpu_seq(TruncatedSeqLength),
    _cost_per_byte_copied_user_seq(TruncatedSeqLength),
    _pending_cards_seq(TruncatedSeqLength),
    _rs_length_seq(TruncatedSeqLength),
    _constant_other_time_ms_seq(TruncatedSeqLength),
    _young_other_cost_per_region_ms_seq(TruncatedSeqLength),
    _non_young_other_cost_per_region_ms_seq(TruncatedSeqLength),
    _recent_prev_end_times_for_all_gcs_sec(NumPrevPausesForHeuristics),
    _long_term_pause_time_ratio(0.0),
    _short_term_pause_time_ratio(0.0) {

  // Seed sequences with initial values.
  _recent_prev_end_times_for_all_gcs_sec.add(os::elapsedTime());
  _prev_collection_pause_end_ms = os::elapsedTime() * 1000.0;

  int index = MIN2(ParallelGCThreads - 1, 7u);

  // Start with inverse of maximum STW cost.
  _concurrent_refine_rate_ms_seq.add(1/cost_per_logged_card_ms_defaults[0]);
  // Some applications have very low rates for logging cards.
  _dirtied_cards_rate_ms_seq.add(0.0);

  _card_scan_to_merge_ratio_seq.set_initial(young_card_scan_to_merge_ratio_defaults[index]);
  _cost_per_card_scan_ms_seq.set_initial(young_only_cost_per_card_scan_ms_defaults[index]);
  _cost_per_card_scan_user_seq.set_initial(young_only_cost_per_card_scan_ms_defaults[index]);

  _rs_length_seq.set_initial(0);
  _cost_per_byte_copied_ms_seq.set_initial(cost_per_byte_ms_defaults[index]);
  _cost_per_byte_copied_user_seq.set_initial(cost_per_byte_ms_defaults[index]);


  _constant_other_time_ms_seq.add(constant_other_time_ms_defaults[index]);
  _young_other_cost_per_region_ms_seq.add(young_other_cost_per_region_ms_defaults[index]);
  _non_young_other_cost_per_region_ms_seq.add(non_young_other_cost_per_region_ms_defaults[index]);

  // start conservatively (around 50ms is about right)
  _concurrent_mark_remark_times_ms.add(0.05);
  _concurrent_mark_cleanup_times_ms.add(0.20);
}

bool ShenandoahAnalytics::enough_samples_available(TruncatedSeq const* seq) {
  return seq->num() >= 3;
}

double ShenandoahAnalytics::predict_in_unit_interval(TruncatedSeq const* seq) const {
  return _predictor->predict_in_unit_interval(seq);
}

size_t ShenandoahAnalytics::predict_size(TruncatedSeq const* seq) const {
  return (size_t)predict_zero_bounded(seq);
}

double ShenandoahAnalytics::predict_zero_bounded(TruncatedSeq const* seq) const {
  return _predictor->predict_zero_bounded(seq);
}

double ShenandoahAnalytics::predict_in_unit_interval(ShenandoahPhaseDependentSeq const* seq, bool for_young_only_phase) const {
  return clamp(seq->predict(_predictor, for_young_only_phase), 0.0, 1.0);
}

size_t ShenandoahAnalytics::predict_size(ShenandoahPhaseDependentSeq const* seq, bool for_young_only_phase) const {
  return (size_t)predict_zero_bounded(seq, for_young_only_phase);
}

double ShenandoahAnalytics::predict_zero_bounded(ShenandoahPhaseDependentSeq const* seq, bool for_young_only_phase) const {
  return MAX2(seq->predict(_predictor, for_young_only_phase), 0.0);
}

int ShenandoahAnalytics::num_alloc_rate_ms() const {
  return _alloc_rate_ms_seq.num();
}

void ShenandoahAnalytics::report_concurrent_mark_remark_times_ms(double ms) {
  _concurrent_mark_remark_times_ms.add(ms);
}

void ShenandoahAnalytics::report_alloc_rate_ms(double alloc_rate) {
  _alloc_rate_ms_seq.add(alloc_rate);
}

void ShenandoahAnalytics::compute_pause_time_ratios(double end_time_sec, double pause_time_ms) {
  double long_interval_ms = (end_time_sec - oldest_known_gc_end_time_sec()) * 1000.0;
  double gc_pause_time_ms = _recent_gc_times_ms.sum() - _recent_gc_times_ms.oldest() + pause_time_ms;
  _long_term_pause_time_ratio = gc_pause_time_ms / long_interval_ms;
  _long_term_pause_time_ratio = clamp(_long_term_pause_time_ratio, 0.0, 1.0);

  double short_interval_ms = (end_time_sec - most_recent_gc_end_time_sec()) * 1000.0;
  _short_term_pause_time_ratio = pause_time_ms / short_interval_ms;
  _short_term_pause_time_ratio = clamp(_short_term_pause_time_ratio, 0.0, 1.0);
}

void ShenandoahAnalytics::report_concurrent_refine_rate_ms(double cards_per_ms) {
  _concurrent_refine_rate_ms_seq.add(cards_per_ms);
}

void ShenandoahAnalytics::report_dirtied_cards_rate_ms(double cards_per_ms) {
  _dirtied_cards_rate_ms_seq.add(cards_per_ms);
}

void ShenandoahAnalytics::report_dirtied_cards_in_thread_buffers(size_t cards) {
  _dirtied_cards_in_thread_buffers_seq.add(double(cards));
}

void ShenandoahAnalytics::report_cost_per_card_scan_ms(double cost_per_card_ms, bool for_young_only_phase) {
  _cost_per_card_scan_ms_seq.add(cost_per_card_ms, for_young_only_phase);
}

void ShenandoahAnalytics::report_cost_per_card_merge_ms(double cost_per_card_ms, bool for_young_only_phase) {
  _cost_per_card_merge_ms_seq.add(cost_per_card_ms, for_young_only_phase);
}

void ShenandoahAnalytics::report_card_scan_to_merge_ratio(double merge_to_scan_ratio, bool for_young_only_phase) {
  _card_scan_to_merge_ratio_seq.add(merge_to_scan_ratio, for_young_only_phase);
}

void ShenandoahAnalytics::report_cost_per_byte_ms(double cost_per_byte_ms, bool for_young_only_phase) {
  _cost_per_byte_copied_ms_seq.add(cost_per_byte_ms, for_young_only_phase);
}

void ShenandoahAnalytics::report_cost_per_byte_cpu(double cost_per_byte_cpu, bool for_young_only_phase) {
  _cost_per_byte_copied_user_seq.add(cost_per_byte_cpu, for_young_only_phase);
}

void ShenandoahAnalytics::report_young_other_cost_per_region_ms(double other_cost_per_region_ms) {
  _young_other_cost_per_region_ms_seq.add(other_cost_per_region_ms);
}

void ShenandoahAnalytics::report_non_young_other_cost_per_region_ms(double other_cost_per_region_ms) {
  _non_young_other_cost_per_region_ms_seq.add(other_cost_per_region_ms);
}

void ShenandoahAnalytics::report_constant_other_time_ms(double constant_other_time_ms) {
  _constant_other_time_ms_seq.add(constant_other_time_ms);
}

void ShenandoahAnalytics::report_pending_cards(double pending_cards, bool for_young_only_phase) {
  _pending_cards_seq.add(pending_cards, for_young_only_phase);
}

void ShenandoahAnalytics::report_rs_length(double rs_length, bool for_young_only_phase) {
  _rs_length_seq.add(rs_length, for_young_only_phase);
}

double ShenandoahAnalytics::predict_alloc_rate_ms() const {
  if (enough_samples_available(&_alloc_rate_ms_seq)) {
    return predict_zero_bounded(&_alloc_rate_ms_seq);
  } else {
    return 0.0;
  }
}

double ShenandoahAnalytics::predict_concurrent_refine_rate_ms() const {
  return predict_zero_bounded(&_concurrent_refine_rate_ms_seq);
}

double ShenandoahAnalytics::predict_dirtied_cards_rate_ms() const {
  return predict_zero_bounded(&_dirtied_cards_rate_ms_seq);
}

size_t ShenandoahAnalytics::predict_dirtied_cards_in_thread_buffers() const {
  return predict_size(&_dirtied_cards_in_thread_buffers_seq);
}

size_t ShenandoahAnalytics::predict_scan_card_num(size_t rs_length, bool for_young_only_phase) const {
  return rs_length * predict_in_unit_interval(&_card_scan_to_merge_ratio_seq, for_young_only_phase);
}

double ShenandoahAnalytics::predict_card_merge_time_ms(size_t card_num, bool for_young_only_phase) const {
  return card_num * predict_zero_bounded(&_cost_per_card_merge_ms_seq, for_young_only_phase);
}

double ShenandoahAnalytics::predict_card_scan_time_ms(size_t card_num, bool for_young_only_phase) const {
  return card_num * predict_zero_bounded(&_cost_per_card_scan_ms_seq, for_young_only_phase);
}

double ShenandoahAnalytics::predict_object_copy_time_ms(size_t bytes_to_copy, bool for_young_only_phase) const {
  return bytes_to_copy * predict_zero_bounded(&_cost_per_byte_copied_ms_seq, for_young_only_phase);
}

double ShenandoahAnalytics::predict_constant_other_time_ms() const {
  return predict_zero_bounded(&_constant_other_time_ms_seq);
}

double ShenandoahAnalytics::predict_young_other_time_ms(size_t young_num) const {
  return young_num * predict_zero_bounded(&_young_other_cost_per_region_ms_seq);
}

double ShenandoahAnalytics::predict_non_young_other_time_ms(size_t non_young_num) const {
  return non_young_num * predict_zero_bounded(&_non_young_other_cost_per_region_ms_seq);
}

double ShenandoahAnalytics::predict_remark_time_ms() const {
  return predict_zero_bounded(&_concurrent_mark_remark_times_ms);
}

double ShenandoahAnalytics::predict_cleanup_time_ms() const {
  return predict_zero_bounded(&_concurrent_mark_cleanup_times_ms);
}

size_t ShenandoahAnalytics::predict_rs_length(bool for_young_only_phase) const {
  return predict_size(&_rs_length_seq, for_young_only_phase);
}

size_t ShenandoahAnalytics::predict_pending_cards(bool for_young_only_phase) const {
  return predict_size(&_pending_cards_seq, for_young_only_phase);
}

double ShenandoahAnalytics::oldest_known_gc_end_time_sec() const {
  return _recent_prev_end_times_for_all_gcs_sec.oldest();
}

double ShenandoahAnalytics::most_recent_gc_end_time_sec() const {
  return _recent_prev_end_times_for_all_gcs_sec.last();
}

void ShenandoahAnalytics::update_recent_gc_times(double end_time_sec,
                                         double pause_time_ms) {
  _recent_gc_times_ms.add(pause_time_ms);
  _recent_prev_end_times_for_all_gcs_sec.add(end_time_sec);
}

void ShenandoahAnalytics::report_concurrent_mark_cleanup_times_ms(double ms) {
  _concurrent_mark_cleanup_times_ms.add(ms);
}
