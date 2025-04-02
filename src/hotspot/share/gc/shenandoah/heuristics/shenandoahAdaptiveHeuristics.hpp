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

#ifndef SHARE_GC_SHENANDOAH_HEURISTICS_SHENANDOAHADAPTIVEHEURISTICS_HPP
#define SHARE_GC_SHENANDOAH_HEURISTICS_SHENANDOAHADAPTIVEHEURISTICS_HPP

#include "runtime/globals_extension.hpp"
#include "memory/allocation.hpp"
#include "gc/shenandoah/heuristics/shenandoahHeuristics.hpp"
#include "gc/shenandoah/heuristics/shenandoahSpaceInfo.hpp"
#include "gc/shenandoah/shenandoahPhaseTimings.hpp"
#include "gc/shenandoah/shenandoahSharedVariables.hpp"
#include "utilities/numberSeq.hpp"

class ShenandoahAllocationRate : public CHeapObj<mtGC> {
 public:
  explicit ShenandoahAllocationRate();
  void allocation_counter_reset();

  double sample(size_t allocated);

  double upper_bound(double sds) const;
  bool is_spiking(double rate, double threshold) const;
 private:

  double instantaneous_rate(double time, size_t allocated) const;

  double _last_sample_time;
  size_t _last_sample_value;
  double _interval_sec;
  TruncatedSeq _rate;
  TruncatedSeq _rate_avg;
};

class ShenandoahAllocationRateUser : public CHeapObj<mtGC> {
  public:
   explicit ShenandoahAllocationRateUser();
   void allocation_counter_reset();
 
   double sample(size_t allocated);
 
   double upper_bound(double sds) const;
   bool is_spiking(double rate, double threshold) const;
  private:
 
   double instantaneous_rate(double time, size_t allocated) const;
 
   // user time
   double _last_sample_time;
   size_t _last_sample_value;
   double _interval_sec;
   TruncatedSeq _rate;
   TruncatedSeq _rate_avg;
 };

 class ShenandoahPredictions {
  private:
   double _sigma;
 
   // This function is used to estimate the stddev of sample sets. There is some
   // special consideration of small sample sets: the actual stddev for them is
   // not very useful, so we calculate some value based on the sample average.
   // Five or more samples yields zero (at that point we use the stddev); fewer
   // scale the sample set average linearly from two times the average to 0.5 times
   // it.
   double stddev_estimate(TruncatedSeq const* seq) const {
     double estimate = seq->dsd();
     int const samples = seq->num();
     if (samples < 5) {
       estimate = MAX2(seq->davg() * (5 - samples) / 2.0, estimate);
     }
     return estimate;
   }
  public:
   ShenandoahPredictions(double sigma) : _sigma(sigma) {
     assert(sigma >= 0.0, "Confidence must be larger than or equal to zero");
   }
 
   // Confidence factor.
   double sigma() const { return _sigma; }
 
   double predict(TruncatedSeq const* seq) const {
     return seq->davg() + _sigma * stddev_estimate(seq);
   }
 
   double predict_in_unit_interval(TruncatedSeq const* seq) const {
     return clamp(predict(seq), 0.0, 1.0);
   }
 
   double predict_zero_bounded(TruncatedSeq const* seq) const {
     return MAX2(predict(seq), 0.0);
   }
 };

 class ShenandoahPhaseDependentSeq {
  TruncatedSeq _young_only_seq;
  TruncatedSeq _mixed_seq;

  NONCOPYABLE(ShenandoahPhaseDependentSeq);

  TruncatedSeq* seq_raw(bool use_young_only_phase_seq);

  bool enough_samples_to_use_mixed_seq() const;
public:

  ShenandoahPhaseDependentSeq(int length);

  void set_initial(double value);
  void add(double value, bool for_young_only_phase);

  double predict(const ShenandoahPredictions* predictor, bool use_young_only_phase_seq) const;
};

/*
 * The adaptive heuristic tracks the allocation behavior and average cycle
 * time of the application. It attempts to start a cycle with enough time
 * to complete before the available memory is exhausted. It errors on the
 * side of starting cycles early to avoid allocation failures (degenerated
 * cycles).
 *
 * This heuristic limits the number of regions for evacuation such that the
 * evacuation reserve is respected. This helps it avoid allocation failures
 * during evacuation. It preferentially selects regions with the most garbage.
 */
class ShenandoahAdaptiveHeuristics : public ShenandoahHeuristics {
public:
  ShenandoahAdaptiveHeuristics(ShenandoahSpaceInfo* space_info);

  virtual ~ShenandoahAdaptiveHeuristics();

  virtual void choose_collection_set_from_regiondata(ShenandoahCollectionSet* cset,
                                                     RegionData* data, size_t size,
                                                     size_t actual_free);

  void record_cycle_start();
  void record_success_concurrent(bool abbreviated);
  void record_success_degenerated();
  void record_success_full();
  void print_info();

  virtual bool should_start_gc();

  virtual const char* name()     { return "Adaptive"; }
  virtual bool is_diagnostic()   { return false; }
  virtual bool is_experimental() { return false; }

 private:
  // These are used to adjust the margin of error and the spike threshold
  // in response to GC cycle outcomes. These values are shared, but the
  // margin of error and spike threshold trend in opposite directions.
  const static double FULL_PENALTY_SD;
  const static double DEGENERATE_PENALTY_SD;

  const static double MINIMUM_CONFIDENCE;
  const static double MAXIMUM_CONFIDENCE;

  const static double LOWEST_EXPECTED_AVAILABLE_AT_END;
  const static double HIGHEST_EXPECTED_AVAILABLE_AT_END;

  friend class ShenandoahAllocationRate;
  friend class ShenandoahAllocationRateUser;

  // Used to record the last trigger that signaled to start a GC.
  // This itself is used to decide whether or not to adjust the margin of
  // error for the average cycle time and allocation rate or the allocation
  // spike detection threshold.
  enum Trigger {
    SPIKE, RATE, OTHER
  };

  void adjust_last_trigger_parameters(double amount);
  void adjust_margin_of_error(double amount);
  void adjust_spike_threshold(double amount);

public:
  ShenandoahAllocationRate _allocation_rate;
  ShenandoahAllocationRateUser _allocation_rate_user;
  size_t _copy_bytes_during_gc;

  // The margin of error expressed in standard deviations to add to our
  // average cycle time and allocation rate. As this value increases we
  // tend to overestimate the rate at which mutators will deplete the
  // heap. In other words, erring on the side of caution will trigger more
  // concurrent GCs.
  double _margin_of_error_sd;

  // The allocation spike threshold is expressed in standard deviations.
  // If the standard deviation of the most recent sample of the allocation
  // rate exceeds this threshold, a GC cycle is started. As this value
  // decreases the sensitivity to allocation spikes increases. In other
  // words, lowering the spike threshold will tend to increase the number
  // of concurrent GCs.
  double _spike_threshold_sd;

  // Remember which trigger is responsible for the last GC cycle. When the
  // outcome of the cycle is evaluated we will adjust the parameters for the
  // corresponding triggers. Note that successful outcomes will raise
  // the spike threshold and lower the margin of error.
  Trigger _last_trigger;

  // Keep track of the available memory at the end of a GC cycle. This
  // establishes what is 'normal' for the application and is used as a
  // source of feedback to adjust trigger parameters.
  TruncatedSeq _available;

  size_t min_free_threshold();
};

#endif // SHARE_GC_SHENANDOAH_HEURISTICS_SHENANDOAHADAPTIVEHEURISTICS_HPP
