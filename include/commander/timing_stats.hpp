/* Copyright 2023, Cinar, A. L.
 * SPDX-License-Identifier: MIT
 */

#include <chrono>

class TimingStats
{
  public:
	TimingStats();
	void sampling_check();

	size_t run_count;
	size_t violation_count;
	double avg_rate;
	double avg_margin;
	double avg_elapsed;
	double min_margin;
	double max_elapsed;

	size_t run_count_accum;
	double margin_accum;
	double elapsed_accum;

  private:
	void reset_accum();
	std::chrono::high_resolution_clock::time_point prev_sample_instant;
};