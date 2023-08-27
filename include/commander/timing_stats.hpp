/* Copyright 2023, Cinar, A. L.
 * SPDX-License-Identifier: MIT
 */

#include <chrono>

class TimingStats
{
  public:
	TimingStats();

	void reset();
	void update(double const margin, double const elapsed);
	void print();

	size_t violation_count;
	size_t skip_count;

  private:
	void sampling_check();
	void reset_accum();

	size_t run_count;
	double avg_rate;
	double avg_margin;
	double avg_elapsed;
	double min_margin;
	double max_elapsed;
	
	size_t run_count_accum;
	double margin_accum;
	double elapsed_accum;

	std::chrono::high_resolution_clock::time_point prev_sample_instant;
};