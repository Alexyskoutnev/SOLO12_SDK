#include "commander/timing_stats.hpp"

TimingStats::TimingStats()
{
	prev_sample_instant = std::chrono::high_resolution_clock::now();
	min_margin = std::numeric_limits<double>::max();
	max_elapsed = 0;
	reset_accum();
}

void
TimingStats::sampling_check()
{
	auto now_time = std::chrono::high_resolution_clock::now();

	if (now_time > prev_sample_instant && run_count_accum > 0) {
		avg_rate = static_cast<double>(run_count_accum) /
		    std::chrono::duration<double>(now_time - prev_sample_instant).count();
		avg_elapsed = elapsed_accum / run_count_accum;
		avg_margin = margin_accum / run_count_accum;
		reset_accum();
		prev_sample_instant = now_time;
	}
}

void
TimingStats::reset_accum()
{
	run_count_accum = 0;
	margin_accum = 0;
	elapsed_accum = 0;
}