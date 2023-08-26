#include "commander/timing_stats.hpp"
#include <cstdio>

TimingStats::TimingStats()
{
	reset();
	reset_accum();
	prev_sample_instant = std::chrono::high_resolution_clock::now();
}

void
TimingStats::reset()
{
	run_count = 0;
	violation_count = 0;
	skip_count = 0;
	min_margin = std::numeric_limits<double>::max();
	max_elapsed = 0;
}

void
TimingStats::reset_accum()
{
	run_count_accum = 0;
	margin_accum = 0;
	elapsed_accum = 0;
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
TimingStats::update(double const margin, double const elapsed)
{
	run_count++;
	run_count_accum++;
	if (margin < min_margin) {
		min_margin = margin;
	}
	if (elapsed > max_elapsed) {
		max_elapsed = elapsed;
	}
	margin_accum += margin;
	elapsed_accum += elapsed;
}

void
TimingStats::print()
{
	sampling_check();

	printf("| %11s | %11s | %11s | %11s | %11s |\n", " Rate(Hz)", "Viol. (%)", "Viol. Count",
	       "Skip Count", "Run Count");
	printf("| %11.3g | %11.3g | %11lu | %11lu | %11lu |\n", avg_rate,
	       static_cast<double>(violation_count) / run_count * 100, violation_count, skip_count,
	       run_count);
	printf("| %15s | %15s | %15s | %15s |\n", "Min Margin (ms)", "Avg Margin (ms)", "Max Elap. (ms)",
	       "Avg Elap. (ms)");
	printf("| %15.3g | %15.3g | %15.3g | %15.3g |\n", min_margin * 1e3, 
	       avg_margin * 1e3, max_elapsed * 1e3, avg_elapsed * 1e3);
}
