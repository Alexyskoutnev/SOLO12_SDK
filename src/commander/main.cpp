#include "commander.hpp"
#include <atomic>
#include <thread>

using commander::Commander;
using commander::State;

int
main(int argc, char const *argv[])
{
#ifndef DRY_BUILD
	/** give the process a high priority */
	nice(-20);
#else
	rt_timer::set_process_priority();
#endif

	Commander com;
	// TimerStats stats;

	std::atomic_bool is_running = true;
	std::atomic_bool is_changing_state = false;

	/* capture inputs in another thread */
	auto thread = std::thread([&] {
		while (is_running) {
			char in = std::getchar();

			if (in == 'q') {
				is_running = false;
			} else {
				is_changing_state = true;
			}
		}
	});

	/* main loop */
	std::chrono::high_resolution_clock::time_point now_time;

	auto command_time = std::chrono::high_resolution_clock::now();
	auto print_time = std::chrono::high_resolution_clock::now();

	while (is_running) {
		now_time = std::chrono::high_resolution_clock::now();

		if (now_time >= command_time) {
			com.command();
			// when the command finished
			auto post_time = std::chrono::high_resolution_clock::now();

			// when the command should have been finished by
			auto deadline = now_time +
			    std::chrono::nanoseconds(static_cast<size_t>(
				std::nano::den * commander::command_period));

			double margin;
			if (post_time < deadline) {
				// we are good
				margin =
				    std::chrono::duration<double>(deadline - post_time).count();
			} else {
				// we are late
				margin =
				    -std::chrono::duration<double>(post_time - deadline).count();
			}
			if (margin < com.timing_stats.min_margin) {
				com.timing_stats.min_margin = margin;
			}

			double elapsed =
			    std::chrono::duration<double>(post_time - now_time).count();
			if (elapsed > com.timing_stats.max_elapsed) {
				com.timing_stats.max_elapsed = elapsed;
			}

			com.timing_stats.run_count++;
			com.timing_stats.run_count_accum++;
			com.timing_stats.margin_accum += margin;
			com.timing_stats.elapsed_accum += elapsed;

			com.update_stats(); // Probably not the best place to put this [but needs to
			                    // update relatively fast]
			command_time = deadline; // the next command time is the current deadline.
		}

		if (now_time >= print_time) {
			printf("\33[H\33[2J"); //* clear screen

			auto start = std::chrono::high_resolution_clock::now();

			com.print_all();
			auto end = std::chrono::high_resolution_clock::now();
			com.print_time_dur =
			    std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

			// skip if needed
			while (print_time < now_time) {
				print_time += std::chrono::nanoseconds(
				    static_cast<size_t>(std::nano::den * commander::print_period));
			}
		}

		if (is_changing_state) {
			is_changing_state = false;
			com.next_state();
		}
	}

	thread.join();

	return 0;
}
