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
			auto start = std::chrono::high_resolution_clock::now();
			com.command();
			auto end = std::chrono::high_resolution_clock::now();
			com.command_time_dur = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
			com.stats();  //Probably not the best place to put this [but needs to update relatively fast]
			command_time += std::chrono::nanoseconds(
			    static_cast<size_t>(std::nano::den * commander::command_period));
		}

		if (now_time >= print_time) {
			printf("\33[H\33[2J"); //* clear screen
			auto start = std::chrono::high_resolution_clock::now();
			com.print_all();
			auto end = std::chrono::high_resolution_clock::now();
			com.print_time_dur = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
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
