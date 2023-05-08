#include "commander.hpp"
#include "cxxopts.hpp"
#include "rt_timer.hpp"

using commander::Commander;
using commander::State;

int
main(int argc, char const *argv[])
{
	cxxopts::Options options("rt_timer example.\n");
	options.show_positional_help();
	// clang-format off
	options.add_options()
		("h,help", "Print usage")
		("c,calibration", "Enable calibration", cxxopts::value<bool>()->default_value("false")
	);
	// clang-format on
	auto result = options.parse(argc, argv);

	if (result.count("help")) {
		printf("%s", options.help().c_str());
		return 0;
	}

	const bool is_calibrating = result["calibration"].as<bool>();

#ifndef DRY_BUILD
	/** not yet supported by rt_timer */
	/** give the process a high priority */
	nice(-20);
#else
	rt_timer::set_process_priority();
#endif
	State state = State::standby;
	Commander com;
	ClInfo clinfo(state, com);

	rt_timer::Timer init_timer(commander::send_init_period, com, &Commander::send_init);
	rt_timer::Timer clinfo_timer(commander::clinfo_period, clinfo, &ClInfo::print);
	rt_timer::Timer hold_timer(commander::hold_period, com, &Commander::hold);
	rt_timer::Timer track_timer(commander::track_period, com, &Commander::track);
	rt_timer::Timer calibrate_timer(commander::hold_period, com, &Commander::calibrate);

	rt_timer::TimerThread init_thread(init_timer);
	rt_timer::TimerThread cli_thread(clinfo_timer);
	rt_timer::TimerThread hold_thread(hold_timer);
	rt_timer::TimerThread track_thread(track_timer);
	rt_timer::TimerThread calibrate_thread(calibrate_timer);

	clinfo.push_message("Enter to cycle through states, enter 'q' to quit.");
	if (is_calibrating) {
		clinfo.push_message("Calibration enabled");
	}
	clinfo.push_message("Waiting...");

	cli_thread.start();

	while (true) {
		switch (state) {
		case State::standby: {
			hold_thread.stop();
			track_thread.stop();
			break;
		}
		case State::hold: {
			track_thread.stop();
			hold_timer.reset();
			hold_thread.start();
			break;
		}
		case State::track: {
			hold_thread.stop();
			track_timer.reset();
			track_thread.start();
			break;
		}
		}

		/** change state on key press */
		char in = std::getchar();

		switch (state) {
		case State::standby: {
			com.initialize();
			clinfo.push_message("Sending initialization...");
			init_thread.run_for(std::chrono::seconds(commander::init_duration));
			clinfo.pop_message();
			clinfo.pop_message();
			state = State::hold;

			clinfo.push_message("Holding...");
			clinfo.push_timer(&hold_timer);
			break;
		}

		case State::hold: {
			state = State::track;
			clinfo.pop_message();
			clinfo.pop_timer();
			if (is_calibrating) {
				clinfo.push_message("Calibrating...");
				clinfo.push_timer(&calibrate_timer);
			} else {
				clinfo.push_message("Tracking...");
				clinfo.push_timer(&track_timer);
			}
			break;
		}
		case State::track: {
			state = State::standby;
			clinfo.pop_message();
			clinfo.pop_timer();
			clinfo.push_message("Waiting...");
			com.log();
			break;
		}
		}

		if (in == 'q') {
			break;
		}
	}
	cli_thread.stop();
	hold_thread.stop();
	track_thread.stop();
	return 0;
}