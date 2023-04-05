#include "commander.hpp"
#include "rt_timer.hpp"

using commander::Commander;
using commander::State;

int
main(int argc, char *argv[])
{

#ifndef DRY_BUILD
	/** not yet supported by rt_timer */
	/** give the process a high priority */
	nice(-20);
#else
	rt_timer::set_process_priority();
#endif
	State state = State::hold;

	//if (argc != 2) {
	//	throw std::runtime_error("Please provide the interface name "
	//	                         "(i.e. using 'ifconfig' on linux");
	//}
	Commander com;

	//
	rt_timer::Timer init_timer(commander::send_init_period, com, &Commander::send_init);
	rt_timer::Timer hold_timer(commander::hold_period, com, &Commander::hold);
	rt_timer::Timer track_timer(commander::track_period, com, &Commander::track);

	ClInfo clinfo(state, com, init_timer, hold_timer, track_timer);
	rt_timer::Timer clinfo_timer(commander::clinfo_period, clinfo, &ClInfo::print);

	rt_timer::TimerThread init_thread(init_timer);
	rt_timer::TimerThread hold_thread(hold_timer);
	rt_timer::TimerThread track_thread(track_timer);
	rt_timer::TimerThread cli_thread(clinfo_timer);

	/** block for the timer send init */
	printf("Sending init...\n");
	//init_thread.run_for(std::chrono::seconds(commander::masterboard_timeout));
	printf("Done!");

	cli_thread.start();

	while (true) {
		switch (state) {
		case State::standby:
			break;
		case State::hold:
			com.initialize();
			hold_timer.reset();
			hold_thread.start();
			break;
		case State::track:
			track_timer.reset();
			track_thread.start();
			break;
		}

		/** on key press */
		char in = std::getchar();

		if (in == 'q') {
			break;
		}
		switch (state) {
		case State::standby:
			state = State::hold;
			break;
		case State::hold:
			state = State::track;
			hold_thread.stop();
			break;
		case State::track:
			state = State::standby;
			track_thread.stop();
			com.log();
			break;
		}
	}
	cli_thread.stop();
	hold_thread.stop();
	track_thread.stop();
	return 0;
}