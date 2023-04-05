#include "commander.hpp"
#include "rt_timer.hpp"

using commander::Commander;
using commander::State;

int
main(int, char *[])
{

#ifndef DRY_BUILD
	/** not yet supported by rt_timer */
	/** give the process a high priority */
	nice(-20);
#else
	rt_timer::set_process_priority();
#endif
	State state = State::standby;
	Commander com;

	rt_timer::Timer init_timer(commander::send_init_period, com, &Commander::send_init);
	rt_timer::Timer hold_timer(commander::hold_period, com, &Commander::hold);
	rt_timer::Timer track_timer(commander::track_period, com, &Commander::track);
	ClInfo clinfo(state, com, init_timer, hold_timer, track_timer);
	rt_timer::Timer clinfo_timer(commander::clinfo_period, clinfo, &ClInfo::print);

	rt_timer::TimerThread init_thread(init_timer);
	rt_timer::TimerThread hold_thread(hold_timer);
	rt_timer::TimerThread track_thread(track_timer);
	rt_timer::TimerThread cli_thread(clinfo_timer);

	cli_thread.start();

	while (true) {

		switch (state) {
		case State::standby:
			hold_thread.stop();
			track_thread.stop();
			break;
		case State::hold:
			track_thread.stop();
			hold_timer.reset();
			hold_thread.start();
			break;
		case State::track:
			hold_thread.stop();
			track_timer.reset();
			track_thread.start();
			break;
		}

		/** change state on key press */
		char in = std::getchar();

		switch (state) {
		case State::standby:
			com.initialize();
			printf("Sending init...\n");
			init_thread.run_for(std::chrono::seconds(commander::masterboard_timeout));
			printf("Done!");
			state = State::hold;
			break;
		case State::hold:
			state = State::track;
			break;
		case State::track:
			state = State::standby;
			com.log();
			break;
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