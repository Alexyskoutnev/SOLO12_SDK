#include "commander/clinfo.hpp"

using std::chrono::duration;

ClInfo::ClInfo(State &state, Commander &com, rt_timer::Timer<Commander> &init_timer,
               rt_timer::Timer<Commander> &hold_timer, rt_timer::Timer<Commander> &track_timer)
    : state(state), com(com), init_timer(init_timer), hold_timer(hold_timer),
      track_timer(track_timer)
{
#ifdef WIN32
	/** enable VT100 for win32*/
	DWORD l_mode;
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	GetConsoleMode(hStdout, &l_mode);
	SetConsoleMode(hStdout,
	               l_mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING | DISABLE_NEWLINE_AUTO_RETURN);
#endif
}

void
ClInfo::print()
{
	printf("\33[H\33[2J"); //* clear screen
	printf("Enter to cycle through states, enter 'q' to quit.\n");

	printf("Status: ");
	switch (state) {
	case State::standby:
		printf("Standby\n");
		break;
	case State::hold:
		printf("Hold\n");
		break;
	case State::track:
		printf("Track\n");
		break;
	}

	if (never_printed) {
		never_printed = false;
		start_time = steady_clock::now();
	}
	real_time = duration<double>(steady_clock::now() - start_time).count();
	printf("| %-16s |\n", "Real Time:");
	printf("| %14.4g s |\n", real_time);

	/** debug only */
	// printf("Init timer:\n");
	// ClInfo::print_timer_stats(init_timer);

	printf("Hold timer:\n");
	ClInfo::print_timer_stats(hold_timer);

	printf("Track timer:\n");
	ClInfo::print_timer_stats(track_timer);

	com.print();
}

void
ClInfo::print_timer_stats(rt_timer::Timer<Commander> &timer)
{
	/** sample the timer */
	double timer_time;
	double call_lag_max;
	double act_elapsed_max;
	Size call_count;
	Size rt_viol_count;
	double rate_avg;
	double call_lag_avg;
	double act_elapsed_avg;

	timer.sample(timer_time, call_lag_max, act_elapsed_max, call_count, rt_viol_count, rate_avg,
	             call_lag_avg, act_elapsed_avg);

	// clang-format off
	printf("| %-16s | %-16s | %-16s | %-16s |\n",  
	"Timer Time:", "RT Violations:", "Max. Call Lag:", "Avg. Call Lag:");
	printf("| %14.4g s | %16zu | %13.4g ms | %13.4g ms |\n", 
	timer_time, rt_viol_count, std::milli::den * call_lag_max, std::milli::den * call_lag_avg);
	printf("| %-16s | %-16s | %-16s | %-16s |\n", 
	"Avg. Rate", "Violation Ratio", "Max. Elapsed:", "Avg. Elapsed:");
	printf("| %13.4g Hz | %15.4g%% | %13.4g ms | %13.4g ms |\n", 
	rate_avg, static_cast<double>(rt_viol_count) / call_count * 100, std::milli::den * act_elapsed_max, std::milli::den * act_elapsed_avg);
	// clang-format on
}
