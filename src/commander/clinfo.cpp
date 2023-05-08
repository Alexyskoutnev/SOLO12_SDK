#include "commander/clinfo.hpp"

using std::chrono::duration;

ClInfo::ClInfo(State &state, Commander &com) : state(state), com(com)
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
ClInfo::push_timer(rt_timer::Timer<Commander> *timer)
{
	timers.push_back(timer);
}
void
ClInfo::pop_timer()
{
	timers.pop_back();
}
void
ClInfo::push_message(const std::string &message)
{
	messages.push_back(message);
}
void
ClInfo::pop_message()
{
	messages.pop_back();
}

void
ClInfo::print()
{
	printf("\33[H\33[2J"); //* clear screen

	for (auto &message : messages) {
		printf("%s\n", message.c_str());
	}
	if (never_printed) {
		never_printed = false;
		start_time = steady_clock::now();
	}
	real_time = duration<double>(steady_clock::now() - start_time).count();
	printf("| %-16s |\n", "Real Time:");
	printf("| %14.4g s |\n", real_time);

	for (auto &timer : timers) {
		ClInfo::print_timer_stats(*timer);
	}
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
