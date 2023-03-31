#include "traj_track/clinfo.hpp"


ClInfo::ClInfo(Interface &interface, rt_timer::Timer<Interface> &interface_timer, size_t t_dim, size_t x_dim,
               double ref_traj[])
    : interface(interface), interface_timer(interface_timer), t_dim(t_dim), x_dim(x_dim),
      ref_traj(ref_traj)
{
}

void
ClInfo::print()
{
	printf("\33[H\33[2J"); //* clear screen

	interface.print();

	/** sample the timer */
	interface_timer.sample(timer_time, call_lag_max, act_elapsed_max, call_count, rt_viol_count, rate_avg,
	             call_lag_avg, act_elapsed_avg);

	if (never_sampled) {
		never_sampled = false;
		start_time = steady_clock::now();
	}
	real_time = duration<double>(steady_clock::now() - start_time).count();

	// clang-format off
	printf("| %-16s | %-16s | %-16s |\n", "Real Time:", "Timer Time:", "Avg. Rate");
	printf("| %14.4g s | %14.4g s | %13.4g Hz |\n",
		real_time, timer_time, rate_avg);
	printf("| %-16s | %-16s | %-16s |\n", "RT Violations:", "Max. Call Lag:", "Avg. Call Lag:");
	printf("| %16zu | %13.4g ms | %13.4g ms |\n",
			rt_viol_count, std::milli::den * call_lag_max, std::milli::den * call_lag_avg);
	printf("| %-16s | %-16s | %-16s |\n", "Violation Rate", "Max. Elapsed:", "Avg. Elapsed:");
	printf("| %15.4g%% | %13.4g ms | %13.4g ms |\n",
		static_cast<double>(rt_viol_count) / call_count * 100, std::milli::den * act_elapsed_max, std::milli::den * act_elapsed_avg);
	// clang-format on

	const int step = interface.get_step_count();
	for (size_t i = 0; i < 12; ++i) {
		if (step < 10000) {
			printf("%6.3g ", ref_traj[interface.get_step_count() * 36 + i]);
		}
	}
	printf("\n");
	fflush(stdout);
}
