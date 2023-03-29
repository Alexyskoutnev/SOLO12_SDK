#include "traj_track/clinfo.hpp"

ClInfo::ClInfo(Interface &interface, Timer<Interface> &interface_timer, size_t t_dim, size_t x_dim,
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

	printf("| %-16s | %-16s | %-16s | %-16s |\n| %16zu | %16.3g | %16.3g | %16zu |\n",
	       "Rate [hz]:", "Avg [us]:", "Max [us]:", "Overtime [#]:",
	       interface_timer.get_action_rate(), interface_timer.get_avg_action_duration(),
	       interface_timer.get_max_action_duration(), interface_timer.get_overtime_count());

	const int step = interface.get_step_count();
	for (size_t i = 0; i < 12; ++i) {
		if (step < 10000) {
			printf("%6.3g ", ref_traj[interface.get_step_count() * 36 + i]);
		}
	}
	printf("\n");
}
