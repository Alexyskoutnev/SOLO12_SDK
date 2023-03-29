#include "traj_track/clinfo.hpp"

ClInfo::ClInfo(Interface &interface, Timer<Interface> &interface_timer)
    : interface(interface), interface_timer(interface_timer)
{
}

void
ClInfo::print()
{
	printf("\33[H\33[2J"); //* clear screen

	interface.print();

	printf("| %-16s | %-16s | %-16s | %-16s |\n| %16zu | %16.5g | %16.5g | %16zu |\n",
	       "Rate [hz]:", "Avg [us]:", "Max [us]:", "Overtime [#]:",
	       interface_timer.get_action_rate(), interface_timer.get_avg_action_duration(),
	       interface_timer.get_max_action_duration(), interface_timer.get_overtime_count());
}
