#include "traj_track/clinfo.hpp"

ClInfo::ClInfo(Timer<Interface> &interface_timer) : interface_timer(interface_timer)
{
	for (size_t i = 0; i < clinfo_length; ++i) {
		printf("\n");
	}
}

void
ClInfo::print()
{
	for (size_t i = 0; i < clinfo_length; ++i) {
		printf("\033[A\033[2K\r"); //* move the cursor up then clear the line
	}

	printf("| %-16s | %-16s | %-16s | %-16s |\n| %16zu | %16.5g | %16.5g | %16zu |\n", "Rate [hz]:",
	       "Avg [us]:", "Max [us]:", "Overtime [#]:", interface_timer.get_action_rate(),
	       interface_timer.get_avg_action_duration(), interface_timer.get_max_action_duration(),
	       interface_timer.get_overtime_count());

	//printf("\33[H\33[2J"); // clear screen
	//robot_if.PrintIMU();
	//robot_if.PrintADC();
	//robot_if.PrintMotors();
	//robot_if.PrintMotorDrivers();
	//robot_if.PrintStats();
	//fflush(stdout);
}
