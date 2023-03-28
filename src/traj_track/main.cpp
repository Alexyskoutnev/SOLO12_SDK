
#include "traj_track/clinfo.hpp"
#include "traj_track/interface.hpp"
#include "traj_track/timer.hpp"
#include <cstdio>
#include <sys/stat.h>
#include <unistd.h>

//#include "master_board_sdk/defines.h"

constexpr double update_rate = 1000;               //* [Hz]
constexpr double update_period = 1. / update_rate; //* [Hz]
constexpr double clinfo_rate = 1;                  //* [Hz]
constexpr double clinfo_period = 1. / clinfo_rate; //* [Hz]

int
main(int, char *argv[])
{
	nice(-20); // give the process a high priority

	Interface interface(argv[1]);
	Timer<Interface> interface_timer(update_period);

	ClInfo clinfo(interface_timer);
	Timer<ClInfo> clinfo_timer(clinfo_period);

	while (true) {
		interface_timer.check(interface, &Interface::update);
		clinfo_timer.check(clinfo, &ClInfo::print);
	}

	return 0;
}
