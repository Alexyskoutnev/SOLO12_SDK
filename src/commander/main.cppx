
#include "matrix_rw.hpp"
#include "rt_timer.hpp"
#include "traj_track/clinfo.hpp"
#include "traj_track/interface.hpp"
#include <cstdio>
#include <filesystem>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

//* verify what it should look like in the sim
//* velocities are not used, should be
//* get the data to file
//? fixed dimensions may be too restrictive
constexpr size_t t_dim = 10000;
constexpr size_t x_dim = 36;

constexpr double update_rate = 1000;               //* [Hz]
constexpr double update_period = 1. / update_rate; //* [Hz]
constexpr double clinfo_rate = 1;                  //* [Hz]
constexpr double clinfo_period = 1. / clinfo_rate; //* [Hz]

const std::string data_dir = "../../data";
const std::string data_prefix = data_dir + "/";
const std::string data_fname = "gait.csv";
char IP_NAME[16] = {'e', 'n', 'x', '6', '0', '6', 'd', '3',
                    'c', 'd', '5', '0', '4', 'b', 'f', '\n'};

int
main(int argc, char *argv[])
{
	//* give the process a high priority
	nice(-20);
	// todo change working directory to the executable's directory
	// auto path = std::filesystem::current_path();
	// std::filesystem::current_path(path);
	// printf("%s\n", path);

	double ref_traj[t_dim * x_dim];
	matrix_rw::read<t_dim, x_dim>(data_prefix + data_fname, ref_traj);

	/** create a timer thread to call the interface periodically */
	Interface interface(argv[1], t_dim, x_dim, ref_traj);
	rt_timer::Timer<Interface> interface_timer(update_period, interface, &Interface::update);
	rt_timer::TimerThread<Interface> interface_thread(interface_timer);
	interface_thread.start();

	/** create a second timer thread to sample the interface timer periodically */
	ClInfo clinfo(interface, interface_timer, t_dim, x_dim, ref_traj);
	rt_timer::Timer<ClInfo> clinfo_timer(clinfo_period, clinfo, &ClInfo::print);
	rt_timer::TimerThread<ClInfo> clinfo_thread(clinfo_timer);
	clinfo_thread.start();

	/** wait for key press to stop the timer threads */
	printf("Press any key to execute the trajectory..."); 
	std::getchar();
	interface.execute();

	/** wait for key press to stop the timer threads */
	printf("Press any key to stop..."); 
	std::getchar();
	interface_thread.stop();
	clinfo_thread.stop();

	return 0;
}
