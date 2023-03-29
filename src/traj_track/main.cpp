
#include "matrix_rw.hpp"
#include "traj_track/clinfo.hpp"
#include "traj_track/interface.hpp"
#include "traj_track/timer.hpp"
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

constexpr double update_rate = 1000;                //* [Hz]
constexpr double update_period = 1. / update_rate; //* [Hz]
constexpr double clinfo_rate = 1;                  //* [Hz]
constexpr double clinfo_period = 1. / clinfo_rate; //* [Hz]

const std::string data_dir = "../../data";
const std::string data_prefix = data_dir + "/";
const std::string data_fname = "gait.csv";

int
main(int, char *argv[])
{
	//* give the process a high priority
	nice(-20);
	// todo change working directory to the executable's directory
	// auto path = std::filesystem::current_path();
	// std::filesystem::current_path(path);
	// printf("%s\n", path);

	double ref_traj[t_dim * x_dim];
	matrix_rw::read<t_dim, x_dim>(data_prefix + data_fname, ref_traj);

	Interface interface(argv[1], t_dim, x_dim, ref_traj);
	Timer<Interface> interface_timer(update_period);

	ClInfo clinfo(interface, interface_timer, t_dim, x_dim, ref_traj);
	Timer<ClInfo> clinfo_timer(clinfo_period);

	while (true) {
		interface_timer.check(interface, &Interface::update);
		clinfo_timer.check(clinfo, &ClInfo::print);
	}

	return 0;
}
