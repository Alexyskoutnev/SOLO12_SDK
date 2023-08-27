#include "commander.hpp"
#include "master_board_sdk/defines.h"
#include "master_board_sdk/master_board_interface.h"
#include <atomic>
#include <cxxopts.hpp>
#include <thread>

using commander::Commander;

const std::string if_name_default = "enx606d3cd504bf";
const std::string ref_traj_fname_default = "gait.csv";
const char quit_key = 'q';
const char reset_key = 'r';

int
main(int argc, char **argv)
{
	/* parse arguments */
	cxxopts::Options options("SOLO12_SDK",
	                         "A wrapper for the Solo12 robot masterboard interface.");

	// clang-format off
	options.add_options()
		("d,debug", "Enable debugging", cxxopts::value<bool>()->default_value("false"))
		("i,if-name", "Masterboard interface name", cxxopts::value<std::string>()->default_value(if_name_default))
		("t,traj-fname", "Trajectory filename", cxxopts::value<std::string>()->default_value(ref_traj_fname_default))
		("h,help", "Print usage")
	;
	// clang-format on

	auto result = options.parse(argc, argv);

	if (result.count("help")) {
		std::cout << options.help() << std::endl;
		exit(0);
	}

	auto const if_name = result["if-name"].as<std::string>();
	auto const traj_fname = result["traj-fname"].as<std::string>();

	/* initialize commander */
	Commander com(if_name, traj_fname);

	if (result["debug"].as<bool>()) {
		printf("Debug mode, skipping masterboard initialization.\n");
	} else {
		com.initialize_masterboard();
	}

	/* give the process a high priority */
	nice(-20);

	/* capture inputs in a second thread */
	std::atomic_bool is_running = true;
	std::atomic_bool is_changing_state = false;

	auto thread = std::thread([&] {
		while (is_running) {
			char in = std::getchar();

			if (in == quit_key) {
				is_running = false;
			} else if (in == reset_key) {
				com.reset();
			} else {
				is_changing_state = true;
			}
		}
	});

	/* main loop */
	com.loop(is_running, is_changing_state);

	/* cleanup */
	thread.join();

	return 0;
}
