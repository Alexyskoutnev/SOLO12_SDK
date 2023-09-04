#include "commander.hpp"
#include "commander/config.hpp"
#include "master_board_sdk/defines.h"
#include "master_board_sdk/master_board_interface.h"
#include <atomic>
#include <cxxopts.hpp>
#include <thread>

using commander::Commander;

const std::string if_name_default = "enx606d3cd504bf";
const std::string ref_traj_fname_default = "gait.csv";
const char quit_key = 'q';

int
main(int argc, char **argv)
{
	/* parse arguments */
	cxxopts::Options options("main",
	                         "A wrapper for the Solo12 robot masterboard interface.");

	// clang-format off
	options.add_options()
		("d,debug", "Enable debugging")
		("i,if-name", "Masterboard interface name", cxxopts::value<std::string>()->default_value(if_name_default))
		("t,traj-fname", "Trajectory filename", cxxopts::value<std::string>()->default_value(ref_traj_fname_default))
		("c,calibrate", "Hard calibration")
		("p,no-onboard-pd", "Enable current control")
		("o,offset-index", "Set index offset in integer increments of one full rotation", cxxopts::value<std::vector<int>>()->default_value("0,0,0,0,0,0,0,0,0,0,0,0"))
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
			} else {
				is_changing_state = true;
			}
		}
	});

	/* initialize commander */
	Commander com(if_name, traj_fname);

	if (result["debug"].as<bool>()) {
		printf("Debug mode, skipping masterboard initialization.\n");
	} else {
		com.initialize_masterboard();
	}

	if (result["calibrate"].as<bool>()) {
		com.enable_hard_calibration();
	}

	if (result["no-onboard-pd"].as<bool>()) {
		com.disable_onboard_pd();
	}

	if (result.count("offset-index")) {
		auto offset_vec = result["offset-index"].as<std::vector<int>>();
		if (offset_vec.size() != commander::motor_count) {
			printf("%66s\n%66s","Invalid offset vector size! Hint: motor: 0,1,2,3,4,5,6,7,8,9,10,11", "$ ./main -o 0,0,0,0,0,0,0,0,0,0,0,0 \n");
			exit(1);
		}
		int offset[commander::motor_count];

		for (size_t i = 0; i < commander::motor_count; ++i) {
			offset[i] = offset_vec[i];
		}

		com.set_int_index_offset(offset);
	}

	/* main loop */
	com.loop(is_running, is_changing_state);

	/* cleanup */
	thread.join();

	return 0;
}
