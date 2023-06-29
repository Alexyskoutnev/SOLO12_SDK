/* Copyright 2023, Cinar, A. L.
 * SPDX-License-Identifier: MIT
 */

#ifndef COMMANDER_HPP_CINARAL_230403_1507
#define COMMANDER_HPP_CINARAL_230403_1507

#include "config.hpp"
#include "matrix_rw.hpp"
#include "types.hpp"
#include <map>
#include <string>
#include <vector>


#ifndef DRY_BUILD
	#include <sys/stat.h>
	#include <unistd.h>
	#include "master_board_sdk/defines.h"
	#include "master_board_sdk/master_board_interface.h"
#else
	#include "dummy_interface.hpp"
#endif

namespace commander
{
enum State { hold, track };

class Commander
{
  public:
	Commander(const std::string ref_traj_fname = ref_traj_fname_default,
	          const std::string mb_hostname = mb_hostname_default, const double kp = kp_default,
	          const double kd = kd_default);
	~Commander();

  public:
	void print_all();
	void print_state();
	void log_traj();
	bool check_ready();
	void track(double (&pos_ref)[motor_count], double (&vel_ref)[motor_count]);
	void track_traj();
	void sample_traj();
	void command();
	void next_state();

  private:
 	bool is_ready = false;

 	void initialize();
	void initialize_mb();

	matrix_rw::Reader<traj_dim> readmatrix;
	matrix_rw::Writer<traj_dim> writematrix;

	double index_pos[motor_count];
	double motor_pos[motor_count];
	size_t t_index;
	size_t t_size;
	std::string ref_traj_fname;

	MasterBoardInterface mb;
	double kp;
	double kd;
	double init_pos[motor_count];

	std::array<double, 1> imu_logs;
	std::vector<Row<traj_dim>> traj;
	std::vector<Row<traj_dim>> ref_traj;
	std::vector<Row<traj_dim + 1>> logs;

	State state = hold;
};
} // namespace commander
#endif