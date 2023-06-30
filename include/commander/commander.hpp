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
enum State { hold, sweep, track };

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

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
	void print_traj();
	void print_offset();
	void print_stats();
	void log_traj();
	bool check_ready();
	void track(double (&pos_ref)[motor_count]);
	void track(double (&pos_ref)[motor_count], double (&vel_ref)[motor_count]);
	void set_offset(double (&index_offset)[motor_count]);
	void track_traj();
	void sweep_traj();
	void sample_traj();
	void command();
	void next_state();
	void stats();
	void reset();

	/* stat vars */
	std::chrono::milliseconds command_time_dur{0};
	std::chrono::milliseconds print_time_dur{0};

  private:
	bool is_ready = false;

	void initialize();
	void initialize_mb();

	matrix_rw::Reader<traj_dim> readmatrix;
	matrix_rw::Writer<traj_dim> writematrix;

	double index_pos[motor_count];
	double motor_pos[motor_count];
	// double index_offset[motor_count] = {4.78476, -3.20884, -2.4988, 
	// 									5.96416, 0.172924, -5.06818, 
	// 									2.2356, -1.53264, -1.50784, 
	// 									-4.11112, 4.21654, -0.00550638};
	// double index_offset[motor_count] = {4.26096, -3.20884, -2.50225, 0.212153, 0.168673, -5.59543, 1.72045, -1.533, -1.51272, -4.63414, 4.21937, 5.8449};
	double index_offset[motor_count] = {0.0, 0.0, 0.0, 
										0.0, 0.0, 0.0, 
										0.0, 0.0, 0.0, 
										0.0, 0.0, 0.0};
	bool was_index_detected[motor_count] = {false, false, false, false, false, false,
											false, false, false, false, false, false};

	size_t t_index;
	size_t t_size;
	size_t t_sweep_index = 0;
	std::string ref_traj_fname;

	MasterBoardInterface mb;
	double kp;
	double kd;
	double init_pos[motor_count];

	std::array<double, 1> imu_logs;
	std::vector<Row<traj_dim>> traj;
	std::vector<Row<traj_dim>> ref_traj;
	std::vector<Row<traj_dim + 1>> logs;

	double pos_ref[motor_count];
	double vel_ref[motor_count];
	double pos[motor_count];
	double vel[motor_count];
	bool was_offset_enabled = false;
	bool sweep_done = false;
	bool hard_calibrating = false;
	State state = sweep;

	/* Stats Vars */
	double max_amp_stat = 0;
	double max_command_exc_stat = 0;
	double max_print_exc_stat = 0;
	

};
} // namespace commander
#endif