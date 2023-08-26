/* Copyright 2023, Cinar, A. L.
 * SPDX-License-Identifier: MIT
 */

#ifndef COMMANDER_HPP_CINARAL_230403_1507
#define COMMANDER_HPP_CINARAL_230403_1507

#include "config.hpp"
#include "matrix_rw.hpp"
#include "timing_stats.hpp"
#include "types.hpp"
#include <atomic>
#include <fstream>
#include <iostream>
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

template <typename T>
int
get_sign(T val)
{
	return (T(0) < val) - (val < T(0));
}

class Commander
{
  public:
	Commander(const std::string mb_if_name, const std::string ref_traj_fname);

	~Commander();

  public:
	void loop(std::atomic_bool &is_running, std::atomic_bool &is_changing_state);
	void print_info();
	void command();
	void change_to_next_state();
	void update_stats();
	void initialize_masterboard();

  private:
	TimingStats command_timing_stats;
	TimingStats print_timing_stats;

	bool is_masterboard_ready = false;
	bool is_ready = false;

	void print_state();
	void print_traj();
	void print_offset();
	void print_stats();
	// void print_timing_stats();
	void log_traj();
	bool check_ready();
	void track(double (&pos_ref)[motor_count]);
	void track(double (&pos_ref)[motor_count], double (&vel_ref)[motor_count]);
	void track(double (&pos_ref)[motor_count], double (&vel_ref)[motor_count],
	           double (&toq_red)[motor_count]);
	void initialize_csv_file_track_error();
	void track_error(double (&pos_ref)[motor_count], double (&vel_ref)[motor_count]);
	void set_offset(double (&index_offset)[motor_count]);
	void track_traj();
	void sweep_traj();
	void sample_traj();

	void reset();

	matrix_rw::Reader<traj_dim> ref_traj_reader;
	matrix_rw::Writer<traj_dim> writematrix;

	double index_pos[motor_count];
	double motor_pos[motor_count];
	double index_offset[motor_count] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
	                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
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
	double toq_ref[motor_count];
	double pos[motor_count];
	double vel[motor_count];
	bool was_offset_enabled = false;
	bool sweep_done = false;
	bool hard_calibrating = false;
	bool loop_track_traj = true;
	bool torque_control_flag = false;
	bool PD_control_flag = false;

	State state = sweep;

	/* Stats Vars */

	double max_amp_stat = 0;
	double max_command_exc_stat = 0;
	double max_print_exc_stat = 0;

	bool hip_offset_flag = true;
	double hip_offset_position[motor_count] = {0.15, -0.15, 0.0, 0.0, 0.0, 0.0,
	                                           0.15, -0.15, 0.0, 0.0, 0.0, 0.0};

	// std::ofstream track_realized_control_io(track_realized_control_data, std::ios_base::app);
	std::ofstream track_realized_control_io;
};

} // namespace commander
#endif