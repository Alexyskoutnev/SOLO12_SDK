/* Copyright 2023, Cinar, A. L.
 * SPDX-License-Identifier: MIT
 */

#ifndef COMMANDER_HPP_CINARAL_230403_1507
#define COMMANDER_HPP_CINARAL_230403_1507

#include "config.hpp"
#include "master_board_sdk/defines.h"
#include "master_board_sdk/master_board_interface.h"
#include "matrix_rw.hpp"
#include "timing_stats.hpp"
#include "types.hpp"
#include <atomic>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

namespace commander
{
enum State { not_ready, hold, sweep, track };

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
	void initialize_masterboard();
	void loop(std::atomic_bool &is_running, std::atomic_bool &is_changing_state);
	void enable_hard_calibration();
	void disable_onboard_pd();
	void set_int_index_offset(int (&offset)[motor_count]);

  private:
	void reset();
	void change_to_next_state();

	/* commanding functions */
	void command();
	bool command_check_ready();
	void command_reference(double (&pos_ref)[motor_count], double (&vel_ref)[motor_count]);
	void command_current(double (&pos_ref)[motor_count], double (&vel_ref)[motor_count]);

	/* read reference */
	void get_zero_reference(double (&pos_ref)[motor_count], double (&vel_ref)[motor_count]);
	void get_hold_reference(double (&pos_ref)[motor_count], double (&vel_ref)[motor_count]);
	void get_reference(const size_t t_index, double (&pos_ref)[motor_count],
	                   double (&vel_ref)[motor_count]);

	/* command generation function */
	void generate_step_single_motor_command(const size_t motor_idx, const double amplitude);
	void generate_track_command();
	void generate_sweep_command();

	/* utility functions */
	void sample_traj();
	void update_stats();
	void saturate_reference(double (&pos_ref)[motor_count]);
	void log_traj();
	void set_offset(double (&index_offset)[motor_count]);

	/* printing functions */
	void print_info();
	void print_state();
	void print_stats();
	void print_offset();
	void print_traj();
	void print_masterboard();

	MasterBoardInterface mb;
	TimingStats command_timing_stats;
	TimingStats print_timing_stats;

	bool is_hard_calibrating = false;
	bool using_masterboard_pd = true;

	State state = not_ready;

	matrix_rw::Reader<traj_dim> ref_traj_reader;
	matrix_rw::Writer<traj_dim> writematrix;

	double index_pos[motor_count];
	double motor_pos[motor_count];
	bool was_index_detected[motor_count] = {false, false, false, false, false, false,
	                                        false, false, false, false, false, false};
	int integer_offset[motor_count] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	size_t t_size;
	size_t t_index = 0;
	size_t t_sweep_index = 0;
	size_t t_step_index = 0;
	const size_t max_t_step_index = 2e3;
	std::string ref_traj_fname;

	double kp = commander::kp_default;
	double kd = commander::kd_default;

	std::vector<Row<traj_dim>> traj;
	std::vector<Row<traj_dim>> ref_traj;

	double pos_ref[motor_count];
	double vel_ref[motor_count];
	double pos[motor_count];
	double vel[motor_count];

	bool is_masterboard_connected = false;
	bool is_masterboard_ready = false;
	bool is_sweep_done = false;
	bool all_index_detected = false;
	bool traj_is_sampled = false;

	/* Stats Vars */
	double max_amp_stat = 0;
};

} // namespace commander
#endif