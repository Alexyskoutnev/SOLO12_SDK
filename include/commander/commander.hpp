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

template <typename T>
int
sgn(T val)
{
	return (T(0) < val) - (val < T(0));
}

class TimingStats
{
  public:
	TimingStats()
	{
		prev_sample_instant = std::chrono::high_resolution_clock::now();
		min_margin = std::numeric_limits<double>::max();
		max_elapsed = 0;
		reset_accum();
	}

	void
	sampling_check()
	{
		auto now_time = std::chrono::high_resolution_clock::now();

		if (now_time > prev_sample_instant && run_count_accum > 0) {
			avg_rate = static_cast<double>(run_count_accum) /  std::chrono::duration<double>(now_time - prev_sample_instant).count();;
			avg_elapsed = elapsed_accum / run_count_accum;
			avg_margin = margin_accum / run_count_accum;
			reset_accum();
			prev_sample_instant = now_time;
		}
	}

	size_t run_count;
	size_t violation_count;
	double avg_rate;
	double avg_margin;
	double avg_elapsed;
	double min_margin;
	double max_elapsed;

	size_t run_count_accum;
	double margin_accum;
	double elapsed_accum;

  private:
	void
	reset_accum()
	{
		run_count_accum = 0;
		margin_accum = 0;
		elapsed_accum = 0;
	}
	std::chrono::high_resolution_clock::time_point prev_sample_instant;
};

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
	void print_timing_stats();
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
	void update_stats();
	void reset();

	/* stat vars */
	TimingStats timing_stats;
	// std::chrono::milliseconds command_time_dur{0};
	std::chrono::milliseconds print_time_dur{0};

  private:
	bool is_ready = false;

	void initialize();
	void initialize_mb();

	matrix_rw::Reader<traj_dim> readmatrix;
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