/*
 * SOLO12_SDK commander
 *
 * MIT License
 *
 * Copyright (c) 2023 Cinar, A. L.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to do
 * so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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
	void initialize();
	void print_all();
	void log_traj();
	bool check_ready();
	void track(double (&pos_ref)[motor_count], double (&vel_ref)[motor_count]);
	void track_traj();
	void sample_traj();
	void command();
	void next_state();

  private:
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