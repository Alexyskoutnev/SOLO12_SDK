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

// #include "master_board_sdk/master_board_interface.h"
#include "config.hpp"
#include "matrix_rw.hpp"
#include "rt_timer.hpp"
#include "types.hpp"
#include <map>
#include <string>

namespace commander
{

class Commander
{
	enum State { standby, hold, track };

  public:
	Commander(const std::string ref_traj_fname = ref_traj_fname_default,
	          const char mb_hostname[] = mb_hostname_default, const double kp = kp_default,
	          const double kd = kd_default);
	~Commander();
	void update();
	// void execute();
	//  bool check_ready();
	//  void print();
	//  size_t get_step_count();

  private:
	matrix_rw::Reader<traj_dim> readmatrix;
	matrix_rw::Reader<traj_dim> writematrix;

	size_t t_dim;
	State state = standby;
	double kp;
	double kd;
	VarRowMat_T<traj_dim> traj;
	VarRowMat_T<traj_dim> ref_traj;

	// MasterBoardInterface mb;

	// double init_pos[motor_count];
	// bool is_ready = false;
	// bool is_executing = false;
};
} // namespace commander
#endif