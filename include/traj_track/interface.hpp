/*
 * SOLO12_SDK
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

#ifndef INTERFACE_HPP_CINARAL_230328_1323
#define INTERFACE_HPP_CINARAL_230328_1323

#include "master_board_sdk/master_board_interface.h"
#include <string>

class Interface
{
  public:
	Interface(char *name, double kp = 5.0, double kd = .1, double current_sat = 4.0,
	          size_t timeout = 5);
	~Interface();

	void update();
	bool check_ready();

  private:
	//? fixed dimensions may be too restrictive
	static constexpr size_t t_dim = 5010;
	static constexpr size_t x_dim = 36;
	static constexpr size_t driver_count = 6;
	static constexpr size_t motor_count = 2 * driver_count;
	const std::string data_dir = "../../data";
	const std::string data_prefix = data_dir + "/";
	const std::string data_fname = "joint_trajectory_jan_23.csv";

	MasterBoardInterface masterboard;
	double (&ref_traj)[t_dim * x_dim] = *(double (*)[t_dim * x_dim]) new double[t_dim * x_dim];
	size_t step_counter = 0;

	const double kp;
	const double kd;
	const double current_sat;
	const double timeout;

	bool is_ready = false;
	double init_pos[motor_count];
};

#endif