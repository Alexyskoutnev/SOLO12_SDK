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
#include <map>

class Interface
{
  public:
	Interface(char *name, size_t t_dim, size_t x_dim, double ref_traj[], double kp = 5.0,
	          double kd = .1, double current_sat = 4.0, size_t timeout = 5);
	~Interface();
	void update();
	bool check_ready();
	void print();
	size_t get_step_count();

  private:
	// size_t get_traj_idx(size_t motor_idx);

	static constexpr size_t driver_count = 6;
	static constexpr size_t motor_count = 2 * driver_count;
	const size_t t_dim;
	const size_t x_dim;
	const double kp;
	const double kd;
	const double current_sat;
	const double timeout;
	std::map<size_t, size_t> ref_idx = {{0, 0}, {1, 3}, {2, 4}, {3, 1}, {4, 2},   {5, 5},
	                                    {6, 6}, {7, 8}, {8, 9}, {9, 7}, {10, 11}, {11, 10}};

	static constexpr double gear_ratio[] = {9., 9., 9., -9., -9., 9., 9., -9., -9., -9., 9., 9.};

	MasterBoardInterface masterboard;
	size_t step_counter = 0;
	double *ref_traj;
	double init_pos[motor_count];
	bool is_ready = false;
};

#endif