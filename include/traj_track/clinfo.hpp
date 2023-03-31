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

#ifndef CLINFO_HPP_CINARAL_230328_1341
#define CLINFO_HPP_CINARAL_230328_1341

#include "rt_timer.hpp"
#include "traj_track/interface.hpp"

#include <chrono>
using std::chrono::duration;
using std::chrono::steady_clock;
using time_sc = steady_clock::time_point;

class ClInfo
{
  public:
	ClInfo(Interface &interface, rt_timer::Timer<Interface> &interface_timer, size_t t_dim, size_t x_dim,
	       double ref_traj[]);

	void print();

  private:
	static constexpr size_t clinfo_length = 6;
	bool never_sampled = true;
	time_sc start_time;
	double real_time;
	double timer_time;
	double call_lag_max;
	double act_elapsed_max;
	size_t call_count;
	size_t rt_viol_count;
	double rate_avg;
	double call_lag_avg;
	double act_elapsed_avg;
	Interface &interface;
	rt_timer::Timer<Interface> &interface_timer;
	const size_t t_dim;
	const size_t x_dim;
	double *ref_traj;
};

#endif