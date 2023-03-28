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

#ifndef TIMER_HPP_CINARAL_230328_1243
#define TIMER_HPP_CINARAL_230328_1243

#include <chrono>

typedef std::chrono::duration<size_t, std::nano> Duration_T;

using std::chrono::duration;
using std::chrono::high_resolution_clock;
using std::chrono::nanoseconds;
using std::chrono::seconds;
using std::chrono::steady_clock;

using hi_res_time_point_T = std::chrono::high_resolution_clock::time_point;
using hi_res_duration_T = std::chrono::high_resolution_clock::duration;
using steady_time_point_T = std::chrono::steady_clock::time_point;
using steady_duration_T = std::chrono::steady_clock::duration;

template <typename Action_T> using ActionFun_T = void (Action_T::*)();

template <typename Action_T> class Timer
{
  public:
	Timer(const double action_period)
	    : action_period(nanoseconds(static_cast<size_t>(1e9 * action_period)))
	{
		next_action_time = high_resolution_clock::now();
	};

	void
	check(Action_T &action, ActionFun_T<Action_T> fun)
	{
		now_time = high_resolution_clock::now();

		if (now_time >= next_action_time) {
			(action.*fun)();
			action_duration = high_resolution_clock::now() - now_time;
			total_action_duration += action_duration;
			++action_counter;

			if (action_duration > action_period) {
				++overtime_counter;
			}

			if (action_duration > max_action_duration) {
				max_action_duration = action_duration;
			}

			action_time = next_action_time;
			next_action_time = action_time + action_period;
		}

		if (steady_clock::now() >= next_counter_reset_time) {
			action_rate = action_counter;
			
			if (action_counter > 0) {
				avg_action_duration = total_action_duration / action_counter;
			} else {
				avg_action_duration = nanoseconds(0);
			}
			action_counter = 0;
			total_action_duration = nanoseconds(0);
			reset_action_counter();

			counter_reset_time = next_counter_reset_time;
			next_counter_reset_time = counter_reset_time + seconds(1);
		}
	}

	size_t
	get_action_rate() const
	{
		return action_rate;
	}

	double
	get_max_action_duration() const
	{
		return duration<double>(max_action_duration).count() * std::micro::den;
	}

	double
	get_avg_action_duration() const
	{
		return duration<double>(avg_action_duration).count() * std::micro::den;
	}

	size_t
	get_overtime_count() const
	{
		return overtime_counter;
	}

  private:
	void
	reset_action_counter()
	{
		action_counter = 0;
	}
	size_t period_index = 0;
	size_t action_rate = 0; //* [Hz]
	size_t action_counter = 0;
	size_t overtime_counter = 0;
	hi_res_time_point_T now_time;
	hi_res_time_point_T action_time;
	hi_res_time_point_T next_action_time;
	const hi_res_duration_T action_period;
	hi_res_duration_T action_duration = nanoseconds(0);
	hi_res_duration_T total_action_duration = nanoseconds(0);
	hi_res_duration_T avg_action_duration = nanoseconds(0);
	hi_res_duration_T max_action_duration = nanoseconds(0);
	steady_time_point_T counter_reset_time;
	steady_time_point_T next_counter_reset_time;
};
#endif
