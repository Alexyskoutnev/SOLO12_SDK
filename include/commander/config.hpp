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

#ifndef CONFIG_HPP_CINARAL_230403_1528
#define CONFIG_HPP_CINARAL_230403_1528

#include "types.hpp"
#include <map>
#include <string>

namespace commander
{
#ifndef M_PI
	#define M_PI 3.14159265358979323846
#endif

constexpr double clinfo_freq = 1;      /** [hz] */
constexpr double send_init_freq = 1e1; /** [hz] */
constexpr double hold_freq = 1e2;      /** [hz] */
constexpr double track_freq = 1e3;     /** [hz] */

constexpr double clinfo_period = 1. / clinfo_freq;       /** [s] */
constexpr double send_init_period = 1. / send_init_freq; /** [s] */
constexpr double hold_period = 1. / hold_freq;           /** [s] */
constexpr double track_period = 1. / track_freq;         /** [s] */

constexpr double idx_sweep_freq = .1;       /** [hz] */
constexpr double idx_sweep_ampl = M_PI / 8; /** [rad] */

constexpr Size t_dim_expected = 1e5;
constexpr Size traj_dim = 36;
constexpr Size init_duration = 1;       /** [s] */
constexpr Size masterboard_timeout = 5; /** [s] */
const std::string ref_traj_fprefix = "../../data/active/";
const std::string ref_traj_fname_default = "gait.csv";
const std::string fprefix = "../data/";
const std::string traj_fname = "traj.csv";

const std::string mb_hostname_default = "enx606d3cd504bf";
constexpr double kp_default = 5.;
constexpr double kd_default = 1.;
constexpr double max_current = 4.; /** [A] */
constexpr Size driver_count = 6;
constexpr Size motor_count = 2 * driver_count;
constexpr Size velocity_shift = 12;
constexpr Size torque_shift = 24;
constexpr Size log_size = 1e5;
static std::map<Size, Size> motor2ref_idx = {{0, 0}, {1, 3}, {2, 4}, {3, 1}, {4, 2},   {5, 5},
                                             {6, 6}, {7, 8}, {8, 9}, {9, 7}, {10, 11}, {11, 10}};
static std::map<Size, Size> ref2motor_idx = {{0, 0}, {3, 1}, {4, 2}, {1, 3}, {2, 4},   {5, 5},
                                             {6, 6}, {8, 7}, {9, 8}, {7, 9}, {11, 10}, {10, 11}};
constexpr double gear_ratio[] = {9., 9., 9., -9., -9., 9., 9., -9., -9., -9., 9., 9.};
constexpr double index_offset[] = {3.607416e-01,  1.843653e-01,  -2.675598e-01, 6.811692e-01,
                                   -1.964900e-01, 9.449629e-01,  -9.606218e-02, 1.045705e+00,
                                   1.757172e+00,  -1.222252e+00, -6.761350e-03, -5.420240e-01};
} // namespace commander
#endif