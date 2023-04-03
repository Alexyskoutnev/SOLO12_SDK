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
constexpr size_t t_dim_expected = 1e5;
constexpr size_t traj_dim = 16;
constexpr size_t masterboard_timeout = 5; /** [s] */
const std::string ref_traj_fprefix = "../../data/";
const std::string ref_traj_fname_default = "gait.csv";
constexpr char mb_hostname_default[] = {'e', 'n', 'x', '6', '0', '6', 'd', '3',
                                        'c', 'd', '5', '0', '4', 'b', 'f'};
constexpr double kp_default = 5.;
constexpr double kd_default = 1.;
constexpr double max_current = 1.;
constexpr size_t driver_count = 6;
constexpr size_t motor_count = 2 * driver_count;
constexpr size_t velocity_shift = 12;
constexpr size_t torque_shift = 24;
const std::map<size_t, size_t> motor2ref_idx = {{0, 0}, {1, 3}, {2, 4}, {3, 1}, {4, 2},   {5, 5},
                                                {6, 6}, {7, 8}, {8, 9}, {9, 7}, {10, 11}, {11, 10}};
static constexpr double gear_ratio[] = {9., 9., 9., -9., -9., 9., 9., -9., -9., -9., 9., 9.};
} // namespace commander
#endif