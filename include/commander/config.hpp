/* Copyright 2023, Cinar, A. L.
 * SPDX-License-Identifier: MIT
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

constexpr double command_freq = 1e3; /** [hz] */
constexpr double print_freq = 2;  /** [hz] */
constexpr double input_freq = 1e1; /** [hz] */

constexpr double command_period = 1. / command_freq;   /** [s] */
constexpr double print_period = 1. / print_freq; /** [s] */
constexpr double input_period = 1. / input_freq; /** [s] */

// constexpr double idx_sweep_freq = 0.05;     /** [hz] */
constexpr double idx_sweep_freq = 0.05;     /** [hz] */
constexpr double idx_sweep_ampl = M_PI / 9; /** [rad] */

constexpr size_t t_dim_expected = 1e5;
constexpr size_t traj_dim = 36;
constexpr size_t init_duration = 1;       /** [s] */
constexpr size_t sweep_duration = 5;      /** [s] */
// constexpr size_t masterboard_timeout = 10; /** [ms] */
constexpr size_t masterboard_timeout = 0; /** [ms] */
const std::string ref_traj_fprefix = "../../data/active/";
const std::string ref_traj_fname_default = "gait.csv";
const std::string fprefix = "../data/";
const std::string traj_fname = "traj.csv";

const std::string mb_hostname_default = "enx606d3cd504bf";
constexpr double kp_default = 5.0; // 20, 5;
constexpr double kd_default = 1.0;
constexpr double max_current = 5.0; /** [A] */
constexpr size_t driver_count = 6;
constexpr size_t motor_count = 2 * driver_count;
constexpr size_t velocity_shift = 12;
constexpr size_t torque_shift = 24;
constexpr size_t log_size = 1e5;
// constexpr double ref_hold_position[motor_count] = {0.016,  0.76,  -1.69, -0.016,  0.76,   -1.698,
//                                                    0.0164, -0.76, 1.698, -0.0164, -0.761, 1.698};
constexpr double ref_hold_position[motor_count] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

/** convert traj convention when iterating through motors */
static std::map<size_t, size_t> ref2motor_idx = {{0, 0}, {1, 3}, {2, 4},   {3, 1},
                                                 {4, 2}, {5, 5}, {6, 6},   {7, 8},
                                                 {8, 9}, {9, 7}, {10, 11}, {11, 10}};

/** converting measured to traj convention */
static std::map<size_t, size_t> motor2ref_idx = {{0, 0}, {3, 1}, {4, 2},   {1, 3},
                                                 {2, 4}, {5, 5}, {6, 6},   {8, 7},
                                                 {9, 8}, {7, 9}, {11, 10}, {10, 11}};
constexpr double gear_ratio[motor_count] = {9., -9., -9., -9., 9., 9., 9., -9., -9., -9., 9., 9.};
constexpr double index_offset[motor_count] = {4.78005, 3.07857, -3.02573, 5.96886, -5.59072, 1.21769, 1.71526, 3.18527, 4.76795, 1.12588, 4.22048, 0.087874};

static std::map<int, std::string> state_to_name = {{0, "hold"}, {1, "sweep"}, {2, "track"}};

} // namespace commander



#endif