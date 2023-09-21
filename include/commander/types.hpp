/* Copyright 2023, Cinar, A. L., Skoutnev, A.
 * SPDX-License-Identifier: MIT
 */

#ifndef TYPES_HPP_CINARAL_230403_1522
#define TYPES_HPP_CINARAL_230403_1522

#include <array>
#include <cstddef>

namespace commander
{
using std::size_t;
template <size_t M_COL> using Row = std::array<double, M_COL>;
} // namespace commander

#endif