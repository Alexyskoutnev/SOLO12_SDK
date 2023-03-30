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

#include "traj_track/interface.hpp"
#include "traj_track/timer.hpp"

class ClInfo
{
  public:
	ClInfo(Interface &interface, Timer<Interface> &interface_timer, size_t t_dim, size_t x_dim, double ref_traj[]);

	void print();

  private:
	static constexpr size_t clinfo_length = 2;
	Timer<Interface> &interface_timer;
	Interface &interface;

	const size_t t_dim;
	const size_t x_dim;
	double *ref_traj;
};

#endif