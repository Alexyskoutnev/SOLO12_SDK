/* Copyright 2022, Cinar, A. L.
 * SPDX-License-Identifier: MIT
 */

#ifndef WRITE_HPP_CINARAL_220924_0019
#define WRITE_HPP_CINARAL_220924_0019

#include "types.hpp"
#include <fstream>
#include <iomanip> /** std::setprecision */
#include <limits>  /** std::numeric_limits */
#include <string>
#include <vector>

namespace matrix_rw
{
template <Size M_COL> class Writer
{
  public:
	Writer(std::string_view delimiter = ",") : delimiter(delimiter){};
	~Writer(){};

	void
	operator()(const std::string &file_name, std::vector<Row<M_COL>> &matrix)
	{
		std::ofstream file;
		file.open(file_name);

		if (!file.is_open()) {
			printf("Could not open file: %s\n", file_name.c_str());
			return;
		}
		const Size n_row = matrix.size();
		/** set write precision */
		file << std::setprecision(std::numeric_limits<Real>::digits10 + 1);

		for (Size i = 0; i < n_row; i++) {
			for (Size j = 0; j < M_COL; j++) {
				file << std::scientific << matrix[i][j];

				if (j < M_COL - 1) {
					file << delimiter;
				}
			}

			/** add newline except for the last line */
			if (i < n_row) {
				file << std::endl;
			}
		}
		file.close();
	}

  private:
	const std::string_view delimiter;
};
} // namespace matrix_rw

#endif