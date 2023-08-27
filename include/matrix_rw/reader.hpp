/* Copyright 2022, Cinar, A. L.
 * SPDX-License-Identifier: MIT
 */

#ifndef READ_HPP_CINARAL_220924_0017
#define READ_HPP_CINARAL_220924_0017

#include "types.hpp"
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace matrix_rw
{
template <Size M_COL> class Reader
{
  public:
	Reader(std::string_view delimiter = ",") : delimiter(delimiter){};
	~Reader(){};

	void
	operator()(const std::string &file_name, std::vector<Row<M_COL>> &matrix)
	{
		std::ifstream file;
		file.open(file_name);

		if (!file.is_open()) {
			printf("Could not open file: %s\n", file_name.c_str());
			return;
		}
		Row<M_COL> row;

		while (std::getline(file, line)) {
			if (line.empty()) {
				continue;
			}
			parse_row(line, row);
			matrix.push_back(row);
		}
		file.close();
	}

  private:
	void
	parse_row(std::string &line, Row<M_COL> &row)
	{
		/** parse the line by splitting at the delimiters */
		for (Size i = 0; i < M_COL; i++) {
			const Size str_pos = line.find(delimiter);

			if (str_pos != std::string::npos) {
				/** an */
				item = line.substr(0, str_pos);
				line.erase(0, str_pos + delimiter.length());
#ifdef USE_SINGLE_PRECISION
				row[i] = std::stof(item);
#else
				row[i] = std::stod(item);
#endif
			} else { /** the last entry does not have the delimiter after it */
#ifdef USE_SINGLE_PRECISION
				row[i] = std::stof(line);
#else
				row[i] = std::stod(line);
#endif
			}
			// printf("row[%ld] = %f", i, row[i]);
		}
	}

  private:
	const std::string_view delimiter;
	std::string line;
	std::string item;
};

} // namespace matrix_rw

#endif