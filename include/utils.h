#ifndef UTILS_HEADER
#define UTILS_HEADER

#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>

template <typename T>
std::vector<std::vector<T>> csv_reader(std::string file_name);

#endif