#include "utils.h"

#define DELIMITER ','

template <typename T>
std::vector<std::vector<T>> csv_reader(std::string file_name){
    std::string line;
    std::vector<std::vector<T>> rows {};
    std::ifstream file(file_name, std::ifstream::in);
    while (getline(file, line)){
        char tmpChar;
        std::vector<T> tmpVec;
        std::stringstream ss(line);
        std::string tmpstr;
        rows.push_back(tmpVec);
        while (getline(ss, tmpstr, DELIMITER)){
            std::cout << "tmpstr " << tmpstr << std::endl;
            rows.back().push_back(std::stod(tmpstr));
        }
    }
    return rows;
}