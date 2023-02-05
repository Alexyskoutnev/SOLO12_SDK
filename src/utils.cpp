#include <vector>
#include <string>

#include "utils.h"

#define DELIMITER ','

template <typename T>
std::vector<std::vector<T>> csv_reader(std::string file_name){
    std::string line;
    std::vector<std::vector<T>> rows {};
    std::ifstream file(file_name, std::ifstream::in);
    while (getline(file, line)){
        double tmpNum;
        std::vector<T> tmpVec;
        std::stringstream ss(line);
        while (ss >> tmpNum){
            std::string tmpstr;
            std::cout << "tmp: " << tmpstr << std::endl;
            tmpVec.push_back(tmpNum);
            if (!getline(ss, tmpstr, DELIMITER)){
                break;
            }
        }
        rows.append(tmpVec);
    }
    return rows;
}