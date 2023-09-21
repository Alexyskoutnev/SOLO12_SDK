#include "commander/utils.hpp"

bool isWhitespace(std::string str) {
    return std::all_of(str.begin(), str.end(), ::isspace);
}

double reduce_2_zero(double d){
    if (std::abs(d) < tolerance){
        return 0.0;
    } else {
        return d;
    }
}