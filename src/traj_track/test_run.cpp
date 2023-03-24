#include <iostream>
#include <string>
#include <stdexcept>
#include <cstdlib>
#include <cassert>
#include <vector>

#include "interface.h"

#define N_DRIVER_CNT 6
#define TIMEOUT 5

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
            rows.back().push_back(std::stod(tmpstr));
        }
    }
    return rows;
}

int main(int argc, char** argv){
    std::cout << "Main Script " << std::endl;
    std::string file_name;
    if (argc < 2){
        throw std::runtime_error("Please provide the Ethernet interface name");
    } else {
        if (argc == 4){
            double kp = std::atof(argv[1]);
            double kd = std::atof(argv[2]);
            file_name = argv[3];
            assert(kp > 0);
            assert(kd > 0);
        } else {
            double kp = 1.0;
            double kd = 0.1;
            file_name = "./data/joint_trajectory_jan_23.csv";
        }
    }
    double dt = 0.001;
    double t = 0;
    double iq_sat = 4.0;
    double init_joint_pos[N_DRIVER_CNT  * 2] = {0};
    

   

    

    //CSV data reader
    // assert((file_name.empty() != NULL));
    std::vector<std::vector<double>> joint_traj_vec = csv_reader<double>(file_name);
    int joint_traj_idx = 0;
    int motor_idx = 0;
    int state = 0;
    

    for (int j = 0; j < 10; j++){
        
        for (auto cmd : joint_traj_vec[joint_traj_idx]) {
                if (state == 1 or state == 13 or state == 25)
                {
                    motor_idx = 0;
                }
                std::cout << "state -> " << state << " motor index " << motor_idx << std::endl; 
                if (state >= 1 and state < 13)
                {
                    std::cout << "pos -> " << cmd << std::endl;
                    motor_idx++;
                } else if (state >= 13 and state < 25){ 
                    std::cout << "velt -> " << cmd << std::endl;
                    motor_idx++; 
                } else if (state >= 25 and state < 37){
                    std::cout << "cur -> " << cmd << std::endl;
                    motor_idx++;
                    break;
                }
                state++;
                }
            joint_traj_idx++;
            motor_idx = 0;
            state = 0;
    }
    return 0;
}
