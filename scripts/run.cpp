#include <iostream>
#include <string>
#include <stdexcept>
#include <cstdlib>
#include <cassert>

#include "interface.h"


#define N_DRIVER_CNT 6


int main(int argc, char** argv){
    std::cout << "Main Script " << std::endl;
    if (argc < 2){
        throw std::runtime_error("Please provide the Ethernet interface name");
    } else {
        if (argc == 4){
            double kp = std::atof(argv[2]);
            double kd = std::atof(argv[3]);
            assert(kp > 0);
            assert(kd > 0);
        }
    }
    double dt = 0.001;
    double t = 0;
    double iq_sat = 4.0;
    double init_joint_pos[N_DRIVER_CNT  * 2] = {0};

   Interface interface(argv[1]);
   

    // interface.init();

    return 0;
}
