#include <iostream>
#include <string>
#include <stdexcept>
#include <cstdlib>
#include <cassert>

#include "interface.h"

#define N_DRIVER_CNT 6
#define TIMEOUT 5

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
    interface.Init();

    for (int i  = 0; i < N_DRIVER_CNT; i++){
        interface.motor_driver[i].motor1->SetCurrentReference(0.0);
        interface.motor_driver[i].motor2->SetCurrentReference(0.0);
        interface.motor_driver->Enable();
        interface.motor_driver[i].motor1->Enable();
        interface.motor_driver[i].motor2->Enable();
        interface.motor_driver->SetTimeout(TIMEOUT);
    }

    std::chrono::time_point<std::chrono::system_clock> t_last_update = std::chrono::system_clock::now();
    while (!interface.IsTimeout() && !interface.IsAckMsgReceived()){
        std::chrono::duration<double> diff = std::chrono::system_clock::now() - t_last_update;
        if (diff.count() > dt){
            t_last_update = std::chrono::system_clock::now();
            interface.SendInit();
        }
    }

    //CSV data reader
    std::string file_name = "./data/joint_trajectory_jan_23.csv";
    std::vector<std::vector<double>> joint_traj_vec = csv_reader<double>(file_name);

    //main run loop
    while (!interface.IsTimeout()){
        std::chrono::duration<double> diff = std::chrono::system_clock::now() - t_last_update;
        if (diff.count() > dt){
            t_last_update = std::chrono::system_clock::now();
            t += dt;
            interface.ParseSensorData();
            for (int i = 0; i < N_DRIVER_CNT * 2; i++){
                if (interface.motors[i].isEnabled()){
                    ;    
                }
            }
        }

    }
    interface.Stop();
    return 0;
}
