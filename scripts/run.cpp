#include <iostream>
#include <string>
#include <stdexcept>
#include <cstdlib>
#include <cassert>


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
            std::cout << "tmpstr " << tmpstr << std::endl;
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
    

    // Interface interface(argv[1]);
    // interface.Init();

    // for (int i  = 0; i < N_DRIVER_CNT; i++){
    //     interface.motor_driver[i].motor1->SetCurrentReference(0.0);
    //     interface.motor_driver[i].motor2->SetCurrentReference(0.0);
    //     interface.motor_driver->Enable();
    //     interface.motor_driver[i].motor1->Enable();
    //     interface.motor_driver[i].motor2->Enable();
    //     interface.motor_driver->SetTimeout(TIMEOUT);
    // }

    // std::chrono::time_point<std::chrono::system_clock> t_last_update = std::chrono::system_clock::now();
    // while (!interface.IsTimeout() && !interface.IsAckMsgReceived()){
    //     std::chrono::duration<double> diff = std::chrono::system_clock::now() - t_last_update;
    //     if (diff.count() > dt){
    //         t_last_update = std::chrono::system_clock::now();
    //         interface.SendInit();
    //     }
    // }

    

    //CSV data reader
    // assert((file_name.empty() != NULL));
    std::vector<std::vector<double>> joint_traj_vec = csv_reader<double>(file_name);
    int joint_traj_idx = 0;
    int motor_idx = 0;
    int state = 0;
    

    for (int j = 0; j < 100; j++){
        for (auto cmd : joint_traj_vec[joint_traj_idx]) {
                std::cout << "idx: " << joint_traj_idx << " Cmd: " << cmd << std::endl;
                std::cout << "state: " << state << std::endl;
                if (joint_traj_idx == 0)
                    continue;
                switch (state)
                {
                case 1:
                    // interface.motors[i].SetPositionReference(cmd);
                    std::cout << "pos" << std::endl;
                    state++;
                    motor_idx++;
                    break;
                case 13:
                    // interface.motors[i].SetVelocityReference(cmd);
                    std::cout << "velocity" << std::endl;
                    state++;   
                    motor_idx++; 
                    break;
                case 25:
                    // interface.motors[i].SetCurrentReference(cmd);
                    std::cout << "current" << std::endl;
                    state++;
                    motor_idx++;
                    break;
                default:
                    state++;
                    motor_idx++;
                    break;
                }  
        }
        joint_traj_idx++;
        motor_idx = 0;
        state = 0;
    }



    // //main run loop
    // while (!interface.IsTimeout()){
    //     std::chrono::duration<double> diff = std::chrono::system_clock::now() - t_last_update;
    //     if (diff.count() > dt){
    //         t_last_update = std::chrono::system_clock::now();
    //         t += dt;
    //         interface.ParseSensorData();
    //         for (int i = 0; i < N_DRIVER_CNT * 2; i++){
    //             if (interface.motors[i].isEnabled()){
    //                 for (auto cmd : joint_traj_vec[joint_traj_idx]) {
    //                     if (joint_traj_idx == 0)
    //                         continue;
    //                     switch (state)
    //                     {
    //                     case 1:
    //                         interface.motors[i].SetPositionReference(cmd);
    //                         state++;
    //                         break;
    //                     case 13:
    //                         interface.motors[i].SetVelocityReference(cmd);
    //                         state++;    
    //                         break;
    //                     case 25:
    //                         interface.motors[i].SetCurrentReference(cmd);
    //                         state++;
    //                         break;
    //                     default:
    //                         state++;
    //                         break;
    //                     }
    //                 }    
    //             }
    //         }
    //         joint_traj_idx++;
    //         try
    //         {
    //             interface.SendCommand();
    //         }
    //         catch(std::exception& e)
    //         {
    //             std::cerr << "interface failed to send command" << std::endl;
    //             std::cerr << e.what() << '\n';
    //         }
    //     }
    // }
    // interface.Stop();
    return 0;
}
