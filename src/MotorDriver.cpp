#include <iostream>

#include "MotorDriver.h"

MotorDriver::MotorDriver()
{
    isEnable  = false;
    timeout = 0;
}

MotorDriver::~MotorDriver(){
    // std::cout << "Motordrivers are disconnected";
}

void MotorDriver::Enable(){
    isEnable = true;
}

void MotorDriver::Disable(){
    isEnable = false;
}

void MotorDriver::SetTimeout(int time){
    timeout = time;
}

void MotorDriver::SetMotors(Motor* motor1, Motor* motor2){
    this->motor1 = motor1;
    this->motor2 = motor2;
}

int MotorDriver::GetDriverStatus(){
    if (isEnable){
        std::cout << "MotorDriver Status: True" << std::endl; 
        return 0;
    } else {
        std::cout << "MotorDriver Status: False" << std::endl; 
        return -1;
    }
}