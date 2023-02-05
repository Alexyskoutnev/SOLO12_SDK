#include "motor.h"
#include <cassert>

Motor::Motor(){
    position = 0;
    velocity = 0;
    current = 0;
    offsetPosition = 0;
    isEnabledFlag = false;
    isReadyFlag = false;
    indexToggleBitFlag = false;
    indexDectectedFlag = false;
}

void Motor::SetDriver(MotorDriver* driver){
    this->driver = driver;
}

void Motor::Enable(){
    isEnabledFlag = true;
}

void Motor::Disable(){
    isEnabledFlag = false;
}

bool Motor::isEnabled(){
    return isEnabledFlag;
}

bool Motor::isReady(){
    return isReadyFlag;
}

void Motor::SetPositionReference(double cmd){
    position_cmd = cmd;
}

void Motor::SetVelocityReference(double cmd){
    velocity_cmd = cmd;
}

void Motor::SetCurrentReference(double cmd){
    assert(cmd > 0);
    current_cmd = cmd;
}

void Motor::SetPositionOffset(double offset){
    offsetPosition = offset;
}

void Motor::SetKp(double kp_cmd){
    assert(kp_cmd > 0);
    kp = kp_cmd;
}


void Motor::SetKd(double kd_cmd){
    assert(kd_cmd > 0);
    kd = kd_cmd;
}

void Motor::SetMaxAmps(double amps){
    current_max = amps;
}

bool Motor::IndexDectected(){
    return indexToggleBitFlag;
}

bool Motor::GetIndexToggleBit(){
    return indexDectectedFlag;
}

double Motor::getPosition(){
    return position;
}

double Motor::getVelocity(){
    return velocity;

}

double Motor::getCurrent(){
    return current;
}

double Motor::getOffset()
{
    return offsetPosition;
}

void Print(){
    ;
}


