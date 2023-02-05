#include <cstdint>
#include <cstring>
#include <iostream>
#include <math.h>
#include <signal.h>

#include "interface.h"


Interface::Interface(const std::string &if_name)
{
  uint8_t my_mac[6] = {0xa0, 0x1d, 0x48, 0x12, 0xa0, 0xc5};
  uint8_t dest_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  std::memcpy(this->my_mac_, my_mac, 6);
  std::memcpy(this->dest_mac_, dest_mac, 6);
  this->if_name_ = if_name;
  this->listener_mode = listener_mode;
  for (int i = 0; i <N_DRIVER_CNT; i++){
        motors[2 * i].SetDriver(&motor_driver[i]);
        motors[2 * i + 1].SetDriver(&motor_driver[i]);
        motor_driver[i].SetMotors(&motors[2*i], &motors[2*i + 1]);
  }
}

Interface::~Interface()
{
  delete (link_handler_);
}

int Interface::Init()
{
    std::cout << "if_name -> " << if_name_ << std::endl;
    memset(&command_packet, 0, sizeof(command_packet_t));
    memset(&sensor_packet, 0, sizeof(sensor_packet_t));
    memset(&init_packet, 0, sizeof(init_packet_t));
    memset(&ack_packet, 0, sizeof(ack_packet_t));

    timeout = false; //We set interface time out to false at start
    first_command_sent_ = false; //Indicator if we sent a command
    init_sent = false; //indicator if we sent init command
    ack_received = false; 
    try 
    { 
        if (if_name_[0] == 'e'){
            std::cout << "Using Ethernet Connection" << std::endl;
            link_handler_ = new ETHERNET_manager(if_name_, my_mac_, dest_mac_); //Test if mac address is needed?
            link_handler_->set_recv_callback(this);
            link_handler_->start();
        } else {
            return -1;
        }
    } 
    catch(std::bad_alloc & exception){
        throw std::runtime_error("Error in allocating link_handler");
    }
    ParseSensorData();
    return 0;
}

void Interface::ParseSensorData(){
    
    received_packet_mutex.lock();//Makes sure that incoming data doesn't corrupt stored data
    //Read Motor Driver Data
    for (int i = 0; i < N_DRIVER_CNT; i++){
        ;
    }
    received_packet_mutex.unlock();

}

int Interface::Stop(){
    std::cout << "Stopping the interface for robot" << std::endl;
    if (link_handler_ != NULL){
        link_handler_->stop();
        return 0;
    }
    else {
        return -1;
    }
}

void Interface::GenerateSessionID(){
    session_id = (uint16_t)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}


void Interface::callback(uint8_t /*src_mac*/[6], uint8_t *data, int len)
{
  ; 
}
