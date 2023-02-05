#include <string>
#include <stdlib.h>
#include <stdio.h>

#include "interface.h"

// Interface *Interface::instance = NULL;

Interface::Interface(const std::string &if_name)
{
    uint8_t mac_add[6] = {0xa0, 0x1d, 0x48, 0x12, 0xa0, 0xc5}; //take it as an argument?
    uint8_t dest_add[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
//this->my_mac_  = mac_add;
 //   this->dest_mac_ = dest_add;
 //
    //std::copy(std::begin(mac_add), std::end(mac_add), this->my_mac_);
    //this->if_name_ = if_name;
    //instance = this;
}
Interface::Interface(const MasterBoardInterface &to_be_copied) : MasterBoardInterface::MasterBoardInterface(to_be_copied.if_name_, to_be_copied.listener_mode)
{
}

Interface::~Interface(){
    ;
}

void stop(){

	;
}

void callback(uint8_t src_mac[6], uint8_t *data, int len){
	

	;

}
