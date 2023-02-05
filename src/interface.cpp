#include <string>
#include <stdlib.h>

#include "interface.h"

// Interface *Interface::instance = NULL;

Interface::Interface(const std::string &if_name)
{
    uint8_t mac_add[6] = {0xa0, 0x1d, 0x48, 0x12, 0xa0, 0xc5}; //take it as an argument?
    uint8_t dest_add[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    memcpy(this->my_mac_, mac_add, 6);
    memcpy(this->dest_mac_, dest_add, 6);
    this->if_name_ = if_name;
    instance = this;
}

Interface::~Interface(){
    ;
}
