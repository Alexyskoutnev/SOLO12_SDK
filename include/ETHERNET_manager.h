#ifndef ETHERNET_MANAGER_H
#define ETHERNET_MANAGER_H


#include <stdlib.h>
#include <stdint.h>

#include "Link_manager.h"
#include "ETHERNET_types.h"


#define LEN_RAWBYTES_MAX 512


class ETHERNET_manager : public LINK_manager {
	public:
		ETHERNET_manager() : LINK_manager(&myETHpacket) {}

		ETHERNET_manager(const std::string& interface) : LINK_manager(&myETHpacket, interface) {}

		ETHERNET_manager(const std::string& interface, uint8_t src_mac[6], uint8_t dst_mac[6])
		: LINK_manager(&myETHpacket, interface) {
			set_src_mac(src_mac);
			set_dst_mac(dst_mac);
		}

	private:
		ETHERNET_packet myETHpacket;
};

#endif