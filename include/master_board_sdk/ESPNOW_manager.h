#ifndef ESPNOW_manager_H
#define ESPNOW_manager_H

#include <linux/filter.h>

#include "master_board_sdk/Link_manager.h"
#include "master_board_sdk/ESPNOW_types.h"


class ESPNOW_manager : public LINK_manager {
	public:
		ESPNOW_manager() : LINK_manager(&myWiFipacket) {}

		ESPNOW_manager(const std::string& interface) : LINK_manager(&myWiFipacket, interface) {}

		ESPNOW_manager(const std::string& interface, uint8_t datarate, uint16_t channel_freq, uint8_t src_mac[6], uint8_t dst_mac[6], bool filterOn)
		: LINK_manager(&myWiFipacket, interface) {
			set_channel(channel_freq);
			set_datarate(datarate);
			set_src_mac(src_mac);
			set_dst_mac(dst_mac);

			bpf.filter = NULL;
			bpf.len = 0;
			
			if(filterOn) {
				set_filter(dst_mac, src_mac);
			} else {
				set_filter(NULL, NULL);
			}
		}
		~ESPNOW_manager();

		void set_filter(uint8_t *src_mac, uint8_t *dst_mac);
		void unset_filter();
		void bind_filter();
		void stop() override;
		
		void set_channel(uint16_t channel_freq) { myWiFipacket.set_channel(channel_freq); }
		uint16_t get_channel() { return myWiFipacket.data.radiotap.channel_freq; }
		void set_datarate(uint8_t datarate) { myWiFipacket.set_datarate(datarate); }


	private:
		struct sock_fprog bpf;
		
		ESPNOW_packet myWiFipacket;

};



#endif
