#ifndef ESPNOW_TYPES_H
#define ESPNOW_TYPES_H


#include <stdint.h>
#include <string.h>

#include "master_board_sdk/Link_types.h"


#define DATARATE_1Mbps 0x02
#define DATARATE_2Mbps 0x04
#define DATARATE_6Mbps 0x0c
#define DATARATE_9Mbps 0x12
#define DATARATE_12Mbps 0x18
#define DATARATE_18Mbps 0x24
#define DATARATE_24Mbps 0x30
#define DATARATE_36Mbps 0x48
#define DATARATE_48Mbps 0x60
#define DATARATE_54Mbps 0x6c

#define CHANNEL_freq_1 2412
#define CHANNEL_freq_2 2417
#define CHANNEL_freq_3 2422
#define CHANNEL_freq_4 2427
#define CHANNEL_freq_5 2432
#define CHANNEL_freq_6 2437
#define CHANNEL_freq_7 2442
#define CHANNEL_freq_8 2447
#define CHANNEL_freq_9 2452
#define CHANNEL_freq_10 2457
#define CHANNEL_freq_11 2462
#define CHANNEL_freq_12 2467
#define CHANNEL_freq_13 2472
#define CHANNEL_freq_14 2484

#define WLAN_LEN 24
#define ACTIONFRAME_HEADER_LEN 8
#define VENDORSPECIFIC_CONTENT_LEN 7

struct IEEE80211_radiotap {
	uint8_t version;
    uint8_t pad;
    uint16_t length;
    uint32_t present;
    uint8_t flags;
    uint8_t datarate;
    uint16_t channel_freq;
    uint16_t channel_flags_quarter;

	//Default values for ESPNOW
	IEEE80211_radiotap() {
		this->version = 0;
		this->pad = 0;
		this->length = sizeof(IEEE80211_radiotap);
		this->present = 0x0000000e;
		this->flags = 0x10;
		this->datarate = DATARATE_6Mbps;
		this->channel_freq = CHANNEL_freq_1;
		this->channel_flags_quarter = 0x00c0;
	}
} __attribute__((__packed__));


struct IEEE80211_vendorspecific {
	uint8_t elementID;
	uint8_t length;
	uint8_t	OUI[3];
	uint8_t type;
	uint8_t version;
	uint8_t payload[250];

	//Default values for ESPNOW
	IEEE80211_vendorspecific() {
		this->elementID = 0xdd;
		this->OUI[0] = 0x18;
		this->OUI[1] = 0xfe;
		this->OUI[2] = 0x34;
		this->type = 0x04;
		this->version = 0x01;
	}

	void set_length(int payload_length) {this->length = static_cast<uint8_t>(payload_length + 5);}
} __attribute__((__packed__));


struct IEEE80211_actionframe {
	uint8_t category_code;
    uint8_t OUI[3];
	uint8_t unknown_bytes[4];
	struct IEEE80211_vendorspecific content;

	//Default values for ESPNOW
	IEEE80211_actionframe() {
		this->category_code = 0x7f;
		this->OUI[0] = 0x18;
		this->OUI[1] = 0xfe;
		this->OUI[2] = 0x34;
		bzero(this->unknown_bytes, 4);
	}
} __attribute__((__packed__));


struct IEEE80211_wlan {
	uint8_t type;
    uint8_t flags;
    uint16_t duration;
    uint8_t da[6];
    uint8_t sa[6];
    uint8_t bssid[6];
    uint16_t seq;
	struct IEEE80211_actionframe actionframe;
    uint32_t fcs;

	//Default values for ESPNOW
	IEEE80211_wlan() {
		this->type = 0xd0;
		this->flags = 0x00;
		this->duration = 0x0000; //Will be recalculated by the hardware...
		this->seq = 0x0280;
		this->fcs = 0x00000000;	//Will be recalculated by the hardware...
	}
} __attribute__((__packed__));

struct ESPNOW_data {
	struct IEEE80211_radiotap radiotap;
	struct IEEE80211_wlan wlan;
} __attribute__((__packed__));


struct ESPNOW_packet : Packet_t {
	struct ESPNOW_data data;

	void set_channel(uint16_t channel_freq);
	void set_datarate(uint8_t datarate);


	/** Start overriding inherited methods **/
	void set_src_mac(uint8_t my_mac[6]) 		override;
	void set_dst_mac(uint8_t dst_mac[6]) 		override;
	
	int toBytes(uint8_t *bytes, int max_len) 	override;
	uint8_t* get_payload_ptr() 					override;
	int get_payload_len() 						override;
	void set_payload_len(int len) 				override;

	uint8_t* get_src_mac_FromRaw(uint8_t *raw_bytes, int len)	override;
	int get_payload_len_FromRaw(uint8_t *raw_bytes, int len)	override;
	uint8_t* get_payload_FromRaw(uint8_t *raw_bytes, int len)	override;
	/** Finish overriding inherited methods **/


	static int get_radiotap_len_FromRaw(uint8_t *raw_bytes, int len);

};

typedef struct ESPNOW_packet ESPNOW_packet;

#endif
