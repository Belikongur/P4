#ifndef GUARD_LOWNET_UTIL_H
#define GUARD_LOWNET_UTIL_H

#include <stdint.h>

#include <lownet.h>

typedef struct {
	uint8_t mac[6];
	uint8_t node;
} lownet_identifier_t;

lownet_identifier_t lownet_lookup(uint8_t id);
lownet_identifier_t lownet_lookup_mac(const uint8_t* mac);

uint32_t lownet_crc(const lownet_frame_t* frame);

#endif
