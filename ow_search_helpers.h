#ifndef	OW_HELPERS_H__
#define OW_HELPERS_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "ow_packet.h"
	
void ow_read_ROM_code(ow_packet_t* p_ow_packet);
	
#ifdef OW_ROM_SEARCH_SUPPORT
void ow_search_next(ow_packet_t* p_ow_xfer, bool alarm);
void ow_search_first(ow_packet_t* p_ow_xfer, bool alarm);
void ow_search_first_in_family(ow_packet_t* p_ow_xfer, uint8_t family_code, bool alarm);
void ow_search_verify(ow_packet_t* p_ow_xfer);
void ow_search_next_family(ow_packet_t* p_ow_xfer, bool alarm);
#endif
	
#ifdef __cplusplus
}
#endif

#endif // OW_HELPERS_H__
