#ifndef	OW_MASTER_H__
#define OW_MASTER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ow_config.h"	
#include "ow_packet.h"	
	
// 1-wire master callback function. Registering by higher level module, invoking after transfer completion.
// Result of completed transfer and ptr to packet struct passes in callback parameters 
typedef void(*ow_master_callback_t)(ow_result_t result, ow_packet_t* p_ow_packet);

// 1-WIRE master initialization
// Higher level module register callback, wich invoking after transfer completion.
void ow_master_initialize(ow_master_callback_t callback);

// 1-WIRE master deinitialization
// If success, 0 returning. If driver is busy, 1 returning.
uint32_t ow_master_uninitialize();

// Launch 1-WIRE packet transfer. After transfer completion callback invokes.
// Thread unsafe function. Must be called by manager module, which provide thread safe workflow.
// Ptr to packet struct passes in function parameter. Description of packet structure in ow_packet.h
void ow_process_packet(ow_packet_t* p_ow_packet);

// crc8 utility functions.
uint8_t crc8(uint8_t crc, uint8_t value);
void docrc8(uint8_t* crc, uint8_t value);
bool checkcrc8(uint8_t crc, uint8_t* buf, uint32_t count);

#ifdef __cplusplus
}
#endif

#endif // OW_MASTER_H__
