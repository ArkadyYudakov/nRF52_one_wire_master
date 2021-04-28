#ifndef	OW_MASTER_H__
#define OW_MASTER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ow_config.h"	
#include "ow_packet.h"	
	
// 1-wire master callback function. Registering by higher level module, invoking after 
// transfer completion. Result of operation and ptr to packet passes in callback parameters. 
typedef void(*ow_master_callback_t)(ow_result_t result, ow_packet_t* p_ow_packet);

/** 
 * @brief 1-Wire master driver initialization. 
 *
 * Initializer of HAL module invoks in the process.
 *
 * @param callback callback provided by higher level module.
 */
void ow_master_initialize(ow_master_callback_t callback);

/**
 * @brief  1-Wire master driver uninitialization.
 * 
 * Initializer of HAL module invoks in the process.
 * 
 * @retval 0 success.
 * @retval 1 driver is busy.
*/
uint32_t ow_master_uninitialize();

/**
 * @brief Processing 1-wire packet
 * 
 * Launchs 1-WIRE packet processing. After completion registered callback invokes.
 * 
 * @warning Thread unsafe function. Must be called by higher level manager module,
 * which provide thread safe workflow.
 *
 * @param p_ow_packet  packet to process (ptr to)
 */
void ow_process_packet(ow_packet_t* p_ow_packet);

// crc8 utility functions.
uint8_t crc8(uint8_t crc, uint8_t value);
void docrc8(uint8_t* crc, uint8_t value);
bool checkcrc8(uint8_t crc, uint8_t* buf, uint32_t count);

#ifdef __cplusplus
}
#endif

#endif // OW_MASTER_H__
