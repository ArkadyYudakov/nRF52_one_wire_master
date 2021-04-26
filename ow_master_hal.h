
#ifndef	OW_MASTER_HAL_H__
#define OW_MASTER_HAL_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
	
#include "ow_config.h"	
	
// Result of 1 WIRE HAL primitives, passing in callback parameter
typedef enum
{
	OWMHCR_READ_0     = 0x00,		// readed bit 0
	OWMHCR_READ_1     = 0x01,		// readed bit 1
	
#ifdef OW_ROM_SEARCH_SUPPORT
	OWMHCR_POLL_00,					// in polling procedure readed bits 0, 0
	OWMHCR_POLL_01,					// in polling procedure readed bits 0, 1
	OWMHCR_POLL_10,					// in polling procedure readed bits 1, 0
	OWMHCR_POLL_11,					// in polling procedure readed bits 1, 1
#endif
	
	OWMHCR_RESET_OK,				// in reset procedure devices on bus detected
	OWMHCR_RESET_NO_RESPONCE,		// no response of remote devices in reset procedure
	
	OWMHCR_WRITE_OK,				// succesful writeing
	OWMHCR_PACKET_OK,				// succesful transfer

	OWMHCR_WAIT_OK,					// simple delay completed without errors
	OWMHCR_FLAG_OK,					// flag read before timeout reached
	OWMHCR_TIME_OUT,				// flag not read before timeout reached
	
	OWMHCR_ERROR					// incorrect signal timing on bus was detected
} owmh_callback_result_t;

// --------------------------------------------------------------------------------------------

typedef void(*owmh_callback_t)(owmh_callback_result_t result);

/**
 * @brief 1 WIRE HAL initialization. 
 *
 * hardware and variables initialization.
 */
void owm_hal_initialize(owmh_callback_t callback);

/**
 * @brief 1 WIRE HAL uninitialization. 
 *
 * Releasing hardware resources.
 */
	uint32_t owm_hal_uninitialize(void);

#if (defined (OW_MULTI_CHANNEL))
/**
 * @brief 1 WIRE channel setting. 
 *
 * Hardware reinitialization for defined channel.
 */
void ow_set_channel(uint8_t channel);
#endif

// -------------------------------- 1-WIRE HAL primitives --------------------------------------

/**
 * @brief 1-wire reset pulse. 
 *
 * A negative reset pulse is followed by device presence detection zone.
 * If presence of device on 1-wire bus detected, callback parameter = OWMHCR_RESET_OK. 
 * If no presence detected, callback parameter = OWMHCR_RESET_NO_RESPONCE.
 * If incorrect timing on bus detected, callback parameter = OWMHCR_ERROR. 
 */
void owmh_reset(void);

/**
 * @brief 1-wire write bit primitive. 
 *
 * A negative bit_0 or bit_1 pulse generated depend on uint8_t bit parameter.
 * If incorrect timing on bus detected, callback parameter = OWMHCR_ERROR, else OWMHCR_WRITE_OK
 */
void owmh_write(uint8_t bit);	
	
/**
 * @brief 1-wire read bit primitive. 
 *
 * A negative write bit_1 pulse is folowed by device responce detecting zone.
 * If proper device bit_o responce detected, callback parameter = OWMHCR_READ_0
 * If no responce detected, callback parameter = OWMHCR_READ_1
 * If incorrect timing on bus detected, callback parameter = OWMHCR_ERROR
 */
void owmh_read(void);

#ifdef OW_ROM_SEARCH_SUPPORT
	/**
 * @brief Reading of two complement bits in searching process. 
 *
 * Result in callback parameter respectively: 
 *       OWMHCR_POLL_00 / OWMHCR_POLL_01 / OWMHCR_POLL_10 / OWMHCR_POLL_11
 * If incorrect timing on bus detected, callback parameter = OWMHCR_ERROR
 */
void owmh_poll(void);
#endif

/**
 * @brief Continuous 1-WIRE transfer.
 *
 * Transmitting tx_count bits from txdata, then resiving rx_count bits to rxdata 
 * If success, result in callback parameter OWMHCR_XFER_OK
 * If errors are detected, callback parameter = OWMHCR_ERROR
 */
void owmh_sequence(uint8_t* p_txdata, uint8_t* p_rxdata, uint8_t  tx_count, uint8_t  rx_count);

/**
 * @brief Bit reading repeatedly untill "1" readed. Gap betwin readings - 1ms. 
 *
 * Limit time in parameter uint16_t max_wait_ms (microseconds)
 * If flag detected before time out, result OWMHCR_FLAG_OK,
 * else OWMHCR_TIME_OUT
 * If incorrect timing on bus detected, callback parameter = OWMHCR_ERROR
 */
void owmh_wait_flag(uint16_t max_wait_ms);

/**
 * @brief Delay delay_ms in microseconds. 
 *
 * Limit time in parameter uint16_t max_wait_ms (microseconds)
 * If no errors detected, result OWMHCR_WAIT_OK
 */
void owmh_delay(uint16_t delay_ms);
	
#if (defined (OW_PARASITE_POWER_SUPPORT))
/**
 * @brief Hold power.
 *
 * In case of 2 wire connection, hold continuous 
 * power for period delay_ms in microseconds.
 * If no errors detected, result OWMHCR_WAIT_OK
 */
	void owmh_hold_power(uint16_t delay_ms);
#endif

#ifdef __cplusplus
}
#endif

#endif // OW_MASTER_HAL_H__
