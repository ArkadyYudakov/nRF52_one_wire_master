
#ifndef	OW_MASTER_HAL_H__
#define OW_MASTER_HAL_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
	
#include "ow_config.h"	
	
/**
 * Result of 1 WIRE HAL operation, passing in callback parameter
 */
typedef enum
{
	OWMHCR_READ_0     = 0x00,    /**< 0 readed                                               */
	OWMHCR_READ_1     = 0x01,    /**< 1 readed                                               */
	
	OWMHCR_RESET_OK,             /**< in reset procedure responce detected                   */
	OWMHCR_RESET_NO_RESPONCE,    /**< no response in reset procedure                         */
	
	OWMHCR_WRITE_OK,             /**< successful bit writing                                 */
	OWMHCR_SEQUENCE_OK,          /**< successful sequence transfer                           */

	OWMHCR_WAIT_OK,              /**< simple delay completed without errors                  */
	OWMHCR_FLAG_OK,              /**< ready flag read before time-out reached                */
	OWMHCR_TIME_OUT,             /**< ready flag not read before time-out reached            */
	
	OWMHCR_ERROR                 /**< incorrect signal timing on bus was detected            */
} owmh_callback_result_t;

// --------------------------------------------------------------------------------------------

// 1-wire master HAL callback function. Registering by ow_master module, invoking after 
// HAL operation completion. Result of operation passes in callback parameter. 
typedef void(*owmh_callback_t)(owmh_callback_result_t result);

/**
 * @brief 1 WIRE HAL initialization. 
 *
 * @param callback  callback invoking after HAL operation completed.
 */
void owm_hal_initialize(owmh_callback_t callback);

/**
 * @brief 1 WIRE HAL uninitialization. 
 *
 * Releasing hardware resources.
 *   
 * @retval 0 success.
 * @retval 1 driver is busy.
 */
uint32_t owm_hal_uninitialize(void);

#if (defined (OW_MULTI_CHANNEL))
/**
 * @brief 1-wire active channel establishing. 
 *
 * Hardware reinitialization for active channel changing.
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

/**
 * @brief Continuous 1-wire transfer.
 *
 * Transmitting tx_count bits from txdata, then resiving rx_count bits to rxdata 
 * If success, result in callback parameter OWMHCR_SEQUENCE_OK
 * If errors are detected, callback parameter = OWMHCR_ERROR
 */
void owmh_sequence(uint8_t* p_txdata, uint8_t* p_rxdata, uint8_t  tx_count, uint8_t  rx_count);

/**
 * @brief Waiting for ready flag. 
 *
 * Reading repeatedly untill "1" readed or time-out reached. 
 * Gap betwin readings - 1ms. 
 * If ready flag detected before time out, result in callback
 * parameter OWMHCR_FLAG_OK, else OWMHCR_TIME_OUT
 * If incorrect timing on bus detected, result - OWMHCR_ERROR
 *
 * @param time_out_ms  time-out delay value in microseconds.
 */
void owmh_wait_flag(uint16_t time_out_ms);

/**
 * @brief Simple delay. 
 *
 * If no errors detected, result in callback parameter OWMHCR_WAIT_OK
 * If incorrect timing on bus detected, result - OWMHCR_ERROR
 * 
 * @param delay_ms  delay value in microseconds.
 */
void owmh_delay(uint16_t delay_ms);
	
#if (defined (OW_PARASITE_POWER_SUPPORT))
/**
 * @brief Hold power.
 *
 * Reconfiguring pins for continuous power supplying
 * through data line in case of parasite powering.
 * If no errors detected, result OWMHCR_WAIT_OK
 * 
 * @param delay_ms  duration in microseconds.
*/
void owmh_hold_power(uint16_t delay_ms);
#endif

#ifdef __cplusplus
}
#endif

#endif // OW_MASTER_HAL_H__
