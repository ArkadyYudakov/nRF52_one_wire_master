#ifndef	OW_PACKET_H__
#define OW_PACKET_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "ow_config.h"		
	
// 1-wire ROM commands
#define	OWM_CMD_READ            0x33    //*< reading ROM address. Device must be only one on bus */
#define	OWM_CMD_SKIP            0xCC    //*< skip ROM address. All devices on bus addressed      */
#define OWM_CMD_RESUME          0xA5    //*< last addressed device responds                      */
#define	OWM_CMD_MATCH           0x55    //*< onley divice with given ROM addressed               */
#ifdef OW_ROM_SEARCH_SUPPORT
#define	OWM_CMD_SEARCH          0xF0    //*< address searching procedure                         */
#define	OWM_CMD_ALARM_SEARCH    0xEC    //*< address searching procedure for alarmed devices     */
#endif

//#define OWM_CMD_OVERDRIVE       = 0x3C,
//#define OWM_CMD_MATCH_OVERDRIVE = 0x69,

// 1-wire data struct
typedef struct
{
	uint8_t* p_txbuf;                     //*< ptr to transmit data buffer                       */
	uint8_t* p_rxbuf;                     //*< ptr to receive data buffer                        */
	uint8_t tx_count;                     //*< number of bits to transmit                        */
	uint8_t rx_count;                     //*< number of bits to reseive                         */
} ow_packet_data_t;

#ifdef OW_ROM_SEARCH_SUPPORT
// 1-wire searching struct
typedef struct
{
	uint8_t last_discrepancy;           //*< last discrepancy position                           */
	uint8_t last_family_discrepancy;    //*< last discrepancy position in device family bits     */
	bool    last_device;                //*< if 1, last address was discovered                   */
	bool    consistency_fault;          //*< if 1, logical error was detected in search sequence */
} ow_search_data_t;								
#endif											

/**
 * Result of 1 WIRE packet processing, passing in callback parameter
 */
typedef enum
{
	OWMR_SUCCESS = 0,                     //*< saccess finishing of 1-wire transfer              */
	OWMR_NO_RESPONSE,                     //*< no devices present on the bus                     */
	OWMR_TIME_OUT,                        //*< timeout reached in wait flag procedure            */
#ifdef OW_ROM_SEARCH_SUPPORT
	OWMR_NOT_FOUND,                       //*< no alarmed devices on the bus (in alarm search)   */
	OWMR_SEARCH_CONSISTENCY_FAULT,        //*< logical error was detected in search procedure    */
#endif                                    //*< before transfer completion                        */
	OWMR_COMMUNICATION_ERROR,             //*< incorrect signal timing on bus was detected       */
} ow_result_t;

typedef union
{
	uint8_t raw[8];
	struct
	{
		uint8_t family;
		uint8_t serial[6];
		uint8_t crc8;
	};
} ROM_code_t;

// Callback function of 1-wire packet. Invoks after packet processed by 1-wire controlling module.
// Result and ptr to processed packet passes in callback parameters. 
// Higher level module, wich enqueue given packet, can instantly continue 1-wire activiti by 
// modifying packet in callback function and returning 1 from callback function. Packet will be
// retransmitted instead of the next packet from queue.
typedef struct ow_packet_t ow_packet_t;
typedef uint32_t(*ow_packet_callback_t)(ow_result_t result, ow_packet_t* p_ow_packet);
//typedef uint32_t(*ow_packet_callback_t)(ow_result_t result, void* p_ow_packet);
  
/**
 * 1-wire packet struct
 */
typedef struct ow_packet_t
{
	ow_packet_callback_t callback;      //*< packet callback. Invoked after transfer completion  */
	void*                p_context;     //*< context for given transfer packet                   */
	ROM_code_t*          p_ROM_code;    //*< ptr to 8 byte ROM address                           */
	uint8_t              ROM_command;   //*< ROM command (first byte of 1-wire transfer)         */
#if (defined (OW_MULTI_CHANNEL))
	uint8_t              channel;       //*< 1-wire channel for this packet                      */
#endif
	uint16_t delay_ms;                    //*< delay in milliseconds for hold power, wait flag   */
	struct
	{
		uint8_t          wait_flag  : 1;  //*< if 1, flag waiting procedure will be performed    */
#if (defined (OW_PARASITE_POWER_SUPPORT)) //*< before transfer completion                        */
		uint8_t          hold_power : 1;  //*< if 1, hold power procedure will be performed      */
#endif                                    //*< before transfer completion                        */
	};
	union
	{
		ow_packet_data_t data;            //*< struct for data transmitting and receiving        */
#ifdef OW_ROM_SEARCH_SUPPORT
		ow_search_data_t search;          //*< struct for searching process parameters           */
#endif
	};
} ow_packet_t;                            //*< and simple delay procedures                       */

// crc8 utility functions. Defined in ow_master.c	
uint8_t crc8(uint8_t crc, uint8_t value);
void docrc8(uint8_t* crc, uint8_t value);
bool checkcrc8(uint8_t crc, uint8_t* buf, uint32_t count);

#ifdef __cplusplus
}
#endif

#endif // OW_PACKET_H__
