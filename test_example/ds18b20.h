#ifndef DS18B20_H__
#define DS18B20_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "nrf_assert.h"

#include "ow_manager.h"

#define DS18B20_ALARM_TEMPR_SUPPORT

// forward declaration
typedef struct ds18b20_t ds18b20_t;
typedef void(*ds18b20_callback_t)(ds18b20_t* context);
	
// Result of ds18b20 command executing.
typedef enum ds18b20_result_t
{
	PENDING = 0,
	NO_RESPONSE,
	DEVICE_NOT_FOUND,
	CONFIG_WRITING_ERROR,
	COMMUNICATION_ERROR,
	WAITING_FLAG_TIME_OUT,
	SUCCESS
} ds18b20_result_t;

// ds18b20 temperature conversion resolution
typedef enum ds18b20_resolution_t
{
	RES_9_BIT  = 0x1F,
	RES_10_BIT = 0x3F,
	RES_11_BIT = 0x5F,
	RES_12_BIT = 0x7F
} ds18b20_resolution_t;
		
typedef enum ds18b20_command_t 
{ 
	// DS18B20 commands as states 
	CMD_IDLE = 0,
	CMD_CONFIG_READ,
	CMD_TEMP_READ_FAST,
	CMD_TEMP_CONVERT = 0x44,
	CMD_TEMP_READ_SAFE = 0xBE,
	CMD_CONFIG_WRITE = 0x4E,
	CMD_EEPROM_WRITE = 0x48,
	CMD_EEPROM_RECALL = 0xB8,
#ifdef OW_PARASITE_POWER_SUPPORT
	CMD_POWER_SUPPLY_READ = 0xB4
#endif
} ds18b20_command_t;

// Mode of waiting after time consuming ds18b20 commands.
typedef enum ds18b20_waiting_t
{
	OW_NOT_WAIT,                             /*   */
	OW_WAIT_FLAG,                            /*   */
	OW_WAIT_DELAY,                           /*   */
#ifdef OW_PARASITE_POWER_SUPPORT
	OW_HOLD_POWER                            /*   */
#endif
} ds18b20_waiting_t;
	
#ifdef OW_PARASITE_POWER_SUPPORT
// ds18b20 power handling mode.
typedef enum ds18b20_power_config_t
{
	POWER_NORMAL,                            /*   */
	POWER_PARASITE,                          /*   */
	POWER_CHECK,                             /*   */
} ds18b20_power_config_t;
#endif

// ds18b20 object data structure.
typedef struct ds18b20_t
{
	uint32_t                extra_data;      /*   */
	int16_t					temperature;     /*    */
	ds18b20_resolution_t	resolution;      /*   */
#ifdef DS18B20_ALARM_TEMPR_SUPPORT
	int8_t                  low_tempr;       /*   */
	int8_t                  high_tempr;      /*   */
#endif
	ds18b20_command_t		command;         /*   */
	ds18b20_result_t		result;          /*   */
#ifdef OW_MULTI_CHANNEL
	uint8_t				    channel;         /*   */
#endif 
	ds18b20_callback_t		callback;        /*   */
	ow_packet_t				ow_packet;       /*   */
	ROM_code_t				ROM_code;         /*   */
	uint8_t					databuffer[10];  /*   */
	struct
	{
		bool        skip_ROM_code      : 1;   /*   */
		bool        parasite_powered   : 1;   /*   */
		bool        check_power	       : 1;   /*   */
		ds18b20_waiting_t waiting_mode : 2;   /*   */
		bool        convert_after_read : 1;   /*   */
		bool        read_after_convert : 1;   /*   */
		bool        scratchpad_writed  : 1;   /*   */
	};
} ds18b20_t;


#ifdef OW_PARASITE_POWER_SUPPORT
#ifdef OW_MULTI_CHANNEL
void ds18b20_initialize(ds18b20_t* p_self, uint8_t channel, ds18b20_resolution_t resolution, 
		                                   ds18b20_power_config_t power_config);
#else
void ds18b20_initialize(ds18b20_t* p_self, ds18b20_resolution_t resolution, 
		                                   ds18b20_power_config_t power_config);
#endif
#else
#ifdef OW_MULTI_CHANNEL
void ds18b20_initialize(ds18b20_t* p_self, uint8_t channel, ds18b20_resolution_t resolution);
#else
void ds18b20_initialize(ds18b20_t* p_self, ds18b20_resolution_t resolution);
#endif	
#endif	
void ds18b20_set_ROM_code(ds18b20_t* p_self, ROM_code_t* p_ROM_code);
void ds18b20_set_waiting_mode(ds18b20_t* p_self, ds18b20_waiting_t waiting_mode);

#ifdef DS18B20_ALARM_TEMPR_SUPPORT
void ds18b20_set_alarm_tempr(ds18b20_t* p_self, int8_t low_tempr, int8_t high_tempr);
#endif

void ds18b20_skip_ROM_code(ds18b20_t* p_self, bool skip_ROM_code);

void ds18b20_sincronize(ds18b20_t* p_self, ds18b20_callback_t callback);
void ds18b20_start_conversion(ds18b20_t* p_self, ds18b20_callback_t callback);

#ifdef OW_MULTI_CHANNEL
void ds18b20_start_conversion_all(uint8_t channel, ow_packet_callback_t callback, 
								ds18b20_waiting_t waiting_mode, ds18b20_resolution_t resolution);
#else
void ds18b20_start_conversion_all(ow_packet_callback_t callback, 
								ds18b20_waiting_t waiting_mode, ds18b20_resolution_t resolution);
#endif
	
void ds18b20_read_fast(ds18b20_t* p_self, ds18b20_callback_t callback);
void ds18b20_read_safe(ds18b20_t* p_self, ds18b20_callback_t callback);
void ds18b20_convert_and_read(ds18b20_t* p_self, ds18b20_callback_t callback);
void ds18b20_read_and_convert(ds18b20_t* p_self, ds18b20_callback_t callback);
	
#ifdef __cplusplus
}
#endif

#endif // DS18B20_H__
 