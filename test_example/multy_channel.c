#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// platform dependant
#include "app_error.h"
#define  CHECK_ERROR_BOOL( bool_expresion ) APP_ERROR_CHECK_BOOL( bool_expresion )
#define  HANDLE_ERROR() APP_ERROR_CHECK_BOOL( false )
// end of platform dependant section

#include "nrf_gpio.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_timer.h"

#include "bc_scheduler.h"

#include "ow_manager.h"
#include "ow_search_helpers.h"
#include "example_module.h"
#include "ds18b20.h"

#ifdef OW_MULTI_CHANNEL

//*************************************************************************************/
typedef struct ow_channel_t
{
	u_int8_t sensors_count;
	bool     single_family;
	bool     parasite_power;
} ow_channel_t;

//*************************************************************************************/

#define DS18B20_FAMILY      0x28

#define DS18B20_RESOL       RES_11_BIT
#define DS18B20_POWER       POWER_CHECK

#define OW_CANNELS_COUNT    OW_CHANNEL_COUNT

#define SENSORS_MAX_COUNT   5

APP_TIMER_DEF(delay_before_read);

static uint8_t      m_sensors_count;
static uint8_t      m_channel_index;
static uint8_t      m_sensor_index;
static ds18b20_t    m_sensors[SENSORS_MAX_COUNT];
static ow_channel_t m_channels[OW_CANNELS_COUNT] = { { 0, true, false } };

static ROMcode_t    m_ROMcode;
static ROMcode_t    m_ROMcode_test;
static ow_packet_t  m_ow_packet;
static uint8_t		m_counter;
bool                m_sensor_instantiated = false;
bool				m_discovering_completed = false;
bool				m_conversion_cmd_sended_all = false;

//*************************************************************************************/
static int16_t int16_tempr(int16_t arg1, int16_t arg0)
{
	return arg1 * 16 + arg0 * 16 / 100;
}

static void log_hex(void* ptr, uint8_t length)
{
	for (int i = 0; i < length; ++i)
	{
		uint8_t byte = *(uint8_t*)(ptr + i);
		NRF_LOG_RAW_INFO("%02X ", byte);
	}
}

//-----------------------------------------------------------------------------------
static void read_next();
static void start_conversion();
//**********************************************************************************/

static void on_sensor_read(ds18b20_t* p_ds18b20)
{
	NRF_LOG_RAW_INFO("\n         - "); log_hex(&p_ds18b20->ROMcode.serial, 6);
//	NRF_LOG_HEXDUMP_INFO(p_ds18b20->ROMcode.serial, 6); log_hex(&p_ds18b20->ROMcode.serial, 6);
	NRF_LOG_RAW_INFO(": Channel #%i, ParasitePower %i, SkipROMcode %i, temperature %i.%i",
		p_ds18b20->channel, 
		p_ds18b20->parasite_powered, 
		p_ds18b20->skip_ROM_perm,
		(p_ds18b20->temperature)>>4,
		(((p_ds18b20->temperature)&0x000F)*10)/16
		);
	int16_t rate = p_ds18b20->temperature - (int16_t)p_ds18b20->extra_data;
	if (rate > int16_tempr(1, 0))
	{
		NRF_LOG_RAW_INFO(" ^ ^ ^");
	}
	else if (rate > int16_tempr(0, 50))
	{
		NRF_LOG_RAW_INFO(" ^ ^");
	}
	else if (rate > int16_tempr(0, 20))
	{
		NRF_LOG_RAW_INFO(" ^");
	}
	else if (-rate > int16_tempr(0, 60))
	{
		NRF_LOG_RAW_INFO(" v v v");
	}
	else if (-rate > int16_tempr(0, 30))
	{
		NRF_LOG_RAW_INFO(" v v");
	}
	else if (-rate > int16_tempr(0, 15))
	{
		NRF_LOG_RAW_INFO(" v");
	}
	p_ds18b20->extra_data = (uint32_t)p_ds18b20->temperature;
	
	read_next(); 
}

static void read_next() 
{
	// check if all sensors processed
	if (m_sensor_index >= m_sensors_count)
		// All devices have read. Initiate temperature conversion.
		start_conversion();
	else
		// read next sensor
		ds18b20_read_safe(&m_sensors[m_sensor_index++], on_sensor_read, false);
}

static void on_delay_before_read_time_out(void * p_context)
{
	// Initiate sensors reading
	NRF_LOG_RAW_INFO("\n>>> Start reading...");
	m_sensor_index  = 0;
	read_next();
}

//**********************************************************************************/
static void start_convertion_next();

static void on_sensor_start_convertion(ds18b20_t* p_ds18b20)
{
	if((p_ds18b20->result == SUCCESS)&&(p_ds18b20->parasite_powered))
	{
		NRF_LOG_RAW_INFO("\n         - Convert command completed");
	}
	else if((p_ds18b20->result == SUCCESS)&&(p_ds18b20->wait_ready_flag))
	{
		NRF_LOG_RAW_INFO("\n         - Convert command completed on ready flag.");
	}
	else if(p_ds18b20->result == WAITING_FLAG_TIME_OUT)
	{
		NRF_LOG_RAW_INFO("\n         - Convert command completed on time-out.");
	}
	else if(p_ds18b20->result != SUCCESS)
	{
		NRF_LOG_RAW_INFO("\n         - ERROR detected while convert command executing!");
	}

	if (m_conversion_cmd_sended_all)
	{
		// Delay 1 sec. before sensors reading.
		app_timer_start(delay_before_read, 20000, NULL);
		NRF_LOG_RAW_INFO("\n");
	}
	else
		start_convertion_next();
}


uint32_t on_all_start_convertion(ow_result_t result, ow_packet_t* p_ow_packet)
{
	if (result != OWMR_SUCCESS)
	{
		NRF_LOG_RAW_INFO("\n         - ERROR detected while groupe convert command executing!");
	}
	else if(p_ow_packet->hold_power)
	{
		NRF_LOG_RAW_INFO("\n         - Convert command completed");
	}

	if (m_conversion_cmd_sended_all)
	{
		m_sensor_index = 0;
		// Delay 1 sec. before sensors reading.
		app_timer_start(delay_before_read, 20000, NULL);
		NRF_LOG_RAW_INFO("\n");
	}
	else
		start_convertion_next();
	return 0;
}

static void start_convertion_next()
{
	// process next sensor
	uint8_t channel_index = m_sensors[m_sensor_index].channel;
	
	if ((m_channels[channel_index].single_family)&&(m_channels[channel_index].sensors_count > 1))
	{
		// No devices of other types on bus. Group command can be done
		ds18b20_start_conversion_all(channel_index,
			on_all_start_convertion,
			(m_channels[channel_index].parasite_power ? OW_HOLD_POWER : OW_NOT_WAIT),
			DS18B20_RESOL);
		NRF_LOG_RAW_INFO("\n         - Groupe command on channel #%i", channel_index);
		if (m_channels[channel_index].parasite_power)
		{
			NRF_LOG_RAW_INFO(". Hold power delay...");
		}
		else
		{
			NRF_LOG_RAW_INFO(". No hold power delay. Immediate return.");
		}
		// Skip sensors with same channel 
		m_sensor_index += m_channels[channel_index].sensors_count;

	}
	else
	{
		// start temperature conversion in certain sensor
		// in case of single normal powered device on channel test wait flag mode 
		m_sensors[m_sensor_index].wait_ready_flag = 
			((m_channels[channel_index].sensors_count == 1)&&(!m_channels[channel_index].parasite_power));
		ds18b20_start_conversion(&m_sensors[m_sensor_index], on_sensor_start_convertion);
		NRF_LOG_RAW_INFO("\n         - Sensor "); log_hex(&m_sensors[m_sensor_index].ROMcode.serial, 6);
		NRF_LOG_RAW_INFO(" : conversion started. ");
		if (m_channels[channel_index].parasite_power)
		{
			NRF_LOG_RAW_INFO("Hold power delay...");
		}
		else if (m_sensors[m_sensor_index].wait_ready_flag)
		{
			NRF_LOG_RAW_INFO("Wait ready flag...");
		}
		else
		{
			NRF_LOG_RAW_INFO("No delay, immediate return.");
		}
		++m_sensor_index;
	}
	// check if all sensors processed
	if (m_sensor_index >= m_sensors_count)
	{
		// all devices have started temperature conversion. Initiate reading.
		m_conversion_cmd_sended_all = true;
		return;
	}
}

static void start_conversion()
{
	if (m_sensors_count == 0)
	{
		NRF_LOG_RAW_INFO("\n!!! No ds18b20 sensors was discovered! Restart programm.");
	}
	// Start sensors scanning. At first step - scart temperature convertions for all.
	m_sensor_index  = 0;
	m_conversion_cmd_sended_all = false;
	NRF_LOG_RAW_INFO("\n\n>>> Temperature conversion initiating... ");
	start_convertion_next();
}

//*********************************************************************************************/
static void discover_next(bool first_on_channel);
void start_ow_discovering();
static uint32_t on_search_transfer_completed(ow_result_t result, ow_packet_t* p_ow_packet);
//----------------------------------------------------------------------------------------------
static void on_sensor_config_sicronized(ds18b20_t* p_ds18b20)
{
	if (p_ds18b20->parasite_powered)
		m_channels[p_ds18b20->channel].parasite_power = true;
	if(m_discovering_completed)
		start_conversion();
}

uint32_t on_read_ROM_tested(ow_result_t result, ow_packet_t* p_ow_packet)
{
	int success;
	switch (result)
	{
	case OWMR_SUCCESS:
		success = memcmp(&m_ROMcode, &m_ROMcode_test, 8);
		if (success == 0)
		{
			NRF_LOG_RAW_INFO("\n         - Read ROM command successfully tested (case of single device on bus).");
		}
		else
		{
			NRF_LOG_RAW_INFO("\n         - While read ROM command testing (case of single device on bus), wrong code readed.");
		}
		// restore ow packet
		p_ow_packet->callback = on_search_transfer_completed;
		p_ow_packet->p_ROMcode = &m_ROMcode;
		// continue with interrupted discovering workflow
		if (++m_channel_index < OW_CANNELS_COUNT)
			// Start discovering new channel
			discover_next(true);
		else
		{
			// OW discovering completed. Initiate sensors scanning
//			if(m_sensor_instantiated)
//				m_discovering_completed = true; // start_conversion();
//			else
				start_conversion();
		}
		break;

	case OWMR_COMMUNICATION_ERROR:
		NRF_LOG_RAW_INFO("\n!!! Communication error while read ROM command testing on channel %i detected!", m_channel_index);
		NRF_LOG_RAW_INFO("\n!!! Discovering  restarted!");
		start_ow_discovering();
		break;
	default:
		HANDLE_ERROR();
	}
	return 0;
}

// hadling search transfer complition 
static uint32_t on_search_transfer_completed(ow_result_t result, ow_packet_t* p_ow_packet)
{
	m_sensor_instantiated = false;
	switch (result)
	{
	case OWMR_SUCCESS:
		if (m_ROMcode.family == DS18B20_FAMILY)
		{
			// ds18b20 discovered
			++m_channels[m_channel_index].sensors_count;
			if (m_sensors_count < SENSORS_MAX_COUNT)
			{
				NRF_LOG_RAW_INFO("\n\n      ds18b20 discovered, serial = ");  log_hex(&m_ROMcode.serial, 6);
				NRF_LOG_RAW_INFO("\n         - Local object initialized.");
				ds18b20_t* p_sensor = &m_sensors[m_sensors_count];
				ds18b20_initialize(p_sensor, m_channel_index, DS18B20_RESOL, DS18B20_POWER); 
				ds18b20_set_ROMcode(p_sensor, &m_ROMcode);
				ds18b20_sinc_config(p_sensor, on_sensor_config_sicronized);
				m_sensor_instantiated = true;
				++m_sensors_count;
			}
			else
			{
				NRF_LOG_RAW_INFO("\n\n      ds18b20 discovered, serial = ");  log_hex(&m_ROMcode.serial, 6);
				NRF_LOG_RAW_INFO("\n         - Not instantiated! Increase SENSORS_MAX_COUNT.");
			}
		}
		else
		{
			// Device of other then ds18b20 family discovered. Mark channel as not safe for group commands.
			NRF_LOG_RAW_INFO("\n\n      Device other then ds18b20 discovered, family = %x", m_ROMcode.family); // log_hex(&m_ROMcode.serial, 6);
			m_channels[m_channel_index].single_family = false;
		}
		
		//  Go to next channel if last device on channel discovered. If last channel - finish discovering.
		if (p_ow_packet->search.last_device)
		{
			// Last device on channel discovered.
			// if single sensor on bas - choose skip ROMcode mode permanently
			if((m_channels[m_channel_index].sensors_count == 1)&&(m_channels[m_channel_index].single_family)
					&&m_sensor_instantiated)
			{
				m_sensors[m_sensor_index].skip_ROM_perm = true;
				// in case of single device on bus, test 1-wire command of ROM code reading 
				p_ow_packet->callback = on_read_ROM_tested;
				p_ow_packet->delay_ms = 0;
				p_ow_packet->p_ROMcode = &m_ROMcode_test;
				ow_read_ROMcode(p_ow_packet);
				break;
			}
			// Go to next channel or terminate discovering.
			if (++m_channel_index < OW_CANNELS_COUNT)
				// Start discovering new channel
				discover_next(true);
			else
			{
				// OW discovering completed. Initiate sensors scanning
				if(m_sensor_instantiated)
					m_discovering_completed = true; // start_conversion();
				else
					start_conversion();
			}
			break;
		}
		else
			discover_next(false);
		break;
		
	case OWMR_NO_RESPONSE:
		NRF_LOG_RAW_INFO("\n        No devices found on channel #%i", m_channel_index);
		// go to next channel
		if (++m_channel_index < OW_CANNELS_COUNT)
			// Start discovering new channel
			discover_next(true);
		else
			// OW discovering completed. Initiate sensors scanning
			start_conversion();
		break;
		
	case OWMR_CONSISTENCY_FAULT:
		NRF_LOG_RAW_INFO("\n!!! Logical error while discovering on channel %i detected! ", m_channel_index);
		NRF_LOG_RAW_INFO("\n!!! Discovering restarted!");
		start_ow_discovering();
		break;

	case OWMR_COMMUNICATION_ERROR:
		NRF_LOG_RAW_INFO("\n!!! Communication error while discovering on channel %i detected!", m_channel_index);
		NRF_LOG_RAW_INFO("\n!!! Discovering  restarted!");
		start_ow_discovering();
		break;
	default:
		HANDLE_ERROR();
	}
	return 0;
}

static void discover_next(bool first_on_channel)
{
	if (first_on_channel)
	{
		m_ow_packet.channel = m_channel_index;
		m_channels[m_channel_index].sensors_count = 0;
		m_channels[m_channel_index].single_family = true;
		m_channels[m_channel_index].parasite_power = false;
		ow_search_first(&m_ow_packet, false);
//		NRF_LOG_RAW_INFO("\n     Cannel %i:", m_channel_index);
	}
	else
		ow_search_next(&m_ow_packet, false);
}

void start_ow_discovering()
{
	// initialize timer
	APP_ERROR_CHECK(app_timer_create(&delay_before_read, APP_TIMER_MODE_SINGLE_SHOT, on_delay_before_read_time_out));
	m_sensors_count = 0;
//	m_sensor_index  = 0;
	m_channel_index = 0;
	m_ow_packet.p_ROMcode = &m_ROMcode;
	m_ow_packet.callback  = on_search_transfer_completed;
	discover_next(true);
}

//**********************************************************************************/
#endif
