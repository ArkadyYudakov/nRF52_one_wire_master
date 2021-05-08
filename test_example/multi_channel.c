#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// platform dependent
#include "app_error.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#define  CHECK_ERROR( int_expresion ) APP_ERROR_CHECK( int_expresion )
#define  CHECK_ERROR_BOOL( bool_expresion ) APP_ERROR_CHECK_BOOL( bool_expresion )
#define  HANDLE_ERROR() APP_ERROR_CHECK_BOOL( false )
#define  LOG_PRINTF NRF_LOG_RAW_INFO
// end of platform dependent section

#include "ow_manager.h"
#include "ow_search_helpers.h"
#include "ds18b20.h"

#ifdef OW_MULTI_CHANNEL

//----------------------------------------------------------------------------------------------

#define DS18B20_FAMILY      0x28

#define DS18B20_RESOL       RES_12_BIT
#define DS18B20_POWER       POWER_CHECK

#define SENSORS_MAX_COUNT   5

#define DELAY_BETWEEN_SCANS 30000
#define MAX_SCANS_IN_SERIES 5

APP_TIMER_DEF(delay_timer);

typedef struct ow_channel_t
{
	u_int8_t sensors_count;
	bool     single_family;
	bool     parasite_power;
} ow_channel_t;

typedef enum scan_mode_t
{
	SEPARATE_CONVERT_READ,
	READ_AND_CONVERT,
	CONVERT_AND_READ
}scan_mode_t;

scan_mode_t			m_scan_mode;
static uint8_t      m_sensors_count;
static uint8_t      m_channel_index;
static uint8_t      m_sensor_index;
static ds18b20_t    m_sensors[SENSORS_MAX_COUNT];
static ow_channel_t m_channels[OW_CHANNEL_COUNT];
static ow_packet_t  m_ow_packet;
static ROM_code_t   m_ROMcode;
static ROM_code_t   m_ROMcode_test;
static uint8_t      m_scans_counter = 0;

//----------------------------------------------------------------------------------------------
#define ENCODE_TEMPR(msb, lsb) (msb * 16 + lsb * 16 / 100)

static void log_hex(void* ptr, uint8_t length)
{
	for (int i = 0; i < length; ++i)
	{
		uint8_t byte = *(uint8_t*)(ptr + i);
		LOG_PRINTF("%02X ", byte);
	}
}
//----------------------------------------------------------------------------------------------

static void read_next();
static void start_convertion_next();
bool	m_conversion_finished;

//----------------------------------------------------------------------------------------------
// ds18b20 sensor reading result handling.

static void reading_ds18b20_callback(ds18b20_t* p_ds18b20)
{
	if (p_ds18b20->result == SUCCESS)
	{
		LOG_PRINTF("tempr = %i.%i",
			(p_ds18b20->temperature)>>4,
			(((p_ds18b20->temperature)&0x000F)*10)/16
			);
		int16_t rate = p_ds18b20->temperature - (int16_t)p_ds18b20->extra_data;
		if (rate > ENCODE_TEMPR(1, 0))
			{ LOG_PRINTF(" ^ ^ ^"); }
		else if (rate > ENCODE_TEMPR(0, 50))
			{ LOG_PRINTF(" ^ ^"); }
		else if (rate > ENCODE_TEMPR(0, 20))
			{ LOG_PRINTF(" ^"); }
		else if (-rate > ENCODE_TEMPR(0, 60))
			{ LOG_PRINTF(" v v v"); }
		else if (-rate > ENCODE_TEMPR(0, 30))
			{ LOG_PRINTF(" v v"); }
		else if (-rate > ENCODE_TEMPR(0, 15))
			{ LOG_PRINTF(" v"); }
		p_ds18b20->extra_data = (uint32_t)p_ds18b20->temperature;
	}
	else if (p_ds18b20->result == WAITING_FLAG_TIME_OUT)
	{ 
		LOG_PRINTF("Time-out. Not completed!"); 
	}
	else
	{ 
		LOG_PRINTF("ERROR! (Crc8 check)"); 
	}
	read_next(); 
}


static void read_next() 
{
	if (m_sensor_index >= m_sensors_count)
	{
		// All sensors readed. Repeat or transit to other reading mode.
		m_sensor_index = 0;
		if (++m_scans_counter > MAX_SCANS_IN_SERIES)
		{
			// Transit to other reading mode.
			m_scans_counter = 0;
			switch (m_scan_mode)
			{
			case SEPARATE_CONVERT_READ:	
				LOG_PRINTF("\n\n------------------------------------------------------------------------------------");
				LOG_PRINTF("\n Polling in mode 1. Reading followed by conversion command for every distinct sensor.");
				LOG_PRINTF("\n For parasite powered devices response delayed because of power support while conversion.");
				LOG_PRINTF("\n\n*    Reading...");
				m_scan_mode = READ_AND_CONVERT;
				read_next();
				break;
			case READ_AND_CONVERT:	
				LOG_PRINTF("\n\n------------------------------------------------------------------------------------");
				LOG_PRINTF("\n Polling in mode 2. Reading after conversion for every distinct sensor.");
				LOG_PRINTF("\n In case of normal powered devices, ready flag used to detect conversion end.");
				LOG_PRINTF("\n\n*    Reading...");
				m_scan_mode = CONVERT_AND_READ;
				read_next();
				break;
			case CONVERT_AND_READ:	
				LOG_PRINTF("\n\n------------------------------------------------------------------------------------");
				LOG_PRINTF("\n Polling in mode 0. Conversion command for all, pause, then reading all sensors.");
				LOG_PRINTF("\n For parasite powered devices start conversion command lasts until conversion end.");
				LOG_PRINTF("\n\n     Conversion... ");
				m_scan_mode = SEPARATE_CONVERT_READ;
				m_conversion_finished = false;
				start_convertion_next();
			}
		}
		else
		{
			// Repeat reading series in the same mode.
			switch (m_scan_mode)
			{
			case SEPARATE_CONVERT_READ:	
				// start conversion fase in mode 0.
				m_conversion_finished = false;
				LOG_PRINTF("\n\n     Conversion... ");
				start_convertion_next();
				break;
			case READ_AND_CONVERT:	
			case CONVERT_AND_READ:	
				// Delay before next reading series.
				app_timer_start(delay_timer, DELAY_BETWEEN_SCANS, NULL);
			}
		}
	}
	else
	{
		// In the middle of series. Reading next sensor.
		LOG_PRINTF("\n         - "); log_hex(&m_sensors[m_sensor_index].ROM_code.serial, 6);
		LOG_PRINTF(": #%i, PP %i, SR %i.", 
			m_sensors[m_sensor_index].channel,
			m_sensors[m_sensor_index].parasite_powered, 
			m_sensors[m_sensor_index].skip_ROM_code );
		switch (m_scan_mode)
		{
		case SEPARATE_CONVERT_READ:	
			LOG_PRINTF(" Read... ");
			ds18b20_read_safe(&m_sensors[m_sensor_index++], reading_ds18b20_callback);
			break;
		case READ_AND_CONVERT:	
			LOG_PRINTF(" Read & convert... ");
			ds18b20_read_and_convert(&m_sensors[m_sensor_index++], reading_ds18b20_callback);	
			break;
		case CONVERT_AND_READ:	
			LOG_PRINTF(" Convert & read... ");
			ds18b20_convert_and_read(&m_sensors[m_sensor_index++], reading_ds18b20_callback);	
		}
	}
}

//----------------------------------------------------------------------------------------------
static void delay_timer_on_time_out_callback(void * p_context)
{
	UNUSED_PARAMETER(p_context);	
	// Initiate sensors reading series
	LOG_PRINTF("\n\n*    Reading...");
//	m_reading_finished = false;
	m_sensor_index  = 0;
	read_next();
}

//----------------------------------------------------------------------------------------------
static void start_convertion_next();
bool	m_conversion_finished;

//----------------------------------------------------------------------------------------------
// ds18b20 sensor callback after conversion start (or completion in some cases).
static void start_convertion_ds18b20_callback(ds18b20_t* p_ds18b20)
{
	if ((p_ds18b20->result == SUCCESS)&&(p_ds18b20->parasite_powered))
	{
		LOG_PRINTF(" Completed."); 
	}
	else if ((p_ds18b20->result == SUCCESS)&&(p_ds18b20->waiting_mode == OW_WAIT_FLAG))
	{ 
		LOG_PRINTF(" Ready flag readed."); 
	}
	else if (p_ds18b20->result == WAITING_FLAG_TIME_OUT)
	{ 
		LOG_PRINTF(" Time-out. Not completed!"); 
	}
	else if (p_ds18b20->result != SUCCESS)
	{ 
		LOG_PRINTF(" ERROR detected!"); 
	}

	if (m_conversion_finished)
	{
		// Delay before reading.
		app_timer_start(delay_timer, DELAY_BETWEEN_SCANS, NULL);
	}
	else
		start_convertion_next();
}

// Handling completion of groupe start conversion command on channel.
uint32_t start_convertion_ow_callback(ow_result_t result, ow_packet_t* p_ow_packet)
{
	if (result != OWMR_SUCCESS)
	{
		LOG_PRINTF("\n         - ERROR detected while groupe convert command sending!");
	}
	else if((p_ow_packet->delay_ms > 0)&&(
#ifdef OW_PARASITE_POWER_SUPPORT
		(p_ow_packet->hold_power)||
#endif
		(p_ow_packet->wait_flag)))
	{
		LOG_PRINTF(" Conversion completed.");
	}

	if (m_conversion_finished)
	{
		m_sensor_index = 0;
		// Delay 1 sec. before sensors reading.
		app_timer_start(delay_timer, DELAY_BETWEEN_SCANS, NULL);
		LOG_PRINTF("\n");
	}
	else
		start_convertion_next();
	return 0;
}

static void start_convertion_next()
{
	uint8_t channel_index = m_sensors[m_sensor_index].channel;
	
	if ((m_channels[channel_index].single_family)&&(m_channels[channel_index].sensors_count > 1))
	{
		// No devices of other types on bus. Group command can be done
		ds18b20_start_conversion_all(channel_index,
			start_convertion_ow_callback,
#ifdef OW_PARASITE_POWER_SUPPORT
			(m_channels[channel_index].parasite_power ? OW_HOLD_POWER : OW_NOT_WAIT),
#else
			OW_NOT_WAIT,
#endif
			DS18B20_RESOL);
		LOG_PRINTF("\n         - Groupe conversion command on channel #%i", channel_index);
		if (m_channels[channel_index].parasite_power)
		{
			LOG_PRINTF(" Hold power delay...");
		}
		else
		{
			LOG_PRINTF(" No delay.");
		}
		// Skip sensors with same channel. 
		m_sensor_index += m_channels[channel_index].sensors_count;
	}
	else
	{
		// Start temperature conversion in certain sensor.
		ds18b20_start_conversion(&m_sensors[m_sensor_index], start_convertion_ds18b20_callback);
		LOG_PRINTF("\n         - "); log_hex(&m_sensors[m_sensor_index].ROM_code.serial, 6);
		LOG_PRINTF(": conversion started. ");
		if (m_channels[channel_index].parasite_power)
		{
			LOG_PRINTF("Hold power delay...");
		}
		else if (m_sensors[m_sensor_index].waiting_mode == OW_WAIT_FLAG)
		{
			LOG_PRINTF("Wait ready flag...");
		}
		else if (m_sensors[m_sensor_index].waiting_mode == OW_WAIT_DELAY)
		{
			LOG_PRINTF("Simple delay...");
		}
		else
		{
			LOG_PRINTF("No delay.");
		}
		// Shift to next sensor.
		++m_sensor_index;
	}
	// Check if all sensors processed.
	if (m_sensor_index >= m_sensors_count)
	{
		// Signal for ds18b20 callback to initiate reading fase.
		m_conversion_finished = true;
		return;
	}
}

//----------------------------------------------------------------------------------------------
static void start_discovering();
static void discover_next(bool first_on_channel);
static uint32_t discovering_ow_callback(ow_result_t result, ow_packet_t* p_ow_packet);

bool	m_sensor_instantiated = false;
bool	m_discovering_completed = false;

//----------------------------------------------------------------------------------------------

static void start_polling()
{
	LOG_PRINTF("\n\n>>> Polling of ds18b20 sensors started.");
	m_sensor_index = m_sensors_count;
	m_scans_counter = MAX_SCANS_IN_SERIES;
	m_scan_mode = CONVERT_AND_READ;
	read_next();
}

// ds18b20 sensor initializing result handling.
static void initializing_ds18b20_callback(ds18b20_t* p_ds18b20)
{
	// Mark channel if parasite powered device present. 
	if (p_ds18b20->parasite_powered)
		m_channels[p_ds18b20->channel].parasite_power = true;
	// If it is last discovered device, initiate scanning.
	if(m_discovering_completed)
		start_polling();
}

// Handling result of ROM code reading command.
uint32_t reading_ROM_ow_callback(ow_result_t result, ow_packet_t* p_ow_packet)
{
	switch (result)
	{
	case OWMR_SUCCESS:
		if (memcmp(&m_ROMcode, &m_ROMcode_test, 8) == 0)
		{
			LOG_PRINTF("\n         - Read ROM command successfully tested.");
		}
		else
		{
			LOG_PRINTF("\n         - Read ROM command test failed!");
		}
		// restore ow packet
		p_ow_packet->callback = discovering_ow_callback;
		p_ow_packet->p_ROM_code = &m_ROMcode;
		// continue with interrupted discovering workflow
		if (++m_channel_index < OW_CHANNEL_COUNT)
			// Start discovering new channel
			discover_next(true);
		else
		{
			// OW discovering completed. Initiate sensors scanning
			start_polling();
		}
		break;

	case OWMR_COMMUNICATION_ERROR:
		LOG_PRINTF("\n!!! Communication error while read ROM command testing!", m_channel_index);
		LOG_PRINTF("\n!!! Discovering  restarted!");
		start_discovering();
		break;
	default:
		HANDLE_ERROR();
	}
	return 0;
}

// 1-wire search rout result handling.  
static uint32_t discovering_ow_callback(ow_result_t result, ow_packet_t* p_ow_packet)
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
				LOG_PRINTF("\n\n      ds18b20 discovered, serial = ");  log_hex(&m_ROMcode.serial, 6);
				LOG_PRINTF("\n         - Local object initialized.");
				ds18b20_t* p_sensor = &m_sensors[m_sensors_count];
#ifdef OW_PARASITE_POWER_SUPPORT
				ds18b20_initialize(p_sensor, m_channel_index, DS18B20_RESOL, DS18B20_POWER); 
#else
				ds18b20_initialize(p_sensor, m_channel_index, DS18B20_RESOL); 
#endif
				ds18b20_set_ROM_code(p_sensor, &m_ROMcode);
				ds18b20_sincronize(p_sensor, initializing_ds18b20_callback);
				m_sensor_instantiated = true;
				m_sensor_index = m_sensors_count;
				++m_sensors_count;
			}
			else
			{
				LOG_PRINTF("\n\n      ds18b20 discovered, serial = ");  log_hex(&m_ROMcode.serial, 6);
				LOG_PRINTF("\n         - Not instantiated! Increase SENSORS_MAX_COUNT.");
			}
		}
		else
		{
			// Device of other then ds18b20 family discovered. Mark channel as not safe for group commands.
			LOG_PRINTF("\n\n      Device other then ds18b20 discovered, family = %02x", m_ROMcode.family);
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
				m_sensors[m_sensor_index].skip_ROM_code = true;
				// in case of single device on bus, test 1-wire command of ROM code reading 
				p_ow_packet->callback = reading_ROM_ow_callback;
				p_ow_packet->delay_ms = 0;
				p_ow_packet->p_ROM_code = &m_ROMcode_test;
				ow_read_ROM_code(p_ow_packet);
				break;
			}
			// Go to next channel or terminate discovering.
			if (++m_channel_index < OW_CHANNEL_COUNT)
				// Start discovering new channel
				discover_next(true);
			else
			{
				// OW discovering completed. Initiate sensors scanning
				if(m_sensor_instantiated)
				{
					m_discovering_completed = true; // start_conversion();
				}
				else if(m_sensors_count == 0)
				{
					LOG_PRINTF("\n!!! No ds18b20 sensors was discovered! Restart programm.");
				}
				else
					start_polling();
			}
			break;
		}
		else
			discover_next(false);
		break;
		
	case OWMR_NO_RESPONSE:
		LOG_PRINTF("\n        No devices found on channel #%i", m_channel_index);
		// go to next channel
		if (++m_channel_index < OW_CHANNEL_COUNT)
			// Start discovering new channel
			discover_next(true);
		else
			// OW discovering completed. Initiate sensors scanning
			start_polling();
		break;
		
	case OWMR_SEARCH_CONSISTENCY_FAULT:
		LOG_PRINTF("\n!!! Logical error while discovering on channel %i detected! ", m_channel_index);
		LOG_PRINTF("\n!!! Discovering restarted!");
		start_discovering();
		break;

	case OWMR_COMMUNICATION_ERROR:
		LOG_PRINTF("\n!!! Communication error while discovering on channel %i detected!", m_channel_index);
		LOG_PRINTF("\n!!! Discovering  restarted!");
		start_discovering();
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
	}
	else
		ow_search_next(&m_ow_packet, false);
}

static void start_discovering()
{
	m_sensors_count = 0;
	m_channel_index = 0;
	m_ow_packet.p_ROM_code = &m_ROMcode;
	m_ow_packet.callback  = discovering_ow_callback;
	
	discover_next(true);
}

void start_ow_test()
{
	// Initialize 1-wire driver.
	ow_manager_initialize();

	// Create timer for delay between sensor scans.
	CHECK_ERROR(app_timer_create(&delay_timer, APP_TIMER_MODE_SINGLE_SHOT, delay_timer_on_time_out_callback));
	
	LOG_PRINTF("\n>>> One wire discovering started.");
	start_discovering();
}


#endif
