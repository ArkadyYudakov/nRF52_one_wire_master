#include <stdbool.h>
#include <stdint.h>

// platform dependent
#include "app_error.h"
#define  CHECK_ERROR_BOOL( bool_expresion ) APP_ERROR_CHECK_BOOL( bool_expresion )
#define  HANDLE_ERROR() APP_ERROR_CHECK_BOOL( false )

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#define  LOG_PRINTF NRF_LOG_RAW_INFO
// end of platform dependent section

#include "ds18b20.h"

static uint32_t on_ow_transfer_completed(ow_result_t  result, ow_packet_t* p_ow_packet); 
static void prepare_ow_packet(ds18b20_t* p_self, ds18b20_command_t command);
static void send_command(ds18b20_t* p_self, ds18b20_command_t command, ds18b20_callback_t callback);

static uint8_t convert_command = CMD_TEMP_CONVERT;
static ow_packet_t  m_ow_packet = { .ROM_command = OWM_CMD_SKIP };

static void log_hex(void* ptr, uint8_t length)
{
	for (int i = 0; i < length; ++i)
	{
		uint8_t byte = *(uint8_t*)(ptr + i);
		LOG_PRINTF("%x ", byte);
	}
}

static uint16_t waiting_delay(ds18b20_resolution_t resolution)
{
	uint16_t result;
	switch (resolution)
	{
	case RES_12_BIT:
		result  = 750;
		break;
	case RES_11_BIT:
		result  = 375;
		break;
	case RES_10_BIT:
		result  = 190;
		break;
	case RES_9_BIT:
		result  = 95;
	}
	return result;
}
//----------------------------------------- configuring -------------------------------------------

#ifdef OW_PARASITE_POWER_SUPPORT
#ifdef OW_MULTI_CHANNEL
void ds18b20_initialize(ds18b20_t* p_self, uint8_t channel, ds18b20_resolution_t resolution, 
		                                   ds18b20_power_config_t power_config)
#else
void ds18b20_initialize(ds18b20_t* p_self, ds18b20_resolution_t resolution, 
		                                   ds18b20_power_config_t power_config)
#endif
#else
#ifdef OW_MULTI_CHANNEL
void ds18b20_initialize(ds18b20_t* p_self, uint8_t channel, ds18b20_resolution_t resolution)
#else
void ds18b20_initialize(ds18b20_t* p_self, ds18b20_resolution_t resolution)
#endif	
#endif	
{
	memset(p_self, 0, sizeof(ds18b20_t));
	p_self->command = CMD_IDLE;
	p_self->result = PENDING;
	p_self->callback = NULL;
	p_self->temperature = 0;
	p_self->resolution = resolution;
	p_self->scratchpad_writed = false;
#ifdef DS18B20_ALARM_TEMPR_SUPPORT
	p_self->low_tempr = 0;
	p_self->high_tempr = 100;
#endif
#if (defined (OW_MULTI_CHANNEL))
	p_self->channel = channel;
	p_self->ow_packet.channel = channel;
#endif 
#ifdef OW_PARASITE_POWER_SUPPORT
	if (power_config == POWER_PARASITE)
		p_self->parasite_powered = true;
	if (power_config == POWER_CHECK)
		p_self->check_power = true;
#endif
	p_self->ow_packet.callback = on_ow_transfer_completed;
	p_self->ow_packet.p_context = p_self;
	p_self->ow_packet.p_ROM_code = &p_self->ROM_code;
}

void ds18b20_set_ROM_code(ds18b20_t* p_self, ROM_code_t* p_ROM_code)
{
	memcpy(&p_self->ROM_code, p_ROM_code, sizeof(ROM_code_t));
	p_self->ow_packet.ROM_command = (p_ROM_code->family ? OWM_CMD_MATCH : OWM_CMD_SKIP);
}

void ds18b20_skip_ROM_code(ds18b20_t* p_self, bool skip_ROM_code)
{
	p_self->skip_ROM_code = skip_ROM_code;
}

void ds18b20_set_waiting_mode(ds18b20_t* p_self, ds18b20_waiting_t waiting_mode)
{
#ifdef OW_PARASITE_POWER_SUPPORT
	if(waiting_mode == OW_HOLD_POWER)
		HANDLE_ERROR();
#endif
	p_self->waiting_mode = waiting_mode;
}


#ifdef DS18B20_ALARM_TEMPR_SUPPORT
void ds18b20_set_alarm_tempr(ds18b20_t* p_self, int8_t low_tempr, int8_t high_tempr)
{
	p_self->low_tempr = low_tempr;
	p_self->high_tempr = high_tempr;
}
#endif
//----------------------------------------- commands ----------------------------------------------

void ds18b20_sincronize(ds18b20_t* p_self, ds18b20_callback_t callback)
{
	p_self->convert_after_read = false;
	p_self->read_after_convert = false;
#ifdef OW_PARASITE_POWER_SUPPORT
	if (p_self->check_power)
	{
		send_command(p_self, CMD_POWER_SUPPLY_READ, callback);
	}
	else
#endif
	{
		send_command(p_self, CMD_CONFIG_READ, callback);
	}
}

void ds18b20_start_conversion(ds18b20_t* p_self, ds18b20_callback_t callback)
{
	p_self->convert_after_read = false;
	p_self->read_after_convert = false;
	send_command(p_self, CMD_TEMP_CONVERT, callback);
}

#ifdef OW_MULTI_CHANNEL
void ds18b20_start_conversion_all(uint8_t channel, ow_packet_callback_t callback, 
								ds18b20_waiting_t waiting_mode, ds18b20_resolution_t resolution) {
	m_ow_packet.channel = channel;
#else 
void ds18b20_start_conversion_all(ow_packet_callback_t callback, 
								ds18b20_waiting_t waiting_mode, ds18b20_resolution_t resolution) {
#endif
	m_ow_packet.callback = callback;
	m_ow_packet.delay_ms  = 0;
	m_ow_packet.wait_flag = false;
#ifdef OW_PARASITE_POWER_SUPPORT
	m_ow_packet.hold_power = false;
#endif
	
	if (waiting_mode != OW_NOT_WAIT)
	{
		m_ow_packet.delay_ms = waiting_delay(resolution);
		
		if(waiting_mode == OW_WAIT_FLAG)
			m_ow_packet.wait_flag = true;

#ifdef OW_PARASITE_POWER_SUPPORT
		if(waiting_mode == OW_HOLD_POWER)
			m_ow_packet.hold_power = true;
#endif
	}

	m_ow_packet.data.p_txbuf  = &convert_command;
	m_ow_packet.data.tx_count = 8;
	m_ow_packet.data.rx_count = 0;
									
	ow_enqueue_packet(&m_ow_packet);
}

void ds18b20_read_fast(ds18b20_t* p_self, ds18b20_callback_t callback)
{
	p_self->convert_after_read = false;
	p_self->read_after_convert = false;
	send_command(p_self, CMD_TEMP_READ_FAST, callback);
}

void ds18b20_read_safe(ds18b20_t* p_self, ds18b20_callback_t callback)
{
	p_self->convert_after_read = false;
	p_self->read_after_convert = false;
	send_command(p_self, CMD_TEMP_READ_SAFE, callback);
}

void ds18b20_convert_and_read(ds18b20_t* p_self, ds18b20_callback_t callback)
{
	p_self->convert_after_read = false;
	p_self->read_after_convert = true;
	send_command(p_self, CMD_TEMP_CONVERT, callback);
}

void ds18b20_read_and_convert(ds18b20_t* p_self, ds18b20_callback_t callback)
{
	p_self->convert_after_read = true;
	p_self->read_after_convert = false;
	send_command(p_self, CMD_TEMP_READ_SAFE, callback);
}

//------------------------------------ under the hood ---------------------------------------------
									
static void send_command(ds18b20_t* p_self, ds18b20_command_t command, ds18b20_callback_t callback)
{
	CHECK_ERROR_BOOL(p_self->command == CMD_IDLE);
	p_self->command = command;
	p_self->callback = callback;
	prepare_ow_packet(p_self, command);
	ow_enqueue_packet(&(p_self->ow_packet));
}
static void prepare_ow_packet(ds18b20_t* p_self, ds18b20_command_t command)
{
	uint8_t*		  p_buf	    = p_self->databuffer;
	ow_packet_data_t* p_data	= &(p_self->ow_packet.data);
		
	*p_buf = (uint8_t)command;
	
	if ((p_self->skip_ROM_code) || (p_self->ROM_code.family == 0))
	{
		p_self->ow_packet.ROM_command = OWM_CMD_SKIP;
	}
	else
		p_self->ow_packet.ROM_command = OWM_CMD_MATCH;
	
#ifdef OW_PARASITE_POWER_SUPPORT
	p_self->ow_packet.hold_power = p_self->parasite_powered;
#endif
	p_self->ow_packet.wait_flag = ((p_self->waiting_mode == OW_WAIT_FLAG)||(p_self->read_after_convert));
	p_self->ow_packet.delay_ms  = 0;
	
	switch (command)
	{
	case CMD_TEMP_READ_FAST:
		*p_buf = (uint8_t)CMD_TEMP_READ_SAFE;
		p_data->p_txbuf = p_buf;
		p_data->tx_count = 8;
		p_data->p_rxbuf = p_buf + 1;
		p_data->rx_count = 16;
		break;
		
	case CMD_CONFIG_READ:
		*p_buf = (uint8_t)CMD_TEMP_READ_SAFE;
		// no break
	case CMD_TEMP_READ_SAFE :
		p_data->p_txbuf = p_buf;
		p_data->tx_count = 8;
		p_data->p_rxbuf = p_buf + 1;
		p_data->rx_count = 72;
		break;
		
	case CMD_TEMP_CONVERT:
		if ((p_self->read_after_convert)
#ifdef OW_PARASITE_POWER_SUPPORT
			|| (p_self->parasite_powered)	
#endif
			|| ((p_self->waiting_mode != OW_NOT_WAIT)&&(!p_self->convert_after_read)))
		{
			p_self->ow_packet.delay_ms = waiting_delay(p_self->resolution);
		}
		p_data->p_txbuf = p_buf;
		p_data->tx_count = 8;
		p_data->rx_count = 0;
		break;
	
	case CMD_EEPROM_WRITE:
#ifdef OW_PARASITE_POWER_SUPPORT
		if(p_self->parasite_powered) 
			p_self->ow_packet.delay_ms  = 10;
#endif
		p_data->p_txbuf = p_buf;
		p_data->tx_count = 8;
		p_data->rx_count = 0;
		break;
		
	case CMD_CONFIG_WRITE:
		*(p_buf + 1) = p_self->high_tempr;
		*(p_buf + 2) = p_self->low_tempr;
		*(p_buf + 3) = (uint8_t)p_self->resolution;
		p_data->p_txbuf = p_buf;
		p_data->tx_count = 32;
		p_data->rx_count = 0;
		break;
		
#ifdef OW_PARASITE_POWER_SUPPORT
	case CMD_POWER_SUPPLY_READ:
		p_data->p_txbuf = p_buf;
		p_data->tx_count = 8;
		*(p_buf + 1)	= 0;
		//*(++p_buf) = 0;
		p_data->p_rxbuf = p_buf + 1;
		p_data->rx_count = 1;
		break;
#endif
	
	default:
		// common logic error
		HANDLE_ERROR();
	}
}

static uint32_t on_ow_transfer_completed(ow_result_t  result, ow_packet_t* p_ow_packet) 
{
	bool packet_restart	= false;
	uint8_t temprMask	= 0xF8;
	ds18b20_result_t op_result	= SUCCESS;
	ds18b20_command_t  command  = CMD_IDLE;
	
	ds18b20_t* p_self = (ds18b20_t*)(((ow_packet_t*)p_ow_packet)->p_context);
//	callback_t callbackBuf = p_self->callback;
	
	switch (result)
	{
	case OWMR_COMMUNICATION_ERROR:
		op_result = COMMUNICATION_ERROR;
		break;

	case OWMR_NO_RESPONSE:
		op_result = DEVICE_NOT_FOUND;
		break;

	case OWMR_TIME_OUT:
		switch (p_self->command)
		{
		case CMD_TEMP_CONVERT:
		case CMD_EEPROM_RECALL:
		case CMD_EEPROM_WRITE:
			op_result = WAITING_FLAG_TIME_OUT;
			break;
			
		default:
			HANDLE_ERROR();
		}
		break;
		
	case OWMR_SUCCESS:
		switch (p_self->command)
		{
		case CMD_CONFIG_WRITE:
			LOG_PRINTF("\n         - Config written to remote device and verified");
			p_self->scratchpad_writed = true;
			command  = CMD_CONFIG_READ;
			packet_restart = true;
			break;
		case CMD_CONFIG_READ:
			LOG_PRINTF("\n         - Scratchpad readed "); 
			log_hex(p_self->databuffer + 1, 8);
			
			if (!checkcrc8(p_self->databuffer[9], p_self->databuffer + 1, 8))
			{
				LOG_PRINTF("\n         - ERROR! Crc8 check faled!"); 
				op_result = COMMUNICATION_ERROR;
			}
			else if (!p_self->databuffer[5])
			{
				LOG_PRINTF("\n         - ERROR! Device not found!"); 
				op_result = DEVICE_NOT_FOUND;
			}
			else if ((p_self->databuffer[5] != (uint8_t)p_self->resolution)
#ifdef DS18B20_ALARM_TEMPR_SUPPORT
				|| (p_self->databuffer[3] != (uint8_t)p_self->high_tempr)
				|| (p_self->databuffer[4] != (uint8_t)p_self->low_tempr)
#endif
				)
			{
				if (p_self->scratchpad_writed)
				{
					LOG_PRINTF("\n         - ERROR! Config rewriting faled!"); 
					op_result = CONFIG_WRITING_ERROR;
				}
				else
				{
			        LOG_PRINTF("\n         - Remote config not matches current settings. Trying to rewrite.");
					command  = CMD_CONFIG_WRITE;
					packet_restart = true;
				}
			}
			else if (p_self->scratchpad_writed)
			{
				command  = CMD_EEPROM_WRITE;
				packet_restart = true;
			}
			else
			{
				LOG_PRINTF("\n         - Remote config matches current settings.");
			}
			break;

		case CMD_TEMP_CONVERT:
			if (p_self->read_after_convert)
			{
//				LOG_PRINTF("\n         - Reading temperature after connversion completed.");
				command  = CMD_TEMP_READ_SAFE;
				packet_restart = true;
			}
			break;
			
		case CMD_EEPROM_WRITE:
			LOG_PRINTF("\n         - Config in remote device saved to flash.");
			break;
			
		case CMD_TEMP_READ_SAFE:
			if (!checkcrc8(p_self->databuffer[9], p_self->databuffer + 1, 8))
			{
				op_result = COMMUNICATION_ERROR;
				break;
			}
			else if (!p_self->databuffer[5])
			{
				op_result = DEVICE_NOT_FOUND;
				break;
			}
			else if (p_self->databuffer[5] != (uint8_t)p_self->resolution) 
			{
				op_result = COMMUNICATION_ERROR;
				break;
			}
			// no break 
		case CMD_TEMP_READ_FAST:
			if (p_self->resolution == RES_12_BIT)
				temprMask = 0xFF;
			else if (p_self->resolution == RES_11_BIT)
				temprMask = 0xFE;
			else if (p_self->resolution == RES_10_BIT)
				temprMask = 0xFC;
			p_self->temperature = 
				(((int16_t) p_self->databuffer[2]) << 8) | 
				((int16_t)(p_self->databuffer[1]&temprMask));
			if (p_self->convert_after_read)
			{
				command  = CMD_TEMP_CONVERT;
				packet_restart = true;
			}
			break;
		
#ifdef OW_PARASITE_POWER_SUPPORT
		case CMD_POWER_SUPPLY_READ:
			p_self->parasite_powered = (*(p_self->databuffer + 1) == 0);
			command  = CMD_CONFIG_READ;
			packet_restart = true;
			LOG_PRINTF("\n         - Remote power mode read. Parasite powered = %i", p_self->parasite_powered);
			break;
#endif

		default:
			// common logic error
			HANDLE_ERROR();
		}
		if (packet_restart)
		{
			prepare_ow_packet(p_self, command);
			op_result = PENDING;
		}
		break;
	default:
		// common logic error
		HANDLE_ERROR();
	}
	
	p_self->result = op_result;
	p_self->command  = command;

	if (!packet_restart && p_self->callback)
	{
		p_self->callback(p_self);
	}
	
	return packet_restart;
}