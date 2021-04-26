
#include <stdbool.h>
#include <stdint.h>

// platform dependant
#include "app_error.h"
#define  CHECK_ERROR_BOOL( bool_expresion ) APP_ERROR_CHECK_BOOL( bool_expresion )
#define  HANDLE_ERROR() APP_ERROR_CHECK_BOOL( false )
// end of platform dependant section

#include "ow_master_hal.h"	
#include "ow_master.h"

typedef enum
{
	OWM_STATE_IDLE,
	OWM_STATE_RESET,
	OWM_STATE_COMMAND,
	OWM_STATE_ROM,
	OWM_STATE_DATA,
	OWM_STATE_HOLD_POWER,
	OWM_STATE_WAIT_FLAG,
#ifdef OW_ROM_SEARCH_SUPPORT
	OWM_STATE_SEARCH_POLL,
	OWM_STATE_SEARCH_DIR,
#endif
	OWM_WAIT_FLAG,
	OWM_DELAY,
#if defined OW_PARASITE_POWER_SUPPORT
	OWM_POWER_HOLD,
#endif	
	OWM_STATE_NOT_INITIALIZED
} owm_state_t;

static owm_state_t   m_ow_master_state = OWM_STATE_NOT_INITIALIZED;
static ow_packet_t*    m_p_ow_packet;
static uint8_t       m_command_buf;

static ow_master_callback_t  m_callback;

// Default callback. Used if no other registered. 
// Driver nvoks packet callback if defined at the end of 1-wire packet exchange procedure.
// If callback function returns not 0, driver retransmits packed again.
static void owm_default_callback(ow_result_t  result, ow_packet_t* p_packet)
{
	if ((p_packet->callback)&&(p_packet->callback(result, p_packet) != 0))
	{
		ow_process_packet(p_packet);
	}
}	

// forward declaration.
static void owm_fsm(owmh_callback_result_t result);

// 1-Wire master driver initialization. 
// Registering callback of higher level module.
// Callback invoks after packet transfer completion.
void ow_master_initialize(ow_master_callback_t callback)
{
	CHECK_ERROR_BOOL(m_ow_master_state == OWM_STATE_NOT_INITIALIZED);
	
	if (callback)
		m_callback = callback;
	else
		m_callback = owm_default_callback;
	
	owm_hal_initialize(owm_fsm);
	m_ow_master_state = OWM_STATE_IDLE;
}

// 1-Wire master driver uninitialization. 
// If success, returns 0. If driver is busy, returns 1.
uint32_t ow_master_uninitialize()
{
	if ((m_ow_master_state == OWM_STATE_IDLE)&&(owm_hal_uninitialize() == 0))
	{
		m_ow_master_state = OWM_STATE_NOT_INITIALIZED;
		return 0;
	}
	else 
		return 1;
}

// 1-Wire packet transferring. 
void ow_process_packet(ow_packet_t* p_ow_packet)
{
	// Check, if previos process completed
	CHECK_ERROR_BOOL(m_ow_master_state == OWM_STATE_IDLE);
	// Set channal
#if (defined (OW_MULTI_CHANNEL))
	ow_set_channel(p_ow_packet->channel);
#endif
	// Initialise common parameters
	m_p_ow_packet = p_ow_packet;
	m_ow_master_state = OWM_STATE_RESET;
	// Call HAL primitive
	owmh_reset();
}

// Utility function
static void ow_packet_terminate(ow_result_t result)
{
	m_ow_master_state = OWM_STATE_IDLE;
	m_callback(result, (void*)m_p_ow_packet);
}

// Main state machine procedure. Registered as callback in 1-Wire HAL module.
// Invoked after completion of the 1-Wire HAL primitive
static void owm_fsm(owmh_callback_result_t result)
{
#ifdef OW_ROM_SEARCH_SUPPORT
	// Static variables for searching process
	static uint8_t crc8;
	static uint8_t bit_number;
	static uint8_t byte_index;
	static uint8_t byte_mask;
	static uint8_t last_zero;
	static uint8_t last_family_zero;
	static uint8_t direction_bit;
	bool critical_consistency_error;
#endif
	switch (m_ow_master_state)
	{
//----------------------------------------------------------------------------------------------------------------	
// on completion of RESET fase. ( Transfer initiation, detecting devices presence ) 
	case OWM_STATE_RESET:
		if (result == OWMHCR_RESET_OK)
		{
			// set COMMAND state, transfer 1 WIRE ROM COMMAND
			m_ow_master_state = OWM_STATE_COMMAND;
			m_command_buf = (uint8_t)m_p_ow_packet->ROM_command;
			owmh_sequence(&m_command_buf, NULL, 8, 0);
		}
		else if (result == OWMHCR_RESET_NO_RESPONCE)
			// no devices on bus
			ow_packet_terminate(OWMR_NO_RESPONSE);
		else if (result == OWMHCR_ERROR)
			// incorrect signal timing on bus
			ow_packet_terminate(OWMR_COMMUNICATION_ERROR);
		else
			// common logic error
			HANDLE_ERROR();
		break;
		
//----------------------------------------------------------------------------------------------------------------	
// on completion of ROM command fase (first byte after reset).
	case OWM_STATE_COMMAND:
		if (result == OWMHCR_PACKET_OK)
		{
			// prepare and transfer packet depanding on ROM command
			switch (m_p_ow_packet->ROM_command)
			{
			case OWM_CMD_READ:
				// read 8 bit ROM adress
				m_ow_master_state = OWM_STATE_DATA;
				owmh_sequence(NULL, (uint8_t*)(m_p_ow_packet->p_ROM), 0, 64);
				break;
		
			case OWM_CMD_SKIP:
				// skip ROM address transmitting. Send data immediately
				m_ow_master_state = OWM_STATE_DATA;
				owmh_sequence(m_p_ow_packet->data.p_txbuf,
					m_p_ow_packet->data.p_rxbuf, 
					m_p_ow_packet->data.tx_count,
					m_p_ow_packet->data.rx_count);
					break;
		
			case OWM_CMD_MATCH:
				// transfer 8 bit ROM address
				m_ow_master_state = OWM_STATE_ROM;
				owmh_sequence((uint8_t*)(m_p_ow_packet->p_ROM), NULL, 64, 0);
				break;
		
#ifdef OW_ROM_SEARCH_SUPPORT
			case OWM_CMD_SEARCH:
			case OWM_CMD_ALARM_SEARCH:
				// Check, if last device was found
				if(m_p_ow_packet->search.last_device)
					ow_packet_terminate(OWMR_NOT_FOUND);
				else
				{
					// search route initialization
					crc8	   = 0;
					bit_number = 1;
					byte_index = 0;
					byte_mask  = 1;
					last_zero  = 0;
					last_family_zero  = 0;
					m_p_ow_packet->search.consistency_fault = false;
					// polling of direct and complement bits at next position (first in this case) 
					m_ow_master_state = OWM_STATE_SEARCH_POLL;
					owmh_poll();
				}
				break;
#endif
			default: 
				// common logic error
				HANDLE_ERROR();
			}
		}
		else if(result == OWMHCR_ERROR)
			// incorrect signal timing on bus
			ow_packet_terminate(OWMR_COMMUNICATION_ERROR);
		else
			// common logic error
			HANDLE_ERROR();
		break;
//----------------------------------------------------------------------------------------------------------------	
// on completion of ROM address trnsmitting fase.
	case OWM_STATE_ROM :
		if (result == OWMHCR_PACKET_OK)
		{
			// transfer data
			m_ow_master_state = OWM_STATE_DATA;
			owmh_sequence(m_p_ow_packet->data.p_txbuf,
				m_p_ow_packet->data.p_rxbuf, 
				m_p_ow_packet->data.tx_count,
				m_p_ow_packet->data.rx_count);
}
		else if (result == OWMHCR_ERROR)
			// incorrect signal timing on bus
			ow_packet_terminate(OWMR_COMMUNICATION_ERROR);
		else
			// common logic error
			HANDLE_ERROR();
		break;
//----------------------------------------------------------------------------------------------------------------	
// on completion of data transferring fase.
	case OWM_STATE_DATA :
		if (result == OWMHCR_PACKET_OK)
		{
			// finishing of packet processing depanding on ROM command
			switch (m_p_ow_packet->ROM_command)
			{
			case OWM_CMD_READ:
				// terminate after ROM reading
				ow_packet_terminate(OWMR_SUCCESS);
				break;
			case OWM_CMD_SKIP:
			case OWM_CMD_MATCH:
				// finalizing procedures - wate flag, hold power, delay
				if(m_p_ow_packet->delay_ms > 0)
				{
#if defined OW_PARASITE_POWER_SUPPORT
					if (m_p_ow_packet->hold_power)
					{
						m_ow_master_state = OWM_POWER_HOLD;
						owmh_hold_power(m_p_ow_packet->delay_ms);
					}
					else if (m_p_ow_packet->wait_flag)
#else
					if (m_p_ow_packet->wait_flag)
#endif
					{
						m_ow_master_state = OWM_WAIT_FLAG;
						owmh_wait_flag(m_p_ow_packet->delay_ms);
					}
					else
					{
						m_ow_master_state = OWM_DELAY;
						owmh_delay(m_p_ow_packet->delay_ms);
					}
				}
				else
					// no aditional prosedures. Terminate transfer
					ow_packet_terminate(OWMR_SUCCESS);
				break;
			
			default: 
				// common logic error
				HANDLE_ERROR();
			}
		}
		else if (result == OWMHCR_ERROR)
			// incorrect signal timing on bus
			ow_packet_terminate(OWMR_COMMUNICATION_ERROR);
		else
			// common logic error
			HANDLE_ERROR();
		break;

#if defined OW_PARASITE_POWER_SUPPORT
//----------------------------------------------------------------------------------------------------------------	
// on completion of holding forced positive state on 1 wire bus data line
	case OWM_POWER_HOLD :
		if(result == OWMHCR_WAIT_OK)
			// no errors
			ow_packet_terminate(OWMR_SUCCESS);
		else if (result == OWMHCR_ERROR)
			// incorrect signal timing on bus
			ow_packet_terminate(OWMR_COMMUNICATION_ERROR);
		else
			// common logic error
			HANDLE_ERROR();
		break;
#endif
//----------------------------------------------------------------------------------------------------------------	
// after flag reading procedure.   
	case OWM_WAIT_FLAG :
		if(result == OWMHCR_FLAG_OK)
			// flag resived before time out
			ow_packet_terminate(OWMR_SUCCESS);
		else if(result == OWMHCR_TIME_OUT)
			// flag not resived before time out
			ow_packet_terminate(OWMR_TIME_OUT);
		else if (result == OWMHCR_ERROR)
			// incorrect signal timing on bus
			ow_packet_terminate(OWMR_COMMUNICATION_ERROR);
		else
			// common logic error
			HANDLE_ERROR();
		break;
//----------------------------------------------------------------------------------------------------------------	
// after simple delay procedure.
	case OWM_DELAY :
		if(result == OWMHCR_WAIT_OK)
			// no errors
			ow_packet_terminate(OWMR_SUCCESS);
		else if (result == OWMHCR_ERROR)
			// incorrect signal timing on bus
			ow_packet_terminate(OWMR_COMMUNICATION_ERROR);
		else
			// common logic error
			HANDLE_ERROR();
		break;
#ifdef OW_ROM_SEARCH_SUPPORT
//----------------------------------------------------------------------------------------------------------------	
// processing of next pair of complement bits in searching process   
	case OWM_STATE_SEARCH_POLL :
		critical_consistency_error = false;
		if(result == OWMHCR_POLL_01)
		{
		    //case 1:
			// No discrepancy. Direction = polling bit, but check concistency
			direction_bit = 1;
			if (((byte_mask & m_p_ow_packet->p_ROM[byte_index]) == 0) 
				                 && (bit_number < m_p_ow_packet->search.last_discrepancy)) // broken consistency
				{
					m_p_ow_packet->search.consistency_fault = true;
					m_p_ow_packet->search.last_discrepancy = bit_number;   // Reset last_discrepancy to current position
				}
		}
		else if(result == OWMHCR_POLL_10)
		{
		    //case 2:
			// No discrepancy. Direction = polling bit, but check concistency
			direction_bit = 0;
			if (((byte_mask & m_p_ow_packet->p_ROM[byte_index]) == 1) 
				                 && (bit_number < m_p_ow_packet->search.last_discrepancy)) // broken consistency
				{
					m_p_ow_packet->search.consistency_fault = true;
					critical_consistency_error = true;
					//m_p_ow_packet->result = OWMR_SEARCH_CONSISTENCY_FAULT;
				}
		}
		else if(result == OWMHCR_POLL_11)
		{
			//case 3:
			if ((bit_number == 1)&&(m_p_ow_packet->ROM_command == OWM_CMD_ALARM_SEARCH))
				// No response at first polling in alarm searching 
				ow_packet_terminate(OWMR_NOT_FOUND);  //OWMR_NO_ALARMED_DEVICES
			else
			{
				// No response at any other cases. Wrong situation. Termination of search route
				m_p_ow_packet->search.consistency_fault = true;
				critical_consistency_error = true;
				//ow_packet_terminate(OWMR_SEARCH_CONSISTENCY_FAULT);
				break;
			}
		}
		else if(result == OWMHCR_POLL_00)
		{
		    //case 0:
			// Discrepancy detected.
			if(bit_number == m_p_ow_packet->search.last_discrepancy)
			{
				// Last discrepancy position. Direction = 1	
				direction_bit = 1;
			} 
			else if(bit_number < m_p_ow_packet->search.last_discrepancy)
			{
				// Index less than last discrepancy. Get direction from saved ROM					
				direction_bit = byte_mask & m_p_ow_packet->p_ROM[byte_index];
			} 
			else
			{
				// Index after last discrepancy. Direction = 0				
				direction_bit = 0;
			}
			if (!direction_bit)
			{
				last_zero = bit_number;  // Save last turn to direction 0
				if (!byte_index)
					last_family_zero = bit_number; // Save last turn to direction 0 in device family code (first byte in ROM)
			}
		}
		else
		{
			// common logic error
			HANDLE_ERROR();
			break;
		} 

		if (critical_consistency_error) // Termination of search route
			ow_packet_terminate(OWMR_SEARCH_CONSISTENCY_FAULT);
		else // normal workflow
		{
			// Save direction in ROM
			if(direction_bit)
				m_p_ow_packet->p_ROM[byte_index] |= byte_mask;   // Set bit in ROM
			else
				m_p_ow_packet->p_ROM[byte_index] &= (~byte_mask);   // Clear bit in ROM
			
			// Transmit direction bit
			m_ow_master_state = OWM_STATE_SEARCH_DIR;
			owmh_write(direction_bit);
		
			// Last bit in byte
			if(byte_mask == 0x80)
			{
				// Calculate and save CRC
				docrc8(&crc8, m_p_ow_packet->p_ROM[byte_index]);
				// Reset mask, shift byte index
				byte_mask = 0x01;
				++byte_index;
			}
			else 
				byte_mask <<= 1;  // Shift mask

			// Shift bit index, check, if rout is ended
			if(++bit_number > 64)
			{
				// Search rout is ended
				m_p_ow_packet->search.last_family_discrepancy = last_family_zero;
				m_p_ow_packet->search.last_discrepancy = last_zero;
				if(last_zero  == 0) // Last device address was routed                                                                                                                                            
					m_p_ow_packet->search.last_device = true;
			}
		}
		break;
//----------------------------------------------------------------------------------------------------------------	
// after sending next bit of ROM address in searching process   
	case OWM_STATE_SEARCH_DIR :
		if(bit_number > 64) // all bits are routed
		{
			// Check CRC
			if(!crc8)
				// Wrong CRC
				ow_packet_terminate(OWMR_COMMUNICATION_ERROR);
			else
				// Finalise search rout
				ow_packet_terminate(OWMR_SUCCESS);
		}
		else
		{
			// Poll next complement bits
			m_ow_master_state = OWM_STATE_SEARCH_POLL;
			owmh_poll();
		}
		break;
		//----------------------------------------------------------------------------------------------------------------	
#endif
	default : 
		// common logic error
		HANDLE_ERROR();
	}
}

// Utility functions for crc8 calculation
static unsigned char dscrc_table[] = 
{
	0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
	157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
	35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
	190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
	70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
	219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
	101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
	248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
	140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
	17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80, 
	175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
	50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
	202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
	87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
	233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
	116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
};

void docrc8(uint8_t* crc, uint8_t value)
{
	*crc = dscrc_table[*crc ^ value];
}
uint8_t crc8(uint8_t crc, uint8_t value)
{
	return dscrc_table[crc ^ value];
}

bool checkcrc8(uint8_t crc, uint8_t* buf, uint32_t count)
{
	uint8_t crc8 = 0;
	for (uint32_t k = 0; k < count; ++k)
		docrc8(&crc8, buf[k]);
	return (crc8 == crc);
}
