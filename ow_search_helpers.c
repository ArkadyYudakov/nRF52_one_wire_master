
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "ow_manager.h"

void ow_read_ROM_code(ow_packet_t* p_ow_packet)
{
	p_ow_packet->ROM_command = OWM_CMD_READ;
	// Start OW transfer
	ow_enqueue_packet(p_ow_packet);
}

#ifdef OW_ROM_SEARCH_SUPPORT

void ow_search_next(ow_packet_t* p_ow_packet, bool alarm)
{
	p_ow_packet->ROM_command = (alarm ? OWM_CMD_ALARM_SEARCH : OWM_CMD_SEARCH);
	// Start OW transfer
	ow_enqueue_packet(p_ow_packet);
}

void ow_search_first(ow_packet_t* p_ow_packet, bool alarm)
{
	// Initialise search parameters
	memset(p_ow_packet->p_ROM_code, 0, sizeof(ROM_code_t));
	p_ow_packet->search.last_device = false;
	p_ow_packet->search.last_discrepancy = 0;
	p_ow_packet->search.last_family_discrepancy = 0;
	// Continue with search
	ow_search_next(p_ow_packet, alarm);
}

void ow_search_first_in_family(ow_packet_t* p_ow_packet, uint8_t family_code, bool alarm)
{
	// Initialise search parameters
	memset(p_ow_packet->p_ROM_code, 0, sizeof(ROM_code_t));
	p_ow_packet->p_ROM_code->raw[0] = family_code;
	p_ow_packet->search.last_device = false;
	p_ow_packet->search.last_discrepancy = 65;
	p_ow_packet->search.last_family_discrepancy = 0;
	// Continue with search
	ow_search_next(p_ow_packet, alarm);
}

void ow_search_verify(ow_packet_t* p_ow_packet)
{
	// Initialise search parameters
	p_ow_packet->search.last_device = false;
	p_ow_packet->search.last_discrepancy = 65;
	ow_search_next(p_ow_packet, false);
}

void ow_search_next_family(ow_packet_t* p_ow_packet, bool alarm)
{
	// Initialise search parameters
	p_ow_packet->search.last_discrepancy = p_ow_packet->search.last_family_discrepancy;
	// Continue with search
	ow_search_next(p_ow_packet, alarm);
}
#endif
