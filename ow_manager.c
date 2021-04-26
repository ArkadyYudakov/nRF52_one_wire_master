
#include <stdbool.h>
#include <stdint.h>

// platform dependant
#include "app_util_platform.h"	
#include "app_util.h"
#define  _CRITICAL_REGION_ENTER()     CRITICAL_REGION_ENTER()
#define  _CRITICAL_REGION_EXIT()      CRITICAL_REGION_EXIT()
#define  _IS_POWER_OF_TWO(int_value)  IS_POWER_OF_TWO(int_value)
#include "app_error.h"
#define  CHECK_ERROR_BOOL( bool_expresion ) APP_ERROR_CHECK_BOOL( bool_expresion )
// end of platform dependant section

#include "ow_master.h"
#include "ow_manager.h"

typedef enum
{
	OWMM_STATE_NOT_INITIALIZED,
	OWMM_STATE_IDLE,
	OWMM_STATE_BUSY
} owmm_state_t;

static owmm_state_t		  m_manager_state = OWMM_STATE_NOT_INITIALIZED;

static uint16_t           fifo_size_mask; /**< Read/write index mask. Also used for size checking. */
static volatile uint32_t  fifo_read_pos;  /**< Next read position in the FIFO buffer.              */
static volatile uint32_t  fifo_write_pos; /**< Next write position in the FIFO buffer.             */

static ow_packet_t*		  fifo_buf[OW_MANAGER_FIFO_SIZE];

// forward declaration
static void ow_manager_callback(ow_result_t  result, ow_packet_t* p_ow_packet);

// module initialization
// sequential initializing of ow_master and ow_master_hal modules performs
void ow_manager_initialize(void)
{
	ow_master_initialize(ow_manager_callback);
	
	CHECK_ERROR_BOOL(_IS_POWER_OF_TWO(OW_MANAGER_FIFO_SIZE));
	fifo_size_mask     = OW_MANAGER_FIFO_SIZE - 1;
	fifo_read_pos      = 0;
	fifo_write_pos     = 0;

	m_manager_state = OWMM_STATE_IDLE;
}

// module deinitialization
// If success, 0 returns. If driver is busy, 1 returns.
uint32_t ow_manager_uninitialize(void)
{
	uint32_t result = 1;
	
	_CRITICAL_REGION_ENTER()
	if((m_manager_state == OWMM_STATE_IDLE)&&(ow_master_uninitialize() == 0))
	{
		m_manager_state = OWMM_STATE_NOT_INITIALIZED;
		result = 0;
	}
	_CRITICAL_REGION_EXIT()
	return result;
}

// utility function for fifo buffer handling
static __INLINE uint32_t fifo_length(void)
{
	if (fifo_write_pos < fifo_read_pos)
		return (fifo_write_pos + ~fifo_read_pos + 1);	
	else
		return (fifo_write_pos - fifo_read_pos);
}

// Thread safe way to perform 1-wire transaction. Application modules puts packets in quew.
// 1-wire manager sequentially takes packets from queue and pass for execution to ow_master
// After packet transfer completion, manager invoks packet callback. If callback function
// returns not 0, manager reexecutes transferring. So, application module can perform
// continuous transferring of several packets modifying current packet and returning 1 from 
// packet callback.  
void ow_enqueue_packet(ow_packet_t* p_ow_packet)
{
	CHECK_ERROR_BOOL(m_manager_state != OWMM_STATE_NOT_INITIALIZED);
	
	bool	buffer_overflow	= false;
	bool	launch_packet	= false;
	
	// thread safe buffer handling in critical section	
	_CRITICAL_REGION_ENTER()
	if(m_manager_state == OWMM_STATE_IDLE)
	{
		// manager in idle, buffer is empty. Pass packet for processing directly
		m_manager_state = OWMM_STATE_BUSY;
		launch_packet = true;
	}
	else
	{
		// check buffer overflow
		if (fifo_length() > fifo_size_mask)
			buffer_overflow = true;
		else 
		{
			// put packet into buffer
			fifo_buf[fifo_write_pos&fifo_size_mask] = p_ow_packet;
			++fifo_write_pos;			
		}
	}
	_CRITICAL_REGION_EXIT()
	// after leaving of critical section check errors and send packet for processing
	APP_ERROR_CHECK_BOOL(!buffer_overflow);
	if(launch_packet) ow_process_packet(p_ow_packet);
}
	
// Callback. Registered in ow_master module. 
// Invoked after packet processing completion.
void ow_manager_callback(ow_result_t  result, ow_packet_t* p_ow_packet)
{
	ow_packet_t*		p_next_packet  = NULL;
	
	// invoke packet callback if defined. If not 0 returned, send packet for processing again
	if ((p_ow_packet->callback)&&(p_ow_packet->callback(result, p_ow_packet) != 0))
	{
		ow_process_packet(p_ow_packet);
	}
	else
	{
		// thread safe buffer handling in critical section 
		_CRITICAL_REGION_ENTER()
		if(fifo_write_pos != fifo_read_pos)
		{
			// buffer is not empty. Take next packet
			p_next_packet	 = fifo_buf[fifo_read_pos&fifo_size_mask];
			++fifo_read_pos;
		}
		else
			// buffer is empty. Go to idle state.
			m_manager_state = OWMM_STATE_IDLE;	
		_CRITICAL_REGION_EXIT()
		// after leaving of critical section send packet for processing
		if(p_next_packet)
		{
			ow_process_packet(p_next_packet);
		}	
	}
}