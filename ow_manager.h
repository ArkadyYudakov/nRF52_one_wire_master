#ifndef	OW_MASTER_MANAGER_H__
#define OW_MASTER_MANAGER_H__

#ifdef __cplusplus
extern "C" {
#endif
	
#include "ow_config.h"	
#include "ow_packet.h"

#define OW_MANAGER_FIFO_SIZE 16
			
/**
 * @brief 1 WIRE manager initialization. 
 * 
 * Sequential initializing of ow_master and HAL modules performs
 * in the process
 */
void ow_manager_initialize(void);
	
/**
 * @brief  1-Wire manager uninitialization.
 * 
 * @retval 0 success.
 * @retval 1 driver is busy.
*/
uint32_t ow_manager_uninitialize(void);

// Thread safe way to perform 1-wire transaction. Application modules puts packets in queue.
// 1-wire manager sequentially takes packets from queue and pass for execution to ow_master
// After packet processing completion, manager invoks packet callback. If callback function
// returns not 0, manager reexecutes transferring. So, application module can perform
// continuous transferring of several packets modifying current packet and returning 1 from 
// packet callback.  
void ow_enqueue_packet(ow_packet_t* p_ow_packet);

#ifdef __cplusplus
}
#endif

#endif // OW_MASTER_MANAGER_H__
