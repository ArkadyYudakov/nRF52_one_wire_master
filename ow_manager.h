#ifndef	OW_MASTER_MANAGER_H__
#define OW_MASTER_MANAGER_H__

#ifdef __cplusplus
extern "C" {
#endif
	
#include "ow_config.h"	
#include "ow_packet.h"

#define OW_MANAGER_FIFO_SIZE 16
			
// 1-WIRE manager initialization
// sequential initializing of ow_master and ow_master_hal modules performs
void ow_manager_initialize(void);
	
// 1-WIRE master deinitialization
// If success, 0 returns. If driver is busy, 1 returns.
uint32_t ow_manager_uninitialize(void);

// Thread safe way to perform 1-wire transaction. Application modules puts packets in quew.
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
