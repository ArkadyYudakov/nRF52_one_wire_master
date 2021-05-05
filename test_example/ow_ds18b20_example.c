#include <stdbool.h>
#include <stdint.h>

#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"	
	
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "bc_scheduler.h"
#include "ow_manager.h"


const uint8_t leds_list[LEDS_NUMBER] = LEDS_LIST;

void start_ow_discovering();

int main(void)
{
	APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
	NRF_LOG_DEFAULT_BACKENDS_INIT();
	
    APP_ERROR_CHECK(nrf_drv_clock_init());
    nrf_drv_clock_lfclk_request(NULL);
	APP_ERROR_CHECK(app_timer_init());
	
	ow_manager_initialize();
	bc_scheduler_initialize();

	NRF_LOG_RAW_INFO("\n\n************************************************************************");
	NRF_LOG_RAW_INFO("\n>>> Example application started.");
	NRF_LOG_FLUSH();
	
	NRF_LOG_RAW_INFO("\n>>> One wire discovering started.");
	start_ow_discovering();
	
	LEDS_CONFIGURE(LEDS_MASK);

//	for (;;)
//	{
//		NRF_LOG_PROCESS();
//		for (int i = 0; i < LEDS_NUMBER; i++)
//		{
//			LEDS_INVERT(1 << leds_list[i]);
//			nrf_delay_ms(500);
//		}
//	}

	NRF_LOG_RAW_INFO("\n>>> Scheduler run.");
	bc_scheduler_run();
}
