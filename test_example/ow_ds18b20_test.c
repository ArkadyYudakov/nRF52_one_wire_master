#include <stdbool.h>
#include <stdint.h>

#include "boards.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"	
#include "nrf_pwr_mgmt.h"
	
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

void start_ow_test();

int main(void)
{
	APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
	NRF_LOG_DEFAULT_BACKENDS_INIT();
	
    APP_ERROR_CHECK(nrf_drv_clock_init());
    nrf_drv_clock_lfclk_request(NULL);
	APP_ERROR_CHECK(app_timer_init());

	bsp_board_init(BSP_INIT_LEDS);

	NRF_LOG_RAW_INFO("\n\n************************************************************************");
	NRF_LOG_RAW_INFO("\n>>> One Wire library example/test started.");
	
	start_ow_test();

	NRF_LOG_RAW_INFO("\n>>> Main loop entered (log processing, sleep mode indication on leds).");
	
	for (;;)
	{
		do {} while (NRF_LOG_PROCESS());

		bsp_board_led_off(0); 
		bsp_board_led_on(1); 
			
		nrf_pwr_mgmt_run();
		
		bsp_board_led_on(0); 
		bsp_board_led_off(1); 
	}
}
