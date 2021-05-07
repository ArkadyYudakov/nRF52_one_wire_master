
#include <nrfx.h>
#include "nrf_drv_gpiote.h"
#include <nrfx_gpiote.h>
#include "prs/nrfx_prs.h"
#include <hal/nrf_gpio.h>
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"

#include "app_error.h"

#include "ow_master_hal.h"

// OW master HAL states
typedef enum
{
	OWMHS_IDLE,

	OWMHS_RESET,
	OWMHS_READ,
	OWMHS_WRITE0,
	OWMHS_WRITE1,

	OWMHS_SEQUENCE,

	OWMHS_READ_FLAG,
	OWMHS_FLAG_PAUSE,
	OWMHS_DELAY,
#ifdef OW_PARASITE_POWER_SUPPORT
	OWMHS_POWER_HOLD,
#endif

	OWMHS_NOT_INITIALIZED
} owmh_state_t;

#define DELAY_MKS(delay_microseconds) ((delay_microseconds)*16)

#define OW_READ_PULSE			DELAY_MKS(5)
#define OW_WRITE1_PULSE			OW_READ_PULSE 
#define OW_WRITE0_PULSE			DELAY_MKS(60)
#define OW_RESET_PULSE			DELAY_MKS(600)

#define OW_WRITE1_PULSE_TOLERANCE	DELAY_MKS(2)
#define OW_WRITE0_PULSE_TOLERANCE	DELAY_MKS(2)

#define OW_WRITE_TIMESLOT_DELAY	DELAY_MKS(70)
#define OW_READ_TIMESLOT_DELAY	DELAY_MKS(100)
#define OW_RESET_DELAY			DELAY_MKS(600+300)

#define OW_MILLISECOND_DELAY	DELAY_MKS(1000)
#define OW_FLAG_PAUSE_DELAY	(OW_MILLISECOND_DELAY - OW_READ_TIMESLOT_DELAY)

#define OW_READ1_BOUND			DELAY_MKS(10)
#define OW_READ0_BOUND			DELAY_MKS(15)
#define OW_PRESENCE_BOUND		DELAY_MKS(600+60)

static const nrf_drv_timer_t ow_timer = NRF_DRV_TIMER_INSTANCE(OW_TIMER_INSTANCE);

static nrf_ppi_channel_t m_ppi_channel_capture;
static nrf_ppi_channel_t m_ppi_channel_strobe_end;

static uint32_t         m_out_pin;
static uint32_t         m_in_pin;
#ifdef OW_PARASITE_POWER_SUPPORT
static uint32_t         m_pwr_pin;
#endif

static owmh_callback_t  m_callback;

#ifdef OW_MULTI_CHANNEL 
typedef struct
{
	uint32_t tx_pin;
	uint32_t rx_pin;
#if ((defined (OW_PARASITE_POWER_SUPPORT)) && (defined (OW_DEDICATED_POWER_PIN)))
	uint32_t pwr_pin;
#endif
} ow_channal_rec_t;

static const ow_channal_rec_t ow_pins[OW_CHANNEL_COUNT] = OW_PINS_ARRAY;
#endif // (defined (OW_MULTI_CHANNEL))

static volatile owmh_state_t m_state = OWMHS_NOT_INITIALIZED; 
static uint8_t    m_tx_bit;
static uint8_t*   m_p_tx_buf;
static uint8_t*   m_p_rx_buf;
static uint8_t    m_tx_count;
static uint8_t    m_rx_count;
static uint8_t    m_byte_mask;
static uint16_t   m_delay_counter;

static void ow_timer_event_handler(nrf_timer_event_t event_type, void * p_context);

static void owmh_handler(void);

static const nrf_drv_gpiote_out_config_t ow_gpiote_out_config =
{
	.action = NRF_GPIOTE_POLARITY_LOTOHI,
	.init_state = NRF_GPIOTE_INITIAL_VALUE_HIGH,
	.task_pin = true,
};

static const nrf_drv_gpiote_in_config_t ow_gpiote_in_config = 
{
	.is_watcher = false,
	.hi_accuracy = true,
	.pull = NRF_GPIO_PIN_NOPULL,
	.sense = NRF_GPIOTE_POLARITY_LOTOHI,
	.skip_gpio_setup = true
};

static const nrf_drv_timer_config_t ow_timer_cfg =
{
	.frequency = NRF_TIMER_FREQ_16MHz,
	.mode = NRF_TIMER_MODE_TIMER,
	.bit_width = NRF_TIMER_BIT_WIDTH_16,
	.interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
	.p_context = NULL
};

void owm_hal_initialize(owmh_callback_t callback)
{
	if (m_state != OWMHS_NOT_INITIALIZED)
    {
	    APP_ERROR_CHECK(NRFX_ERROR_INVALID_STATE);
    }

	m_callback = callback;
	
	// init GPIO
#ifdef OW_MULTI_CHANNEL
	m_out_pin = ow_pins[0].tx_pin;
	m_in_pin  = ow_pins[0].rx_pin;
#if ((defined (OW_PARASITE_POWER_SUPPORT)) && (defined (OW_DEDICATED_POWER_PIN)))
	m_pwr_pin = ow_pins[0].pwr_pin;
#endif
	
	for(uint8_t k = 0 ; k < OW_CHANNEL_COUNT ; ++k)
	{
		nrf_gpio_cfg_input(ow_pins[k].rx_pin, NRF_GPIO_PIN_NOPULL);
		nrf_gpio_pin_set(ow_pins[k].tx_pin);
		nrf_gpio_cfg(ow_pins[k].tx_pin,
			NRF_GPIO_PIN_DIR_OUTPUT,
			NRF_GPIO_PIN_INPUT_DISCONNECT,
			NRF_GPIO_PIN_NOPULL,
			NRF_GPIO_PIN_S0D1,
			NRF_GPIO_PIN_NOSENSE);

#if ((defined (OW_PARASITE_POWER_SUPPORT)) && (defined (OW_DEDICATED_POWER_PIN)))
#if (defined (OW_POWER_PIN_ACTIVE_STATE)&&(OW_POWER_PIN_ACTIVE_STATE == 1))
		nrf_gpio_pin_clear(ow_pins[k].pwr_pin);
#else
		nrf_gpio_pin_set(ow_pins[k].pwr_pin);
#endif
		nrf_gpio_cfg(ow_pins[k].pwr_pin,
			NRF_GPIO_PIN_DIR_OUTPUT,
			NRF_GPIO_PIN_INPUT_DISCONNECT,
			NRF_GPIO_PIN_NOPULL,
			NRF_GPIO_PIN_S0D1,
			NRF_GPIO_PIN_NOSENSE);
#endif
	}
#else	
	m_out_pin = OW_OUT_PIN;
	m_in_pin  = OW_IN_PIN;
#if ((defined (OW_PARASITE_POWER_SUPPORT)) && (defined (OW_DEDICATED_POWER_PIN)))
	m_pwr_pin = OW_PWR_PIN;
#endif

	nrf_gpio_cfg_input(m_in_pin, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_pin_set(m_out_pin);
	nrf_gpio_cfg(m_out_pin,
		NRF_GPIO_PIN_DIR_OUTPUT,
		NRF_GPIO_PIN_INPUT_DISCONNECT,
		NRF_GPIO_PIN_NOPULL,
		NRF_GPIO_PIN_S0D1,
		NRF_GPIO_PIN_NOSENSE);

#if ((defined (OW_PARASITE_POWER_SUPPORT)) && (defined (OW_DEDICATED_POWER_PIN)))
#if (defined (OW_POWER_PIN_ACTIVE_STATE)&&(OW_POWER_PIN_ACTIVE_STATE == 1))
	nrf_gpio_pin_clear(m_pwr_pin);
#else
	nrf_gpio_pin_set(m_pwr_pin);
#endif
	nrf_gpio_cfg(m_pwr_pin,
		NRF_GPIO_PIN_DIR_OUTPUT,
		NRF_GPIO_PIN_INPUT_DISCONNECT,
		NRF_GPIO_PIN_NOPULL,
		NRF_GPIO_PIN_S0D1,
		NRF_GPIO_PIN_NOSENSE);
#endif
#endif

	if(!nrf_drv_gpiote_is_init())
	{
		APP_ERROR_CHECK(nrf_drv_gpiote_init()) ;
	}
	
	nrf_drv_gpiote_out_init(m_out_pin, &ow_gpiote_out_config);
	
	APP_ERROR_CHECK(nrf_drv_gpiote_in_init(m_in_pin, &ow_gpiote_in_config, NULL)) ;
	nrf_drv_gpiote_in_event_enable(m_in_pin, true);
	
	APP_ERROR_CHECK(nrf_drv_timer_init(&ow_timer, &ow_timer_cfg, ow_timer_event_handler)) ; 

	nrf_drv_timer_clear(&ow_timer);
	
	nrf_drv_timer_extended_compare(&ow_timer, NRF_TIMER_CC_CHANNEL0, 0, 0, false);
	
	nrf_drv_timer_extended_compare(&ow_timer,
		NRF_TIMER_CC_CHANNEL1,
		OW_READ_PULSE,
		0,
		false);
	
	nrf_drv_timer_extended_compare(&ow_timer,
		NRF_TIMER_CC_CHANNEL2,
		OW_WRITE_TIMESLOT_DELAY,
		NRF_TIMER_SHORT_COMPARE2_STOP_MASK,
		true);

	APP_ERROR_CHECK(nrf_drv_ppi_init()) ;

	APP_ERROR_CHECK(nrf_drv_ppi_channel_alloc(&m_ppi_channel_capture)) ;
	APP_ERROR_CHECK(nrf_drv_ppi_channel_assign(m_ppi_channel_capture,
		nrf_drv_gpiote_in_event_addr_get(m_in_pin),
		nrf_drv_timer_task_address_get(&ow_timer,
		NRF_TIMER_TASK_CAPTURE0)));

	APP_ERROR_CHECK(nrf_drv_ppi_channel_alloc(&m_ppi_channel_strobe_end));
	APP_ERROR_CHECK(nrf_drv_ppi_channel_assign(m_ppi_channel_strobe_end,
		nrf_drv_timer_event_address_get(&ow_timer,
		NRF_TIMER_EVENT_COMPARE1),
		nrf_drv_gpiote_set_task_addr_get(m_out_pin)));

	APP_ERROR_CHECK(nrf_drv_ppi_channel_enable(m_ppi_channel_capture));
	APP_ERROR_CHECK(nrf_drv_ppi_channel_enable(m_ppi_channel_strobe_end));
	
	nrf_drv_gpiote_out_task_enable(m_out_pin);

	m_state = OWMHS_IDLE;
}

#if ((defined (OW_PARASITE_POWER_SUPPORT)) && (defined (OW_DEDICATED_POWER_PIN)))
uint32_t owmh_ow_change_pins(uint32_t out_pin, uint32_t in_pin, uint32_t pwr_pin)
#else
static uint32_t owmh_ow_change_pins(uint32_t out_pin, uint32_t in_pin)
#endif
{
	nrf_drv_gpiote_out_task_disable(m_out_pin);
	
	nrf_drv_ppi_channel_disable(m_ppi_channel_capture);
	nrf_drv_ppi_channel_disable(m_ppi_channel_strobe_end);
	
	nrf_drv_gpiote_out_uninit(m_out_pin);
	nrf_drv_gpiote_in_uninit(m_in_pin);
	
	m_out_pin = out_pin;
	m_in_pin  = in_pin;
#if ((defined (OW_PARASITE_POWER_SUPPORT)) && (defined (OW_DEDICATED_POWER_PIN)))
	m_pwr_pin = pwr_pin; 
#endif

	nrf_drv_gpiote_out_init(out_pin, &ow_gpiote_out_config);
	
	APP_ERROR_CHECK(nrf_drv_gpiote_in_init(in_pin, &ow_gpiote_in_config, NULL)) ;
	nrf_drv_gpiote_in_event_enable(in_pin, true);
	
	APP_ERROR_CHECK(nrf_drv_ppi_channel_assign(m_ppi_channel_capture,
		nrf_drv_gpiote_in_event_addr_get(in_pin),
		nrf_drv_timer_task_address_get(&ow_timer,
		NRF_TIMER_TASK_CAPTURE0)));

	APP_ERROR_CHECK(nrf_drv_ppi_channel_assign(m_ppi_channel_strobe_end,
		nrf_drv_timer_event_address_get(&ow_timer,
		NRF_TIMER_EVENT_COMPARE1),
		nrf_drv_gpiote_set_task_addr_get(out_pin)));
	
	APP_ERROR_CHECK(nrf_drv_ppi_channel_enable(m_ppi_channel_capture));
	APP_ERROR_CHECK(nrf_drv_ppi_channel_enable(m_ppi_channel_strobe_end));
	
	nrf_drv_gpiote_out_task_enable(out_pin);

	m_state = OWMHS_IDLE;
	return 0;
}
	
uint32_t owm_hal_uninitialize(void)
{
	if(m_state != OWMHS_IDLE) return 1;
	
	// uninit PPI
	nrf_drv_ppi_channel_disable(m_ppi_channel_capture);
	nrf_drv_ppi_channel_disable(m_ppi_channel_strobe_end);
	nrf_drv_ppi_channel_free(m_ppi_channel_capture);	
	nrf_drv_ppi_channel_free(m_ppi_channel_strobe_end);	
	// uninit TIMER	
	nrf_drv_timer_uninit(&ow_timer);
	// uninit GPIOTE
	nrf_drv_gpiote_out_uninit(m_out_pin);
	nrf_drv_gpiote_in_uninit(m_in_pin);
	/* Reset pins to default states */
	
#ifdef OW_MULTI_CHANNEL 
	for (uint8_t k = 0; k < OW_CHANNEL_COUNT; ++k)
	{
		nrf_gpio_cfg_default(ow_pins[k].tx_pin);
		nrf_gpio_cfg_default(ow_pins[k].rx_pin);
#if ((defined (OW_PARASITE_POWER_SUPPORT)) && (defined (OW_DEDICATED_POWER_PIN)))
		nrf_gpio_cfg_default(ow_pins[k].pwr_pin);
#endif
	}
#else 	
	nrf_gpio_cfg_default(m_out_pin);
	nrf_gpio_cfg_default(m_in_pin);
#if ((defined (OW_PARASITE_POWER_SUPPORT)) && (defined (OW_DEDICATED_POWER_PIN)))
	nrf_gpio_cfg_default(m_pwr_pin);
#endif
#endif
	
	m_state = OWMHS_NOT_INITIALIZED;
	return 1;
}

#ifdef OW_MULTI_CHANNEL 
void ow_set_channel(uint8_t channel)
{
	APP_ERROR_CHECK_BOOL(m_state == OWMHS_IDLE);

	if (m_out_pin != ow_pins[channel].tx_pin)
#if ((defined (OW_PARASITE_POWER_SUPPORT)) && (defined (OW_DEDICATED_POWER_PIN)))
		owmh_ow_change_pins(ow_pins[channel].tx_pin, ow_pins[channel].rx_pin, ow_pins[channel].pwr_pin);
#else
		owmh_ow_change_pins(ow_pins[channel].tx_pin, ow_pins[channel].rx_pin);
#endif
}
#endif // (defined (OW_MULTI_CHANNEL))

#ifdef OW_PARASITE_POWER_SUPPORT
static void ow_power_on()
{
#ifdef OW_DEDICATED_POWER_PIN
#if (defined (OW_POWER_PIN_ACTIVE_STATE)&&(OW_POWER_PIN_ACTIVE_STATE == 1))
	nrf_gpio_pin_set(m_pwr_pin);
#else
	nrf_gpio_pin_clear(m_pwr_pin);
#endif
#else
	nrf_drv_gpiote_out_task_disable(m_out_pin);
	nrf_drv_ppi_channel_disable(m_ppi_channel_strobe_end);
	nrf_drv_gpiote_out_uninit(m_out_pin);
	nrf_gpio_cfg(m_out_pin,
		NRF_GPIO_PIN_DIR_OUTPUT,
		NRF_GPIO_PIN_INPUT_DISCONNECT,
		NRF_GPIO_PIN_NOPULL,
		NRF_GPIO_PIN_D0H1,
		NRF_GPIO_PIN_NOSENSE);
#endif
}

static void ow_power_off()
{
#ifdef OW_DEDICATED_POWER_PIN
#if (defined (OW_POWER_PIN_ACTIVE_STATE)&&(OW_POWER_PIN_ACTIVE_STATE == 1))
	nrf_gpio_pin_clear(m_pwr_pin);
#else
	nrf_gpio_pin_set(m_pwr_pin);
#endif
#else
	nrf_gpio_cfg(m_out_pin,
		NRF_GPIO_PIN_DIR_OUTPUT,
		NRF_GPIO_PIN_INPUT_DISCONNECT,
		NRF_GPIO_PIN_NOPULL,
		NRF_GPIO_PIN_S0D1,
		NRF_GPIO_PIN_NOSENSE);
	nrf_drv_gpiote_out_init(m_out_pin, &ow_gpiote_out_config);
	APP_ERROR_CHECK(nrf_drv_ppi_channel_assign(m_ppi_channel_strobe_end,
		nrf_drv_timer_event_address_get(&ow_timer,
			NRF_TIMER_EVENT_COMPARE1),
		nrf_drv_gpiote_set_task_addr_get(m_out_pin)));
	APP_ERROR_CHECK(nrf_drv_ppi_channel_enable(m_ppi_channel_strobe_end));
	nrf_drv_gpiote_out_task_enable(m_out_pin);
#endif
}
#endif // (defined (OW_PARASITE_POWER_SUPPORT))

static void owmh_continue(uint32_t pulse, uint32_t delay)
{
	nrf_drv_timer_clear(&ow_timer);
	nrfx_timer_capture(&ow_timer, NRF_TIMER_CC_CHANNEL0);

	nrf_drv_timer_compare(&ow_timer,
		NRF_TIMER_CC_CHANNEL1,
		pulse,
		false);
	nrf_drv_timer_compare(&ow_timer,
		NRF_TIMER_CC_CHANNEL2,
		delay,
		true);

	if (m_state < OWMHS_FLAG_PAUSE)
	{
		nrfx_gpiote_clr_task_trigger(m_out_pin); 
	}
	nrf_drv_timer_resume(&ow_timer);
}

static void owmh_start(owmh_state_t state)
{
	uint32_t pulse; 
	uint32_t delay;
	
	if (!nrf_gpio_pin_read(m_in_pin))
	{
		m_callback(OWMHCR_ERROR);
		return;
	}

	switch (state)
	{
	case OWMHS_RESET:
		pulse = OW_RESET_PULSE;
		delay = OW_RESET_DELAY;
		break;

	case OWMHS_WRITE0:
		pulse = OW_WRITE0_PULSE;
		delay = OW_WRITE_TIMESLOT_DELAY;
		m_tx_bit = 0;
		break;

	case OWMHS_WRITE1:
		pulse = OW_READ_PULSE;
		delay = OW_WRITE_TIMESLOT_DELAY;
		m_tx_bit = 1;
		break;

	case OWMHS_READ:
		pulse = OW_READ_PULSE;
		delay = OW_READ_TIMESLOT_DELAY;
		break;

	case OWMHS_SEQUENCE:
		m_tx_bit = ((m_tx_count == 0) || ((*(m_p_tx_buf) & 1)));
		pulse = (m_tx_bit) ? OW_READ_PULSE : OW_WRITE0_PULSE;
		delay = (m_tx_count) ? OW_WRITE_TIMESLOT_DELAY : OW_READ_TIMESLOT_DELAY;
		break;

	case OWMHS_READ_FLAG:
		pulse = OW_READ_PULSE;
		delay = OW_READ_TIMESLOT_DELAY;
		break;

		break;

	case OWMHS_DELAY:
		pulse = OW_MILLISECOND_DELAY + 10;
		delay = OW_MILLISECOND_DELAY;
		break;

#if (defined (OW_PARASITE_POWER_SUPPORT))
	case OWMHS_POWER_HOLD:
		pulse = OW_MILLISECOND_DELAY + 10;
		delay = OW_MILLISECOND_DELAY;
		break;
#endif
	
	default: // OWMHS_IDLE, OWMHS_FLAG_PAUSE, OWMHS_NOT_INITIALIZED
		APP_ERROR_CHECK_BOOL(false);
	}	
	
	m_state = state;
	owmh_continue(pulse, delay);
}

void owmh_reset(void)
{
	APP_ERROR_CHECK_BOOL(m_state == OWMHS_IDLE);
	owmh_start(OWMHS_RESET);
}

void owmh_read(void)
{
	APP_ERROR_CHECK_BOOL(m_state == OWMHS_IDLE);
	owmh_start(OWMHS_READ);
}

void owmh_write(uint8_t bit)
{
	APP_ERROR_CHECK_BOOL(m_state == OWMHS_IDLE);
	if (bit)
		owmh_start(OWMHS_WRITE1);
	else
		owmh_start(OWMHS_WRITE0);
}

void owmh_sequence(uint8_t* p_txdata, uint8_t* p_rxdata, uint8_t  tx_count, uint8_t  rx_count)
{
	APP_ERROR_CHECK_BOOL(m_state == OWMHS_IDLE);
	if ((!tx_count)&&(!rx_count)) return;
	m_p_tx_buf = p_txdata;
	m_p_rx_buf = p_rxdata;
	m_tx_count = tx_count;
	m_rx_count = rx_count;
	m_byte_mask = 1;
	owmh_start(OWMHS_SEQUENCE);
}

void owmh_wait_flag(uint16_t max_wait_ms)
{
	APP_ERROR_CHECK_BOOL(m_state == OWMHS_IDLE);
	m_delay_counter = max_wait_ms;
	owmh_start(OWMHS_READ_FLAG);
}

void owmh_delay(uint16_t delay_ms)
{
	APP_ERROR_CHECK_BOOL(m_state == OWMHS_IDLE);
	m_delay_counter = delay_ms;
	owmh_start(OWMHS_DELAY);
}
	
#ifdef OW_PARASITE_POWER_SUPPORT
void owmh_hold_power(uint16_t delay_ms)
{
	APP_ERROR_CHECK_BOOL(m_state == OWMHS_IDLE);
	m_delay_counter = delay_ms;
	ow_power_on();
	owmh_start(OWMHS_POWER_HOLD);
}
#endif // defined (OW_PARASITE_POWER_SUPPORT)

// timer interrupt handler (on compare2)
static void ow_timer_event_handler(nrf_timer_event_t event_type, void * p_context)
{
	static uint8_t bit_buf;
	uint32_t capture_value;
	owmh_callback_result_t result;
	owmh_state_t state = m_state;
	uint32_t pulse = 0; 
	uint32_t delay;

	capture_value = nrf_drv_timer_capture_get(&ow_timer, NRF_TIMER_CC_CHANNEL0);
	switch (m_state)
	{
//----------------------------------------------------------------------------------------------------------------	
	case OWMHS_RESET :
		if ((capture_value < OW_RESET_PULSE) || (capture_value > (OW_RESET_DELAY - 30)))
			result = OWMHCR_ERROR;
		else if(capture_value > OW_PRESENCE_BOUND)
			result = OWMHCR_RESET_OK;
		else
			result = OWMHCR_RESET_NO_RESPONCE;
		break;
//----------------------------------------------------------------------------------------------------------------	
	case OWMHS_WRITE1:
		if ((capture_value < OW_WRITE1_PULSE) || (capture_value > (OW_WRITE_TIMESLOT_DELAY - 30)))
			result = OWMHCR_ERROR;
		else
			result = OWMHCR_WRITE_OK;
		break;
//----------------------------------------------------------------------------------------------------------------	
	case OWMHS_WRITE0:
		if ((capture_value < OW_WRITE0_PULSE) || (capture_value > (OW_WRITE0_PULSE + 15)))
			result = OWMHCR_ERROR;
		else
			result = OWMHCR_WRITE_OK;
		break;
//----------------------------------------------------------------------------------------------------------------	
	case OWMHS_READ:
		if ((capture_value < OW_WRITE1_PULSE) || (capture_value > (OW_READ_TIMESLOT_DELAY - 30)))
			result = OWMHCR_ERROR;
		else if(capture_value > OW_READ0_BOUND)
			result = OWMHCR_READ_0;
		else if(capture_value < OW_READ1_BOUND)
			result = OWMHCR_READ_1;
		else
			result = OWMHCR_ERROR;
		break;
//----------------------------------------------------------------------------------------------------------------	
	case OWMHS_DELAY:
		if (--m_delay_counter > 0)
		{
			pulse = OW_MILLISECOND_DELAY + 10;
			delay = OW_MILLISECOND_DELAY;
		}
		else 
			result = OWMHCR_WAIT_OK;
		break;
//----------------------------------------------------------------------------------------------------------------	
#ifdef OW_PARASITE_POWER_SUPPORT
	case OWMHS_POWER_HOLD:
		if (--m_delay_counter > 0)
		{
			pulse = OW_MILLISECOND_DELAY + 10;
			delay = OW_MILLISECOND_DELAY;
		}
		else 
			ow_power_off();
			result = OWMHCR_WAIT_OK;
		break;
#endif
//----------------------------------------------------------------------------------------------------------------	
	case OWMHS_READ_FLAG :
		if ((capture_value < OW_WRITE1_PULSE) || (capture_value > (OW_READ_TIMESLOT_DELAY - 30))
			              || ((capture_value > OW_READ1_BOUND) && (capture_value < OW_READ0_BOUND)))
			result = OWMHCR_ERROR;
		else if (capture_value < OW_READ1_BOUND)
		{
			result = OWMHCR_FLAG_OK;
		}
		else if (m_delay_counter > 0)
		{
			state = OWMHS_FLAG_PAUSE;
			pulse = OW_FLAG_PAUSE_DELAY + 10;
			delay = OW_FLAG_PAUSE_DELAY;
		}
		else
		{
			result = OWMHCR_TIME_OUT;
		}
		break;
//----------------------------------------------------------------------------------------------------------------	
	case OWMHS_FLAG_PAUSE :
		--m_delay_counter;
		state = OWMHS_READ_FLAG;
		pulse = OW_WRITE1_PULSE;
		delay = OW_READ_TIMESLOT_DELAY;
		break;
//----------------------------------------------------------------------------------------------------------------	
	case OWMHS_SEQUENCE :
		if(m_tx_count > 0) // bit was transmitted
		{
			// Check if transmitted byte is not corrupted
			if(((m_tx_bit)&&((capture_value < OW_WRITE1_PULSE) || (capture_value > (OW_WRITE1_PULSE + OW_WRITE1_PULSE_TOLERANCE))))
					|| ((!m_tx_bit)&&((capture_value < OW_WRITE0_PULSE) || (capture_value > (OW_WRITE0_PULSE + OW_WRITE0_PULSE_TOLERANCE)))))
			{
				result = OWMHCR_ERROR;
				break;
			}

			if (--m_tx_count > 0) // There are more bits to transmit
				{
					// set data pointer and mask
					if(m_byte_mask == 0x80)
					{
						m_byte_mask = 1;
						++m_p_tx_buf;
					}
					else m_byte_mask <<= 1;
					// form next 1-WIRE writing timeslot
					m_tx_bit = (*(m_p_tx_buf)&m_byte_mask);
					pulse = (m_tx_bit) ? OW_READ_PULSE : OW_WRITE0_PULSE;
					delay = OW_WRITE_TIMESLOT_DELAY;
				}
			else
			{
				if (m_rx_count > 0)
				{
					pulse = OW_READ_PULSE;
					delay = OW_READ_TIMESLOT_DELAY;
					m_byte_mask = 1;
				}
				else
					result = OWMHCR_SEQUENCE_OK;
			}
			break;
		}
				
		if (m_rx_count > 0) // bit was resieved
			{
				if ((capture_value < OW_WRITE1_PULSE) || (capture_value > (OW_READ_TIMESLOT_DELAY - 30))
						|| ((capture_value > OW_READ1_BOUND) && (capture_value < OW_READ0_BOUND)))
				{
					result = OWMHCR_ERROR;
					break;
				}
				
				if (capture_value < OW_READ1_BOUND)
					*(m_p_rx_buf) |= m_byte_mask;
				else 
					*(m_p_rx_buf) &= (~m_byte_mask);
				
				if (m_byte_mask == 0x80)
				{
					m_byte_mask = 1;
					++m_p_rx_buf;
				}
				else m_byte_mask <<= 1;
					
				if (--m_rx_count > 0) // There are more bits to resieve
					{
						pulse = OW_READ_PULSE;
						delay = OW_READ_TIMESLOT_DELAY;
					}
				else
					result = OWMHCR_SEQUENCE_OK;
			}
		break;
//----------------------------------------------------------------------------------------------------------------	
		default: // OWMHS_IDLE, OWMHS_NOT_INITIALIZED
			APP_ERROR_CHECK_BOOL(false);
//----------------------------------------------------------------------------------------------------------------	
	} // switch (m_state)
	
	if (!nrf_gpio_pin_read(m_in_pin))
		result = OWMHCR_ERROR;
	
	if (pulse)
	{
		m_state = state;
		owmh_continue(pulse, delay);
	}
	else
	{
		m_state = OWMHS_IDLE;
		m_callback(result);
	}
}
