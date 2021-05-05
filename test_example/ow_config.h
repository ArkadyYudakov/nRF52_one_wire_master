#ifndef	OW_CONFIG_H__
#define OW_CONFIG_H__

//---------------------------------------------------------------------
//                1-wire master driver configuration
//---------------------------------------------------------------------

// nRFF52 1-wire master HAL timer instance  
#define OW_TIMER_INSTANCE 2

// 1-wire manager packet queue capacity
#define OW_MANAGER_FIFO_SIZE 16

// if not defined, driver functions for ROM searching excluded
#define OW_ROM_SEARCH_SUPPORT

// if not defined, driver functions for parasite power support excluded
#define OW_PARASITE_POWER_SUPPORT

// if defined, separated pin used for power forcing
// else out pin configuration changes temporary
//#define OW_DEDICATED_POWER_PIN 

// active state of power pin. If not defined, default is 0 (low).
//#define OW_POWER_PIN_ACTIVE_STATE 0

// if defined, driver supports several channels
#define OW_MULTI_CHANNEL	

#ifdef OW_MULTI_CHANNEL
// number of channels
#define OW_CHANNEL_COUNT 2
// channel pins definition in form of brace-enclosed array initialization list
// example for 3 channels with dedicated power pin
// { { IN0,  OUT0, PWR0 }, { IN1,  OUT1, PWR1 }, { IN2,  OUT2, PWR2 } }
// example for 2 channels without dedicated power pin
// { { IN0,  OUT0 }, { IN1,  OUT1 } }        
#define OW_PINS_ARRAY  { { 4,  5 }, { 2,  3 } }
#else
// 1-wire pins config
#define OW_OUT_PIN  2
#define OW_IN_PIN   3
#define OW_PWR_PIN  14
#endif 

//	14, 15,
//	16, 17,
//	18, 19,

#endif // OW_CONFIG_H__
