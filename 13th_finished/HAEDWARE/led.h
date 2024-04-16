#ifndef _LED_H
#define _LED_H

#include "gpio.h"

#define LED_ON 	GPIO_PIN_RESET
#define LED_OFF	GPIO_PIN_SET

#define D1 0x01
#define D2 0x02
#define D3 0x04
#define D4 0x08
#define D5 0x10
#define D6 0x20
#define D7 0x40
#define D8 0x80

#endif