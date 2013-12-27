#ifndef _SERVO_H_
#define _SERVO_H_

#include "stm32f4_discovery.h"
#include "FreeRTOS.h"

#define PERIOD     20 /* micro-sec */
#define SERVO_0    1
#define SERVO_180  2

void Servo_Configuration();
void Servo_set_pos(uint8_t angle /* degree */, uint8_t ch);

#endif
