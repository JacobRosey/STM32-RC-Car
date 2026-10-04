/*
 * joystick.h
 *
 *  Created on: Sep 24, 2026
 *      Author: jrose
 */

#include "main.h"

#ifndef INC_JOYSTICK_H_
#define INC_JOYSTICK_H_

HAL_StatusTypeDef BuildJoystickPacket(ADC_HandleTypeDef* hadc1, int8_t values[]);

#endif /* INC_JOYSTICK_H_ */
