/*
 * joystick.c
 *
 *  Created on: Sep 24, 2026
 *      Author: jrose
 */

#include "joystick.h"

#define STEERING_NEUTRAL_POSITION 1700
// different neutral reading, same ~750 span to either extreme
#define THROTTLE_NEUTRAL_POSITION 1630

#define STEERING 0
#define THROTTLE 1

static ADC_HandleTypeDef *adc;

static int8_t NormalizeJoystickValues(uint32_t raw, uint8_t joystick){

	int16_t difference = (joystick == STEERING) ?
			raw - STEERING_NEUTRAL_POSITION
			:
			raw - THROTTLE_NEUTRAL_POSITION;
	// the difference between either span from neutral is 750 for both sticks.
	// I could divide difference by a scaler of 7.5 to convert to -100:100 scale
	// but to avoid floating point operation, just multiply by 2 then divide by 15
	int8_t scaled = (difference * 2) / 15;
	if(scaled < -100) return -100;
	if(scaled > 100) return 100;
	//small deadzone so the joystick isn't a hair trigger
	if(scaled <= 5 && scaled >= -5){
		return 0;
	}
	return scaled;
}

static HAL_StatusTypeDef GetJoystickValues(int8_t values[]){
	uint32_t steering_position;
	uint32_t throttle_position;
	HAL_StatusTypeDef status;

	status = HAL_ADC_Start(adc);
	if(status != HAL_OK) return status;

	status = HAL_ADC_PollForConversion(adc, 100);
	if(status != HAL_OK) return status;

	steering_position = HAL_ADC_GetValue(adc);

	status = HAL_ADC_PollForConversion(adc, 100);
	if(status != HAL_OK) return status;

	throttle_position = HAL_ADC_GetValue(adc);

	// Stop can only ever return HAL_OK, so no need to check status
	HAL_ADC_Stop(adc);

	values[0] = NormalizeJoystickValues(steering_position, STEERING);
	values[1] = NormalizeJoystickValues(throttle_position, THROTTLE);

	return HAL_OK;
}

HAL_StatusTypeDef BuildJoystickPacket(ADC_HandleTypeDef *hadc1, int8_t values[]){
	adc = hadc1;
	return GetJoystickValues(values);
}
