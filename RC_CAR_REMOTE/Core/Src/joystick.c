/*
 * joystick.c
 *
 *  Created on: Sep 24, 2026
 *      Author: jrose
 */

#include "joystick.h"
#include "packets.h"

#define STEERING_NEUTRAL_POSITION 1700
// different neutral reading, same ~750 span to either extreme (from neutral to full left/right/up/down)
#define THROTTLE_NEUTRAL_POSITION 1630

static ADC_HandleTypeDef *adc;

static void NormalizeJoystickValues(JoystickPacket *j, uint32_t steering_adc, uint32_t throttle_adc){

	int16_t steering_difference = (int16_t)steering_adc - STEERING_NEUTRAL_POSITION;
	int16_t throttle_difference = (int16_t)throttle_adc - THROTTLE_NEUTRAL_POSITION;

	// I could divide difference by a scaler of 7.5 to convert to -100:100 scale
	// but to avoid floating point operation, just multiply by 2 then divide by 15
	int16_t steering_scaled = ( steering_difference * 2 ) / 15;
	int16_t throttle_scaled = ( throttle_difference * 2 ) / 15;

	j->steering = steering_scaled < -100 ? -100
			    : steering_scaled > 100 ? 100
			    : steering_scaled <=5 && steering_scaled >= -5 ? 0 // small deadzone so throttle isn't a hairtrigger
			    : steering_scaled;
	j->throttle = throttle_scaled < -100 ? -100
				: throttle_scaled > 100 ? 100
				: throttle_scaled <=5 && throttle_scaled >= -5 ? 0 // small deadzone so throttle isn't a hairtrigger
				: throttle_scaled;
}

static HAL_StatusTypeDef GetJoystickValues(JoystickPacket *j){
	uint32_t steering_adc;
	uint32_t throttle_adc;
	HAL_StatusTypeDef status;

	status = HAL_ADC_Start(adc);
	if(status != HAL_OK) return status;

	status = HAL_ADC_PollForConversion(adc, 100);
	if(status != HAL_OK) return status;

	steering_adc = HAL_ADC_GetValue(adc);

	status = HAL_ADC_PollForConversion(adc, 100);
	if(status != HAL_OK) return status;

	throttle_adc = HAL_ADC_GetValue(adc);

	// ADC_Stop can only ever return HAL_OK, no need to check status
	HAL_ADC_Stop(adc);

	NormalizeJoystickValues(j, steering_adc, throttle_adc);

	return HAL_OK;
}

HAL_StatusTypeDef BuildJoystickPacket(ADC_HandleTypeDef *hadc1, JoystickPacket *j){
	adc = hadc1;
	return GetJoystickValues(j);
}
