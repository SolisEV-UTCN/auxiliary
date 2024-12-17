/*
 * functions.h
 *
 *  Created on: Nov 30, 2024
 *      Author: Alecsia
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

#include "main.h"

#define DASHBOARD_ID 0x701
#define AUXILIARY_ID 0x700
#define SAFE_STATE 0x00
#define AUXILIARY_WORKS 0xFF
#define AUXILIARY_ERROR 0x00

typedef union {
	//8-bit value representing the entire union
	uint8_t state;
	//to access each bit individually
	struct {
		uint8_t sign_left :1;
		uint8_t sign_right :1;
		uint8_t safe_state :1;
		uint8_t brake :1;
		uint8_t horn :1;
		uint8_t rear_lights:1;
		uint8_t camera :1;
		uint8_t lights :1;
	};
} Auxiliary;

typedef struct {
	uint32_t Sign_Left_Current;
	uint32_t Sign_Right_Current;
	uint32_t Fan_Current;
	uint32_t Brake_Current;
	uint32_t Horn_Current;
	uint32_t Hazard_Current;
	uint32_t Camera_Current;
	uint32_t HeadLights_Current;
} Aux_Error;

typedef enum {
	OFF = 0, ON = 1
} bool;

void Update_Auxiliary_System(Auxiliary *aux_pointer);

void Get_ADC_Value(ADC_HandleTypeDef hadc4, Auxiliary *aux_pointer,
		bool toggle_sign_left, bool toggle_sign_right, uint32_t adc_value,
		uint8_t *activity_check);

void Send_Auxiliary_State_CAN(CAN_HandleTypeDef hcan, uint8_t *Activity_Check);

void Update_Offline_Mode(Auxiliary *Offline_Mode);

#endif /* FUNCTIONS_H_ */
