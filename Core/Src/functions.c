/*
 * functions.c
 *
 *  Created on: Nov 30, 2024
 *      Author: Alecsia
 */

#include "functions.h"
#include "main.h"
#include "stdio.h"

/*
 * This functions updates the physical Auxiliary system
 */
void Update_Auxiliary_System(Auxiliary *aux_pointer) {
	static uint8_t sign_left_500ms = 0;
	static uint8_t sign_right_500ms = 0;
	static bool toggle_sign_left = OFF;
	static bool toggle_sign_right = OFF;
	/*
	 * Set all pins with the negated values from auxiliary structure
	 * Outputs are negated -> transistors type P
	 */
	HAL_GPIO_WritePin(GPIOB, BACK_LIGHT_Pin | FRONT_LIGHT_Pin,
			(!aux_pointer->lights));
	HAL_GPIO_WritePin(GPIOB, HORN_Pin, (!aux_pointer->horn));
//	HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, (!aux_pointer->fan));
	HAL_GPIO_WritePin(GPIOA, CAMERA_Pin, (!aux_pointer->camera));
	HAL_GPIO_WritePin(GPIOB, BRAKE_Pin, (!aux_pointer->brake));

	/*
	 * For the signals:
	 * 	VERIFY THE COMPETITION REGULATIONS
	 * 	each sign should toggle at 500ms
	 * 	when signaling left, the right one should be off
	 * 	when signaling right, the left one should be off
	 * 	when using hazard signals ("pe avarii"), make sure left & right are synchronized
	 */


 if(aux_pointer->sign_left == ON && aux_pointer->sign_right == ON && toggle_sign_left == toggle_sign_right){
	 sign_left_500ms = sign_right_500ms;

 }else if(toggle_sign_left == ON && toggle_sign_right == OFF){
	 aux_pointer->sign_right = OFF;
 } else if(toggle_sign_right == ON && toggle_sign_left == OFF){
	 aux_pointer->sign_left = OFF;
 }

	if (aux_pointer->sign_left == ON) {
		//count 500ms
		if (++sign_left_500ms == 10) {
			HAL_GPIO_TogglePin(GPIOB, SIGN_LEFT_Pin);
			//store the current state of sign left
			toggle_sign_left = (!toggle_sign_left);
			//reset counter
			sign_left_500ms = 0;
		}
	} else {
		//turn off
		HAL_GPIO_WritePin(GPIOB, SIGN_LEFT_Pin, (!aux_pointer->sign_left));
		toggle_sign_left = OFF;
		sign_left_500ms = 9;
	}

	if (aux_pointer->sign_right == ON) {
		if (++sign_right_500ms == 10) {
			HAL_GPIO_TogglePin(GPIOB, SIGN_RIGHT_Pin);
			toggle_sign_right = (!toggle_sign_right);
			sign_right_500ms = 0;
		}
	} else {
		HAL_GPIO_WritePin(GPIOB, SIGN_RIGHT_Pin, (!aux_pointer->sign_right));
		toggle_sign_right = OFF;
		sign_right_500ms = 9;
	}

}

void Send_Auxiliary_State_CAN(CAN_HandleTypeDef hcan, uint8_t *Activity_Check) {
	CAN_TxHeaderTypeDef txHeader;
	uint32_t txMailbox;

	// Prepare the CAN header
	txHeader.StdId = AUXILIARY_ID;      // Replace with appropriate CAN ID
	txHeader.ExtId = 0;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.DLC = 1;             // Length of data (up to 8 bytes)
	txHeader.TransmitGlobalTime = DISABLE;

	// Transmit the CAN message
	HAL_CAN_AddTxMessage(&hcan, &txHeader, Activity_Check, &txMailbox);
}

void Update_Offline_Mode(Auxiliary *aux_offline) {
	aux_offline->brake = HAL_GPIO_ReadPin(GPIOA, BRAKE_OFFLINE_Pin);
	aux_offline->camera = HAL_GPIO_ReadPin(GPIOB, CAMERA_OFFLINE_Pin);
	aux_offline->horn = HAL_GPIO_ReadPin(GPIOB, HORN_OFFLINE_Pin);
	aux_offline->sign_left = HAL_GPIO_ReadPin(GPIOB, SIGN_LEFT_OFFLINE_Pin);
	aux_offline->sign_right = HAL_GPIO_ReadPin(GPIOB, SIGN_RIGHT_OFFLINE_Pin);

	if (HAL_GPIO_ReadPin(GPIOC, FRONT_LIGHT_OFFLINE_Pin)
			|| HAL_GPIO_ReadPin(GPIOC, BACK_LIGHT_OFFLINE_Pin)) {
		aux_offline->lights = ON;
	} else
		aux_offline->lights = OFF;
}

