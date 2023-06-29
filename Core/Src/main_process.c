/*
 * main_process.c
 *
 *  Created on: Jun 18, 2023
 *      Author: longn
 */
#include "main_process.h"
#include "axis_driver.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
S_PROCESS sProcess;
uint8_t RotateOK;

static bool SendBLDCTimeout(uint16_t Timeout, FlagStatus1 eStatus)
{
	bool flagFinishTask = false;
	static uint32_t oldTime = 0;

	if (eStatus == RESET1)
	{
		oldTime = HAL_GetTick();
	}
	else
	{
		if((HAL_GetTick() - oldTime) >= Timeout)
		{
			flagFinishTask = true;
		}
	}
	return flagFinishTask;
}
void main_process()
{
	switch(sProcess.process)
	{
	case UARThandler:
		UART2_Handler();
		sProcess.process++;
		break;
	case sendBLDC:
		if (SendBLDCTimeout(100, SET1))
		{
			sProcess.frameFront[0] = 0xABCD;
			sProcess.frameFront[1] = sModule2Params.speed;
			sProcess.frameFront[2] = sModule1Params.speed;
			sProcess.frameFront[3] = (uint16_t) (sProcess.frameFront[0] ^sProcess.frameFront[1]^sProcess.frameFront[2]);
			sProcess.frameRear[0] = 0xABCD;
			sProcess.frameRear[1] = sModule3Params.speed;
			sProcess.frameRear[2] = sModule4Params.speed;
			sProcess.frameRear[3] = (uint16_t) (sProcess.frameRear[0] ^sProcess.frameRear[1]^sProcess.frameRear[2]);
			HAL_UART_Transmit(&huart2, (uint8_t *)sProcess.frameFront, sizeof(sProcess.frameFront), 10);
			HAL_UART_Transmit(&huart6, (uint8_t *)sProcess.frameRear, sizeof(sProcess.frameRear), 10);
			SendBLDCTimeout(100, RESET1);
		}
		sProcess.process++;
		break;
	case adjustAngle:
//		if(RotateOK)// CHO HAM QUAY HUONG VAO DAY
//		{
//
//		}

	    axis1.angle = sModule1Params.targetAngle;
        pwm_handler(&htim3, &axis1, cnt1, CH1_CH2);
        axis2.angle = sModule2Params.targetAngle;
        pwm_handler(&htim3, &axis2, cnt2, CH3_CH4);
        axis3.angle = sModule3Params.targetAngle;
        pwm_handler(&htim4, &axis3, cnt3, CH1_CH2);
        axis4.angle = sModule4Params.targetAngle;
        pwm_handler(&htim4, &axis4, cnt4, CH3_CH4);
//		else if (!RotateOK)
//		{
////			if ( && SendBLDCTimeout(10000, SET1))
////			sModule1Params.speed = 0;
////			sModule2Params.speed = 0;
////			sModule3Params.speed = 0;
////			sModule4Params.speed = 0;
////			SendBLDCTimeout(10000, RESET1);
//		}
		sProcess.process = UARThandler;
		break;
	default:
		sProcess.process = UARThandler;
		break;
	}
}

