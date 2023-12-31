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
extern uint8_t axis1_chk;
extern uint8_t axis2_chk;
extern uint8_t axis3_chk;
extern uint8_t axis4_chk;
extern uint32_t pre;
S_PROCESS sProcess;
ret_val_t home_flag = 0;
uint32_t preTime = 0;
uint32_t timeInterval = 0;

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
		if ((HAL_GetTick() - oldTime) >= Timeout)
		{
			flagFinishTask = true;
		}
	}
	return flagFinishTask;
}
void main_process()
{

	switch (sProcess.process)
	{
	case UARThandler:
    timeInterval = HAL_GetTick() - preTime;
		UART2_Handler();
		sProcess.process++;
		break;
	case sendBLDC:


		if (SendBLDCTimeout(100, SET1) && timeInterval > 0)
		{
			sProcess.frameFront[0] = 0xABCD;
			sProcess.frameFront[1] = sModule2Params.speed;
			sProcess.frameFront[2] = sModule1Params.speed;
			sProcess.frameFront[3] = (uint16_t)(sProcess.frameFront[0] ^ sProcess.frameFront[1] ^ sProcess.frameFront[2]);
			sProcess.frameRear[0] = 0xABCD;
			sProcess.frameRear[1] = -sModule4Params.speed;
			sProcess.frameRear[2] = -sModule3Params.speed;
			sProcess.frameRear[3] = (uint16_t)(sProcess.frameRear[0] ^ sProcess.frameRear[1] ^ sProcess.frameRear[2]);
			HAL_UART_Transmit(&huart2, (uint8_t *)sProcess.frameFront, sizeof(sProcess.frameFront), 10);
			HAL_UART_Transmit(&huart6, (uint8_t *)sProcess.frameRear, sizeof(sProcess.frameRear), 10);
			SendBLDCTimeout(100, RESET1);
		}

		sProcess.process = UARThandler;
		break;
	case adjustAngle:
	    if(axis1.angle != sModule1Params.targetAngle || axis2.angle != sModule2Params.targetAngle)
	    {
	        preTime = HAL_GetTick();
	        timeInterval = 0;
	    }
		axis1.angle = sModule1Params.targetAngle;

		axis2.angle = sModule2Params.targetAngle;

		axis3.angle = sModule3Params.targetAngle;

		axis4.angle = sModule4Params.targetAngle;
		/*Re calib axis angle, run till reaching end stop*/
		if (sVehicalParams.Vy != 0 && sVehicalParams.Vx == 0 && sVehicalParams.W == 0 && home_flag != SUCCESSFUL)
		{
			home_flag = auto_home();
		}
		/*Wait till reaching stop*/
//		preTime = HAL_GetTick();
		pwm_handler(&htim3, &axis1, cnt1, CH1_CH2);
		pwm_handler(&htim3, &axis2, cnt2, CH3_CH4);
		pwm_handler(&htim4, &axis3, cnt3, CH1_CH2);
		pwm_handler(&htim4, &axis4, cnt4, CH3_CH4);


		sProcess.process++;
		break;
	default:
		sProcess.process = UARThandler;
		break;
	}
}
