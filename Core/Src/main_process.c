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
S_PROCESS sProcess;
uint8_t RotateOK;
ret_val_t home_flag = 0;
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
extern uint32_t pre;
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
			sProcess.frameRear[1] = -sModule4Params.speed;
			sProcess.frameRear[2] = -sModule3Params.speed;
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
//	    if(0 == sModule1Params.targetAngle)
//	        sModule1Params.targetAngle = 1;
	    axis1.angle = sModule1Params.targetAngle;
//        if(0 == sModule2Params.targetAngle)
//            sModule2Params.targetAngle = 1;
        axis2.angle = sModule2Params.targetAngle;
//        if(0 == sModule3Params.targetAngle)
//            sModule3Params.targetAngle = 1;
        axis3.angle = sModule3Params.targetAngle;
//        if(0 == sModule4Params.targetAngle)
//            sModule4Params.targetAngle = 1;
        axis4.angle = sModule4Params.targetAngle;

//        if(sVehicalParams.Vy != 0 && sVehicalParams.Vx == 0 && sVehicalParams.W == 0 \
//                && axis1_chk == 1 && axis2_chk == 1 && axis3_chk == 1 && axis4_chk == 1)
//        {
//            axis1_chk = 0;
//            axis2_chk = 0;
//            axis3_chk = 0;
//            axis4_chk = 0;
//        }
         if(sVehicalParams.Vy != 0 && sVehicalParams.Vx == 0 && sVehicalParams.W == 0 \
                && home_flag != SUCCESSFUL)
         {
             home_flag = auto_home();
//              __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
//              __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, HOME_SPEED);
//
//              __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
//              __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, HOME_SPEED);
//
//              __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
//              __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, HOME_SPEED);
//
//              __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
//              __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, HOME_SPEED);
//
//
//              while((axis1_chk != 1) || (axis2_chk != 1) || (axis3_chk != 1) || (axis4_chk != 1))
//              {
//                  /*Check each motor if reaching end stop*/
//                  if(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_15) == 1)
//                  {
//                      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
//                      axis1_chk = 1;
//                  }
//                  if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_3) == 1)
//                  {
//                      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
//                      axis2_chk = 1;
//                  }
//                  if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_4) == 1)
//                  {
//                      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
//                      axis3_chk = 1;
//
//                  }
//                  if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_5) == 1)
//                  {
//                      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
//                      axis4_chk = 1;
//                  }

         }
                    pwm_handler(&htim3, &axis1, cnt1, CH1_CH2);
                    pwm_handler(&htim3, &axis2, cnt2, CH3_CH4);
                    pwm_handler(&htim4, &axis3, cnt3, CH1_CH2);
                    pwm_handler(&htim4, &axis4, cnt4, CH3_CH4);
//        if ( HAL_GetTick() - pre <= 10000)
//        {
//            pwm_handler(&htim3, &axis1, cnt1, CH1_CH2);
//            pwm_handler(&htim3, &axis2, cnt2, CH3_CH4);
//            pwm_handler(&htim4, &axis3, cnt3, CH1_CH2);
//            pwm_handler(&htim4, &axis4, cnt4, CH3_CH4);
//        }
//        else
//        {
//            axis1.pwm = 0;
//            axis2.pwm = 0;
//            axis3.pwm = 0;
//            axis4.pwm = 0;
//        }

		sProcess.process = UARThandler;
		break;
	default:
		sProcess.process = UARThandler;
		break;
	}
}

