/*
 * axis_driver.c
 *
 *  Created on: Jun 20, 2023
 *      Author: ADMIN
 */

#include "axis_driver.h"

ret_val_t pwm_handler(TIM_HandleTypeDef *htim, M_axis_t *axis, uint16_t axis_pin_num, uint16_t encoder_val)
{
    uint8_t ret_val = ERR;
    axis->desired_value = (uint32_t)(axis->angle * ANGLE_CONVERT_VAL);
    /*PWM*/
    /*1300 ~ 360 degree*/
    if (axis->desired_value != 0)
    {
        if(encoder_val > (axis->desired_value - CALIB_VAL))
        {
            HAL_GPIO_WritePin(GPIOB,axis_pin_num, RESET);
            axis->desired_value = 0;
            axis->angle = 0;
            __HAL_TIM_SET_COUNTER(htim, 0);
            ret_val = SUCCESSFUL;
        }
        else
        {
            HAL_GPIO_WritePin(GPIOB,axis_pin_num, SET);
        }
    }
    return ret_val;
}
ret_val_t auto_home(uint16_t *axis_pin_num)
{
    /*Turn until receive home signal*/
    ret_val_t ret_val = ERR;
    uint8_t axis1_flag = 0;
    uint8_t axis2_flag = 0;
    uint8_t axis3_flag = 0;
    uint8_t axis4_flag = 0;
    int i = 0;
    for(i = 0; i < 4; i++)
    {
        HAL_GPIO_WritePin(GPIOB, *(axis_pin_num + i), SET);
    }
    while((axis1_flag != 1) || (axis2_flag != 1) || (axis3_flag != 1) || (axis4_flag != 1))
    {
        if(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_2) == 1)
        {
            HAL_GPIO_WritePin(GPIOB, *(axis_pin_num), RESET);
            axis1_flag = 1;
        }
        if(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_3) == 1)
        {
            HAL_GPIO_WritePin(GPIOB, *(axis_pin_num + 1), RESET);
            axis2_flag = 1;
        }
        if(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_4) == 1)
        {
            HAL_GPIO_WritePin(GPIOB, *(axis_pin_num + 2), RESET);
            axis3_flag = 1;

        }
        if(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_5) == 1)
        {
            HAL_GPIO_WritePin(GPIOB, *(axis_pin_num + 3), RESET);
            axis4_flag = 1;
        }
    }
    /*reset count value*/
    ret_val = SUCCESSFUL;
    return ret_val;
}









