/*
 * axis_driver.c
 *
 *  Created on: Jun 20, 2023
 *      Author: ADMIN
 */

#include "axis_driver.h"

const float Kp= 1;
const float  Ki = 0;
const float  Kd = 20;

int PID(float ref, float pitch, uint8_t pid_flag){
    float P = 0, I = 0, D = 0 , pid_pwm = 0;
    float lastError = 0;
    float error = 0;
    //calculate error
//    float a;
//    a = (float)(HAL_GetTick()/( 1.0e6 ));
//    curTime = HAL_GetTick();
//    dt = curTime - previousTime;
//    dt= 0.005;
    if(ref > pitch)
    {
        error = (ref - pitch);
    }
    else
    {
        error = (pitch - ref);
    }

    //calculate Proportional term
    P = Kp * error;

////    calculate Integral term. Account for wind-up
    static float i_err;
    i_err+= error;
    if(pid_flag==1)
        I+=Ki* i_err;
    else
        I+=0.5*i_err; // If the robot has to move the Ki term should be lower so there are less oscillation

    if (I > MAX_PWM)
        I = MAX_PWM;
    else if (I<MIN_PWM){
        I=MIN_PWM;
    }

    ////calculate Derivative term
    D = Kd * (error - lastError); //dt;
//    previousTime = HAL_GetTick();
    // If the robot has to move the control low is PI so the movement is more fluid
    if(pid_flag == 0){
        D = 0;
    }

    //total PID value
    pid_pwm = P + I + D;

    //max sure pwm is bound between allowed min/max thresholds

    int out_pwm = (int) (pid_pwm);
    if (pid_pwm > MAX_PWM)
        out_pwm = MAX_PWM;
    else if (pid_pwm < MIN_PWM)
        out_pwm = MIN_PWM;

    lastError = error;

    return out_pwm;

}
ret_val_t pwm_handler(TIM_HandleTypeDef *htim, M_axis_t *axis, uint16_t encoder_val)
{
    uint8_t ret_val = ERR;
    uint16_t pwm_val = 0;
    uint32_t chA = 0;
    uint32_t chB = 0;
//    axis->desired_value = (uint32_t)(axis->angle * ANGLE_CONVERT_VAL);

    /*PWM*/
    axis->pwm = (uint16_t)PID(axis->desired_value, encoder_val, 1);
//    axis->pwm = 400;
    /*Select channel*/
    if(0 == axis->channel_pin_set)
    {
        chA = TIM_CHANNEL_1;
        chB = TIM_CHANNEL_2;
    }
    else
    {
        chA = TIM_CHANNEL_3;
        chB = TIM_CHANNEL_4;
    }

//    HAL_TIM_PWM_Start (htim, chA);
//    HAL_TIM_PWM_Start (htim, chB);
    /*1300 ~ 360 degree*/
    if (axis->desired_value != 0)
    {
//        if(encoder_val > (axis->desired_value - CALIB_VAL))
//        {
//            HAL_GPIO_WritePin(GPIOB,axis_pin_num, RESET);
//            axis->desired_value = 0;
//            axis->angle = 0;
//            __HAL_TIM_SET_COUNTER(htim, 0);
//            ret_val = SUCCESSFUL;
//        }
//        else
//        {
//            HAL_GPIO_WritePin(GPIOB,axis_pin_num, SET);
//        }
        if(encoder_val < axis->desired_value)
        {
            __HAL_TIM_SET_COMPARE(htim, chA, 0);
            __HAL_TIM_SET_COMPARE(htim, chB, axis->pwm);
//            axis->dir = CW;
        }
        else
        {
            __HAL_TIM_SET_COMPARE(htim, chA, axis->pwm);
            __HAL_TIM_SET_COMPARE(htim, chB, 0);
//            axis->dir = CCW;
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









