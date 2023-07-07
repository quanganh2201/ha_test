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
uint32_t DT = 0;
uint32_t previousTime = 0;
uint32_t msTicks = 0;
uint32_t curTime = 0;

void HAL_IncTick(void)  {                               /* SysTick interrupt Handler. */
//    if(msTicks > 10000) /*10s Toggle led */
//    {
//      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//      msTicks = 0;
//    }
    msTicks++;
}
uint16_t PID(float ref, float pitch, uint8_t pid_flag)
{
    float P = 0, I = 0, D = 0 , pid_pwm = 0;
    float lastError = 0;
    float error = 0;
    static float i_err = 0;
    uint16_t out_pwm = 0;
    //calculate error
    if(ref > pitch)
    {
        error = (ref - pitch);
    }
    else
    {
        error = (pitch - ref);
    }
    DT = msTicks - previousTime;
    //calculate Proportional term
    P = Kp * error;

////    calculate Integral term. Account for wind-up
    i_err += error * DT;
    if(pid_flag == 1)
        I = Ki* i_err;
    else
        I = 0.5*i_err; // If the robot has to move the Ki term should be lower so there are less oscillation

    if (I > MAX_PWM)
        I = MAX_PWM;
    else if (I<MIN_PWM){
        I = MIN_PWM;
    }

    ////calculate Derivative term
    D = Kd * (error - lastError)/ DT; //dt;
//    previousTime = HAL_GetTick();
    // If the robot has to move the control low is PI so the movement is more fluid
    if(pid_flag == 0){
        D = 0;
    }
    //total PID value
    pid_pwm = P + I + D;
    //max sure pwm is bound between allowed min/max thresholds
    out_pwm = (int)(pid_pwm);

    if (pid_pwm > MAX_PWM)
        out_pwm = MAX_PWM;
    else if (pid_pwm < MIN_PWM)
        out_pwm = MIN_PWM;
    previousTime = msTicks;
    lastError = error;

    return out_pwm;

}
ret_val_t pwm_handler(TIM_HandleTypeDef *htim, M_axis_t *axis, uint16_t encoder_val, pwm_pin_set_t ch_pin_set)
{
    uint8_t ret_val = ERR;
    uint32_t chA = 0;
    uint32_t chB = 0;
    /*Choose pin for pwm*/
    axis->channel_pin_set = ch_pin_set;
    /*Set desired value for the motor*/
    axis->desired_value = (uint32_t)(axis->angle * ANGLE_CONVERT_VAL);
    /*PWM*/
    axis->pwm = (uint16_t)PID(axis->desired_value, encoder_val, 1);
    /*Select channel*/
    if(0 == axis->channel_pin_set)
    {
        chA = TIM_CHANNEL_2;
        chB = TIM_CHANNEL_1;
    }
    else
    {
        chA = TIM_CHANNEL_4;
        chB = TIM_CHANNEL_3;
    }

    /*1500 ~ 360 degree => coefficient = 1500/360*/
//    if (axis->desired_value != 0)
//    {
        if(encoder_val < axis->desired_value)
        {
            __HAL_TIM_SET_COMPARE(htim, chA, 0);
            __HAL_TIM_SET_COMPARE(htim, chB, axis->pwm);
        }
        else
        {
            __HAL_TIM_SET_COMPARE(htim, chA, axis->pwm);
            __HAL_TIM_SET_COMPARE(htim, chB, 0);
        }

//    }
    return ret_val;
}
ret_val_t auto_home()
{
    /*Turn until receive home signal*/
    ret_val_t ret_val = ERR;
    uint8_t axis1_flag = 0;
    uint8_t axis2_flag = 0;
    uint8_t axis3_flag = 0;
    uint8_t axis4_flag = 0;
    int i = 0;
    /*Turn 4 motors till reaching end stop*/
    for(i = 0; i < HOME_SPEED; i+=20)
    {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, i);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, i);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, i);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, i);
    }

    while((axis1_flag != 1) || (axis2_flag != 1) || (axis3_flag != 1) || (axis4_flag != 1))
    {
        /*Check each motor if reaching end stop*/
        if(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_15) == 1)
        {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
            axis1_flag = 1;
        }
        if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_3) == 1)
        {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
            axis2_flag = 1;
        }
        if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_4) == 1)
        {
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
            axis3_flag = 1;

        }
        if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_5) == 1)
        {
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
            axis4_flag = 1;
        }
    }
    /*Blynk led to notice*/
    for(i = 0; i < 10; i++)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);
        HAL_Delay(150);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
        HAL_Delay(150);
    }
    /*reset count value*/
    ret_val = SUCCESSFUL;
    return ret_val;
}









