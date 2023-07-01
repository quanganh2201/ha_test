/*
 * axis_driver.h
 *
 *  Created on: Jun 20, 2023
 *      Author: ADMIN
 */

#ifndef INC_AXIS_DRIVER_H_
#define INC_AXIS_DRIVER_H_
#include "main.h"

typedef enum
{
    CW,
    CCW
}dir_t;
typedef enum
{
    ERR,
    SUCCESSFUL
}ret_val_t;
typedef enum
{
    CH1_CH2,
    CH3_CH4
}pwm_pin_set_t;
typedef struct
{
    uint16_t pwm;
    uint16_t angle;
    uint32_t desired_value;
    pwm_pin_set_t channel_pin_set; /*0: ch1 ch2, 1: ch3 ch4*/
    dir_t dir; /*direction of motor*/
}M_axis_t;

extern M_axis_t axis1;
extern M_axis_t axis2;
extern M_axis_t axis3;
extern M_axis_t axis4;
extern volatile uint32_t cnt1;
extern volatile uint32_t cnt2;
extern volatile uint32_t cnt3;
extern volatile uint32_t cnt4;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

#define ANGLE_CONVERT_VAL     (4.16666667) /*   3.61111111111 = 1 round => 1:5 ratio ~ 18.0555555555*/
#define CALIB_VAL             (0U)
#define MAX_PWM               (400U)
#define MIN_PWM               (0)
#define HOME_SPEED            (400U)
#define MIN_AXIS_VAL          (3U)

ret_val_t pwm_handler(TIM_HandleTypeDef *htim, M_axis_t *axis, uint16_t encoder_val, pwm_pin_set_t ch_pin_set);
ret_val_t auto_home(void);




#endif /* INC_AXIS_DRIVER_H_ */
