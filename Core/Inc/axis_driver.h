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
typedef struct
{
    uint16_t pwm;
    uint16_t angle;
    uint32_t desired_value;
    _Bool channel_pin_set; /*0: ch1 ch2, 1: ch3 ch4*/
    dir_t dir; /*direction of motor*/
}M_axis_t;


#define ANGLE_CONVERT_VAL     (18.0555555555) /*   3.61111111111 = 1 round => 1:5 ratio ~ 18.0555555555*/
#define CALIB_VAL             (0U)


#define MAX_PWM 500
#define MIN_PWM 0


//ret_val_t pwm_handler(TIM_HandleTypeDef *htim, M_axis_t *axis, uint16_t axis_pin_num, uint16_t encoder_val);
ret_val_t auto_home(uint16_t *axis_pin_num);




#endif /* INC_AXIS_DRIVER_H_ */
