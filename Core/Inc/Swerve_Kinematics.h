/*
 * Swerve_Kinematics.h
 *
 *  Created on: May 18, 2023
 *      Author: longn
 */

#ifndef INC_SWERVE_KINEMATICS_H_
#define INC_SWERVE_KINEMATICS_H_

#include "main.h"

#define PI (float)3.14159265359
#define WHEEL_RADI (float) 0.1651
#define BASE_LENGHT (float) 0.58 //in metter
#define BASE_WIDTH (float) 0.48
#define M2RAD (float)(60/(2*PI*WHEEL_RADI))

#define COE_A(Vx,w)		Vx-w*(BASE_LENGHT/2)
#define COE_B(Vx,w)		Vx+w*(BASE_LENGHT/2)
#define COE_C(Vy,w)		Vy-w*(BASE_WIDTH/2)
#define COE_D(Vy,w)		Vy+w*(BASE_WIDTH/2)



typedef struct
{
	float Vx,Vy,W;
} S_VEHICAL_PARAMS;

typedef struct
{
	uint8_t currentAngle;
	int targetAngle;
	int16_t speed;
	int reverseVel;
} S_MODULE_PARAMS;

extern S_VEHICAL_PARAMS sVehicalParams;
extern S_MODULE_PARAMS sModule1Params;
extern S_MODULE_PARAMS sModule2Params;
extern S_MODULE_PARAMS sModule3Params;
extern S_MODULE_PARAMS sModule4Params;

void Swerve_CoefitionCal(float Vx,float Vy, float w);
float Swerve_Speed(uint8_t wheelN0);
int Swerve_Angle(uint8_t wheelN0);
int Swerve_angleOptimization(float inputAngle,uint8_t Swerve_reverseSpeedFlag);
void KinematicsHandler();

#endif /* INC_SWERVE_KINEMATICS_H_ */
