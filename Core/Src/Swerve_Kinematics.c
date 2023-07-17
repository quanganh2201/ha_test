/*
 * Swerve_Kinematics.c
 *
 *  Created on: May 18, 2023
 *      Author: longn
 */
#include "Swerve_Kinematics.h"
#include "math.h"
float Vel,Thelta;
float A,B,C,D;
//extern S_RESULT_DATA sResultData;
S_VEHICAL_PARAMS sVehicalParams;
S_MODULE_PARAMS sModule1Params;
S_MODULE_PARAMS sModule2Params;
S_MODULE_PARAMS sModule3Params;
S_MODULE_PARAMS sModule4Params;

int Swerve_reverseSpeedFlag[4] ;
int Swerve_angleOptimization(float inputAngle,uint8_t wheelN0)
{

    int suitableAngle;
    if((inputAngle >= -180) && (inputAngle<0))
    {
        suitableAngle = inputAngle + 180;
        Swerve_reverseSpeedFlag[wheelN0-1] = -1;
    }
    else
    {
        if(inputAngle==180)
        {
            suitableAngle = 0;
            Swerve_reverseSpeedFlag[wheelN0-1] = -1;
        }
        else
        {
            suitableAngle = inputAngle;
            Swerve_reverseSpeedFlag[wheelN0-1] = 1;
        }
        //sModule1Params.reverseVel
    }
    return (int)suitableAngle;
}


void Swerve_CoefitionCal(float Vx,float Vy, float w)
{
    float r;
    r = sqrt((BASE_LENGHT*BASE_LENGHT)+(BASE_WIDTH*BASE_WIDTH))/2;//0.376431
    A = Vx-w*(BASE_WIDTH/r);
    B = Vx+w*(BASE_WIDTH/r);/*Vy = 1, W = 0 => B = 0*/
    C = Vy-w*(BASE_LENGHT/r);/*C = 1*/
    D = Vy+w*(BASE_LENGHT/r);/*Vy = 1, => D = 1*/
}

float Swerve_Speed(uint8_t wheelN0)
{
    float tmp;
    switch(wheelN0)
    {
    case 1:
        if (sVehicalParams.Vx == 0 && sVehicalParams.Vy != 0 && sVehicalParams.W != 0)
        {
            tmp = A*A+D*D;

        }
        else if (sVehicalParams.Vy == 0 && sVehicalParams.Vx != 0 && sVehicalParams.W != 0)
        {
            tmp = A*A+D*D;
        }
        else
        {
            tmp = (B*B+C*C);
        }
        break;
    case 2:
        tmp = B*B+D*D;
        break;
    case 3:
        if (sVehicalParams.Vx == 0 && sVehicalParams.Vy != 0 && sVehicalParams.W != 0)
        {

            tmp = (B*B+C*C);
        }
        else if (sVehicalParams.Vy == 0 && sVehicalParams.Vx != 0 && sVehicalParams.W != 0)
        {
            tmp = (B*B+C*C);
        }
        else
            {
            tmp = A*A+D*D;
            }
        break;
    case 4:
        tmp = A*A+C*C;
        break;
    }//return (sqrt(tmp));
    return (sqrt(tmp)*Swerve_reverseSpeedFlag[wheelN0-1]);
}

int Swerve_Angle(uint8_t wheelN0)
{
    float tmp;
    switch(wheelN0)
    {
    case 1:
        if (sVehicalParams.Vx == 0 && sVehicalParams.Vy != 0 && sVehicalParams.W != 0)
        {

            tmp = atan2(A,D)*180/PI;// OK
        }
        else if (sVehicalParams.Vy == 0 && sVehicalParams.Vx != 0 && sVehicalParams.W != 0)
        {
            tmp = atan2(A,D)*180/PI;//OK
        }
        else if (sVehicalParams.Vx == 0 && sVehicalParams.Vy == 0 && sVehicalParams.W >=0)
        {
            tmp = ((atan2(B,C))*180/PI)-180;//OK
        }
        else if (sVehicalParams.Vx == 0 && sVehicalParams.Vy == 0 && sVehicalParams.W <=0)
        {
            tmp = (atan2(B,C)*180/PI)+180;//OK
        }
        else if (sVehicalParams.Vx != 0 && sVehicalParams.Vy != 0&& sVehicalParams.W != 0)
        {
            tmp = atan2(A,D)*180/PI;//OK
        }
        else
        {
            tmp = atan2(B,C)*180/PI;/*Vy = 0=> temp = 0*/
        }
        break;
    case 2:
        tmp = atan2(B,D)*180/PI;
        break;
    case 3:
        if (sVehicalParams.Vx == 0 && sVehicalParams.Vy != 0 && sVehicalParams.W != 0 )
        {
            tmp = (atan2(B,C))*180/PI;//OK
        }
        else if (sVehicalParams.Vy == 0 && sVehicalParams.Vx != 0 )
        {
            tmp = (atan2(B,C))*180/PI;//OK
        }
        else if (sVehicalParams.Vx == 0 && sVehicalParams.Vy == 0 && sVehicalParams.W >=0)
        {
            tmp = (atan2(A,D)*180/PI)+180;//OK
        }
        else if (sVehicalParams.Vx == 0 && sVehicalParams.Vy == 0 && sVehicalParams.W <=0)
        {
            tmp = (atan2(A,D)*180/PI)-180;//OK
        }
        else if (sVehicalParams.Vx != 0 && sVehicalParams.Vy != 0 && sVehicalParams.W != 0)
        {
            tmp = (atan2(B,C))*180/PI;//OK
        }
        else
        {
            tmp = atan2(A,D)*180/PI;
        }
        break;
    case 4:
        tmp = atan2(A,C)*180/PI;
        break;
    }//return ((int)tmp);
    return (Swerve_angleOptimization(tmp,wheelN0));
}




void KinematicsHandler()
{
    Swerve_CoefitionCal(sVehicalParams.Vx,sVehicalParams.Vy,sVehicalParams.W);
    if(sVehicalParams.Vx == 0 && sVehicalParams.Vy == 0 && sVehicalParams.W == 0)
    {
        sModule1Params.targetAngle = 90;
        sModule2Params.targetAngle = 90;
        sModule3Params.targetAngle = 90;
        sModule4Params.targetAngle = 90;
    }
    else
    {
        sModule1Params.targetAngle = Swerve_Angle(1)&0xFF;
        sModule2Params.targetAngle = Swerve_Angle(2)&0xFF;
        sModule3Params.targetAngle = Swerve_Angle(3)&0xFF;
        sModule4Params.targetAngle = Swerve_Angle(4)&0xFF;

    }

    sModule1Params.reverseVel = Swerve_reverseSpeedFlag[1-1];
    sModule2Params.reverseVel = Swerve_reverseSpeedFlag[2-1];
    sModule3Params.reverseVel = Swerve_reverseSpeedFlag[3-1];
    sModule4Params.reverseVel = Swerve_reverseSpeedFlag[4-1];

    sModule1Params.speed = (int)(Swerve_Speed(1)* M2RAD)&0xFFFF;
    sModule2Params.speed = (int)(Swerve_Speed(2)* M2RAD)&0xFFFF;
    sModule3Params.speed = (int)(Swerve_Speed(3)* M2RAD)&0xFFFF;
    sModule4Params.speed = (int)(Swerve_Speed(4)* M2RAD)&0xFFFF;


    // Limit
    sModule1Params.speed = (sModule1Params.speed > 1000) ? 1000 : sModule1Params.speed;
    sModule2Params.speed = (sModule2Params.speed > 1000) ? 1000 : sModule2Params.speed;
    sModule3Params.speed = (sModule3Params.speed > 1000) ? 1000 : sModule3Params.speed;
    sModule4Params.speed = (sModule4Params.speed > 1000) ? 1000 : sModule4Params.speed;

    sModule1Params.speed = (sModule1Params.speed < -1000) ? -1000 : sModule1Params.speed;
    sModule2Params.speed = (sModule2Params.speed < -1000) ? -1000 : sModule2Params.speed;
    sModule3Params.speed = (sModule3Params.speed < -1000) ? -1000 : sModule3Params.speed;
    sModule4Params.speed = (sModule4Params.speed < -1000) ? -1000 : sModule4Params.speed;

}



