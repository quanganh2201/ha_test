/*
 * main_process.h
 *
 *  Created on: Jun 18, 2023
 *      Author: longn
 */

#ifndef INC_MAIN_PROCESS_H_
#define INC_MAIN_PROCESS_H_
#include "main.h"
#include "Swerve_Kinematics.h"
#include"DATN_Comm.h"



typedef struct
{
	uint16_t speed[4];// min abs(1000)
	uint8_t angle[4];// 180------- 0  (positive)->

} S_FINAL_DATA;

typedef struct
{
	uint8_t process;
	uint16_t frameFront[4];
	uint16_t frameRear[4];
	uint8_t FlagSendBLDC;
	uint8_t directionFlag;
} S_PROCESS;

enum task_stt
{
	UARThandler,
	sendBLDC,
	adjustAngle
};

typedef enum
{
	SET1,
	RESET1
} FlagStatus1;


void main_process();
#endif /* INC_MAIN_PROCESS_H_ */
