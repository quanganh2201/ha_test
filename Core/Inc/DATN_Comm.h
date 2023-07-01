/*
 * DATN_Comm.h
 *
 *  Created on: Jun 10, 2023
 *      Author: longn
 */

#ifndef INC_DATN_COMM_H_
#define INC_DATN_COMM_H_

#include "stm32f4xx_hal.h"
#include "main.h"
#include "stdbool.h"
#include "Swerve_Kinematics.h"

#define VxOFFSET 4
#define VyOFFSET 8
#define WOFFSET (uint8_t)12

#define MODULE_N0_01
/*
 *    02--------01
 *    |			|
 *    03--------04
 */

#define SLAVE_SPI hspi2

typedef enum
{
	params,
	start,
	stop
} E_RECEIVE_STATE;

typedef struct
{
	uint8_t volatile index;
	uint8_t Rx;
	E_RECEIVE_STATE action;
	bool dataValid;
	uint8_t params[12];
} S_SPI_RECEIVE;

typedef struct
{
	uint16_t speed;
	uint8_t  angle;
} S_DATA;


typedef struct
{
	uint8_t volatile index;
	uint8_t Rx;
	E_RECEIVE_STATE action;
	bool dataValid;
	uint8_t params[18];
} S_UART_RECEIVE;

typedef struct
{
	uint8_t rawDatax[4],rawDatay[4],rawDataw[4];
	float Vx,Vy,W;
} S_RESULT_DATA;
extern S_RESULT_DATA sResultData;


void UART2_Handler();

#endif /* INC_DATN_COMM_H_ */
