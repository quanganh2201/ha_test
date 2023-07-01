/*
 * DATN_Comm.c
 *
 *  Created on: Jun 10, 2023
 *      Author: longn
 */
#include"DATN_Comm.h"
#include "string.h"
S_SPI_RECEIVE s_SPI_handler;
S_UART_RECEIVE s_UART_handler;
extern UART_HandleTypeDef huart2;

void Init()
{
  HAL_UART_Receive_IT(&huart2, &s_UART_handler.Rx, 1);
//  HAL_SPI_Receive_IT(&SLAVE_SPI, &s_SPI_handler.Rx, 1);
}



//void dataProcessing()
//{
//  if(s_SPI_handler.dataValid == true)
//  {
//      s_SPI_handler.params[s_SPI_handler.index-1]
//  }
//}




void USART2_IRQHandler(void)
{

  HAL_UART_IRQHandler(&huart2);
  HAL_UART_Receive_IT(&huart2, &s_UART_handler.Rx, 1);

  switch(s_UART_handler.index)
  {
  case 0:
      if (s_UART_handler.Rx == 0x24)
        {
          s_UART_handler.index++;
          s_UART_handler.params[0]= s_UART_handler.Rx;
        }
      break;
  default:
          s_UART_handler.params[s_UART_handler.index++] = s_UART_handler.Rx;
      if(s_UART_handler.index >= 18 && s_UART_handler.params[16] == 0x0D && s_UART_handler.params[17] == 0x0A)// XXX: Thêm mới phần đuôi 0x0D vad 0x0A
      {
          s_UART_handler.dataValid = 1;
          s_UART_handler.index = 0;
      }
      break;
  }

}

void convert_hex2float(float *targetFloatVal, uint8_t* hexVal)
{
    union convert
    {
        float output;
        uint8_t input[4];
    } Uconvert;

    memcpy(&Uconvert.input[0],hexVal,4);
    memcpy(targetFloatVal,&Uconvert.output,4);
}

void convert_float2hex(float *fVal, uint8_t* targetHexVal)
{
    union convert
    {
        float input;
        uint8_t output[4];
    } Uconvert;

    memcpy(&Uconvert.input,targetHexVal,4);
    memcpy(&Uconvert.output[0],fVal,4);
}


/*
 * @brief:  Xu ly du lieu nhan ve tu UART
 * @note:   Tra ve gia tri float cho sResultData.Vx,sResultData.Vy,sResultData.W
 */
S_RESULT_DATA sResultData;
uint32_t pre;
//extern S_VEHICAL_PARAMS sVehicalParams;
void UART2_Handler()
{
    if (s_UART_handler.dataValid == 1 && s_UART_handler.params[0] == 0x24 && s_UART_handler.params[1] == 0x00)//s_UART_handler.params[1]= 0x00: che do bthg
    {
        memcpy(&sResultData.rawDatax[0],&s_UART_handler.params[VxOFFSET],4);
        memcpy(&sResultData.rawDatay[0],&s_UART_handler.params[VyOFFSET],4);
        memcpy(&sResultData.rawDataw[0],&s_UART_handler.params[WOFFSET],4);

        convert_hex2float(&sVehicalParams.Vx, &sResultData.rawDatax[0]);
        convert_hex2float(&sVehicalParams.Vy, &sResultData.rawDatay[0]);
        convert_hex2float(&sVehicalParams.W, &sResultData.rawDataw[0]);
        s_UART_handler.dataValid = 0;
        KinematicsHandler();
        pre = HAL_GetTick();
        memset(&s_UART_handler.params[0],'\0',16);
    }
    else if (s_UART_handler.dataValid == 1 && s_UART_handler.params[0] == 0x24 && s_UART_handler.params[1] == 0x01)//s_UART_handler.params[1]= 0x00: che do chi quay goc
    {
        memcpy(&sResultData.rawDatax[0],&s_UART_handler.params[VxOFFSET],4);
        memcpy(&sResultData.rawDatay[0],&s_UART_handler.params[VyOFFSET],4);
        memcpy(&sResultData.rawDataw[0],&s_UART_handler.params[WOFFSET],4);

        convert_hex2float(&sVehicalParams.Vx, &sResultData.rawDatax[0]);
        convert_hex2float(&sVehicalParams.Vy, &sResultData.rawDatay[0]);
        convert_hex2float(&sVehicalParams.W, &sResultData.rawDataw[0]);
        s_UART_handler.dataValid = 0;
        KinematicsHandler();
        sModule1Params.speed = 0;
        sModule2Params.speed = 0;
        sModule3Params.speed = 0;
        sModule4Params.speed = 0;
        pre = HAL_GetTick();
        memset(&s_UART_handler.params[0],'\0',16);
    }
}




