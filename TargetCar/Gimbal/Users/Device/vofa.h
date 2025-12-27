#ifndef VOFA_H
#define VOFA_H

#include "usart.h"

#define VOFA_UART_HANDLE  huart6

// 发送缓冲区大小
#define VOFA_TX_BUF_SIZE  128

extern float vofa_target_speed;

void Vofa_Init();
void Vofa_Send_CSV(float *data, uint8_t count);
void Vofa_Process_Command(uint8_t *buff, uint16_t len);

#endif // VOFA_H