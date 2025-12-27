#include "vofa.h"
#include "bsp_uart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern UART_HandleTypeDef VOFA_UART_HANDLE;

float vofa_target_speed = 0.0f;

void Vofa_Init() {
    vofa_target_speed = 100.0f;
}

/**
 * @brief 发送CSV格式数据
 *
 * @param data 数据指针
 * @param count 发送通道数
 */
void Vofa_Send_CSV(float *data, uint8_t count) {
    if (count == 0) return;

    char tx_buf[VOFA_TX_BUF_SIZE];
    int offset = 0;
    int remaining = VOFA_TX_BUF_SIZE;

    // 遍历数组，拼装字符串
    for (uint8_t i = 0; i < count; i++) {
        // 格式化浮点数，保留2位小数
        int len = snprintf(tx_buf + offset, remaining, "%.2f%c",
                           data[i],
                           (i == count - 1) ? '\n' : ',');

        if (len < 0 || len >= remaining) break; // 防止缓冲区溢出

        offset += len;
        remaining -= len;
    }

    // 通过串口发送拼装好的字符串
    if (offset > 0) {
        BSP_UART_Send(&VOFA_UART_HANDLE, (uint8_t*)tx_buf, offset);
    }
}

/**
 * @brief 解析指令
 */
void Vofa_Process_Command(uint8_t *buff, uint16_t len) {
    if (buff == NULL || len < 3) return;

    // 简单遍历寻找 "S="
    for (uint16_t i = 0; i < len - 2; i++) {
        if (buff[i] == 'S' && buff[i+1] == '=') {
            vofa_target_speed = (float)atof((char*)&buff[i+2]);
            return;
        }
    }
}