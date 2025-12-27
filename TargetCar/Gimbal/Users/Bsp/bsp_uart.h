#ifndef BSP_UART_H
#define BSP_UART_H

#include "usart.h"

// 接收缓冲区大小
#define UART_RX_BUF_SIZE 256

/**
 * @brief UART接收回调函数指针
 */
typedef void (*BSP_UART_RxCallback_t)(uint8_t *data, uint16_t len);

/**
 * @brief UART通信对象结构体
 */
typedef struct {
    UART_HandleTypeDef *huart;        // HAL库句柄
    uint8_t rx_buffer[UART_RX_BUF_SIZE]; // 接收缓冲区
    BSP_UART_RxCallback_t callback;   // 接收回调函数指针
} BSP_UART_Object_t;

void BSP_UART_Init(UART_HandleTypeDef *huart, BSP_UART_RxCallback_t callback);
void BSP_UART_Send(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len);

#endif