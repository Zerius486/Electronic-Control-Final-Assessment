#include "bsp_uart.h"
#include <string.h>

static BSP_UART_Object_t uart1_obj = {0}; // 用于上位机通讯
static BSP_UART_Object_t uart3_obj = {0}; // 用于遥控器接收
static BSP_UART_Object_t uart6_obj = {0}; // 用于底盘通信

/**
 * @brief 查询UART通信对象
 *
 * @param huart
 */
static BSP_UART_Object_t* UART_Get_Obj(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) return &uart1_obj;
    if (huart->Instance == USART3) return &uart3_obj;
    if (huart->Instance == USART6) return &uart6_obj;
    return NULL;
}

/**
 * @brief UART通信初始化
 *
 * @param huart UART句柄
 * @param callback 回调函数指针
 */
void BSP_UART_Init(UART_HandleTypeDef *huart, BSP_UART_RxCallback_t callback) {
    BSP_UART_Object_t *obj = UART_Get_Obj(huart);
    if (obj == NULL) return;

    obj->huart = huart;
    obj->callback = callback;

    // 启动空闲中断DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(huart, obj->rx_buffer, UART_RX_BUF_SIZE);
}

/**
 * @brief UART阻塞发送
 *
 * @param huart UART句柄
 * @param data 数据指针
 * @param len 数据长度
 */
void BSP_UART_Send(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len) {
    HAL_UART_Transmit(huart, data, len, 10);
}

/**
 * @brief HAL库UART接收回调函数
 *
 * @param huart UART句柄
 * @param Size 数据大小
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    BSP_UART_Object_t *obj = UART_Get_Obj(huart);

    if (obj != NULL) {
        // 调用回调函数
        if (obj->callback != NULL) {
            obj->callback(obj->rx_buffer, Size);
        }

        // 重新开启接收
        HAL_UARTEx_ReceiveToIdle_DMA(huart, obj->rx_buffer, UART_RX_BUF_SIZE);
    }
}

/**
 * @brief UART错误回调函数
 *
 * @param huart UART句柄
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    BSP_UART_Object_t *obj = UART_Get_Obj(huart);
    if (obj != NULL) {
        HAL_UART_AbortReceive(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, obj->rx_buffer, UART_RX_BUF_SIZE);
    }
}