#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "can.h"

/**
 * @brief BSP_CAN回调函数指针
 */
typedef void (*BSP_CAN_RxCallback_t)(CAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t *data, uint32_t len);

/**
 * @brief CAN通信对象结构体
 */
typedef struct {
    CAN_HandleTypeDef *hcan;
    BSP_CAN_RxCallback_t callback; // 项目使用的回调函数
} BSP_CAN_Object_t;

void BSP_CAN_Init(CAN_HandleTypeDef *hcan, BSP_CAN_RxCallback_t callback);
uint8_t BSP_CAN_Send(CAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t *data, uint16_t len);

#endif