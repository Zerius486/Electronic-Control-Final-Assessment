#ifndef M3508_H
#define M3508_H

#include "bsp_can.h"
#include "alg_pid.h"

// M3508最大电流值(对应20.0fA)
#define M3508_CURRENT_LIMIT 16384.0f

/**
 * @brief M3508电机对象结构体
 */
typedef struct {
    uint32_t can_id;        // 反馈ID

    int16_t angle_raw;      // 机械角度(0-8191)
    int16_t speed_rpm;      // 转速
    int16_t current_raw;    // 瞬时转矩电流
    uint8_t temperature;    // 温度

    PID_Controller_t pid_speed; // 速度环
    int16_t give_current;       // 发送给电机的电流值

} M3508_Object_t;

void M3508_Init_All();
void M3508_Update_Data(M3508_Object_t *motor, uint8_t *data);
void M3508_Send_Group1(CAN_HandleTypeDef *hcan);

#endif