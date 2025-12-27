#ifndef GM6020_H
#define GM6020_H

#include <stdint.h>
#include "bsp_can.h"
#include "alg_pid.h"

// GM6020最大电流值(对应3.0fA)
#define GM6020_MAX_CURRENT 16384.0f

/**
 * @brief GM6020电机对象结构体
 */
typedef struct {
    uint32_t can_id;        // 反馈ID

    int16_t angle_raw;      // 机械角度(0-8191)
    int16_t speed_rpm;      // 转速
    int16_t current_raw;    // 瞬时转矩电流
    uint8_t temperature;

    PID_Controller_t pid_speed; // 速度环
    PID_Controller_t pid_angle; // 位置环
    
    int16_t give_current;       // 发送给电机的电流值

} GM6020_Object_t;

extern GM6020_Object_t motor_gimbal;

void GM6020_Init();
void GM6020_Update_Data(GM6020_Object_t *motor, uint8_t *data);
void GM6020_Send_Current_Group1(CAN_HandleTypeDef *hcan);

#endif