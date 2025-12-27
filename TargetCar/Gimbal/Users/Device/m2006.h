#ifndef M2006_H
#define M2006_H

#include <stdint.h>
#include "bsp_can.h"
#include "alg_pid.h"

// M2006最大电流值(对应10.0fA)
#define M2006_CURRENT_LIMIT 10000.0f

/**
 * @brief M2006电机对象结构体
 */
typedef struct {
    uint32_t can_id;        // 反馈ID

    int16_t angle_raw;      // 机械角度(0-8191)
    int16_t speed_rpm;      // 转速
    int16_t current_raw;    // 瞬时转矩电流

    PID_Controller_t pid_speed; // 速度环
    int16_t give_current;       // 发送给电机的电流值

} M2006_Object_t;

extern M2006_Object_t motor_m2006[4];

void M2006_Init_All();
void M2006_Update_Data(M2006_Object_t *motor, uint8_t *data);
void M2006_Send_Group1(CAN_HandleTypeDef *hcan);

#endif