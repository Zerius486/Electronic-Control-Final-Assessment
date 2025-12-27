#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include <stdint.h>

/**
 * @brief 遥控器对象结构体(不考虑键盘部分)
 */
typedef struct {
    int16_t ch0; // 右手左右
    int16_t ch1; // 右手上下
    int16_t ch2; // 左手上下
    int16_t ch3; // 左手左右
    uint8_t s1;  // 开关1
    uint8_t s2;  // 开关2
} RC_Info_t;

extern RC_Info_t rc_info;
extern float chassis_power_feedback;

void Gimbal_Init();
void Gimbal_Loop_Handler();
void Gimbal_RC_Callback(uint8_t *data, uint16_t len);
void Gimbal_Chassis_Callback(uint8_t *data, uint16_t len);

#endif