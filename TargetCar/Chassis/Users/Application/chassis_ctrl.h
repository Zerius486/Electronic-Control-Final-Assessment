#ifndef CHASSIS_CTRL_H
#define CHASSIS_CTRL_H

#include "m3508.h"
#include <stdint.h>

// 轮径(m)
#define CHASSIS_WHEEL_RADIUS  0.055f
// 对角距(m)
#define CHASSIS_WHEEL_HALF_DISTANCE  0.400f
// 底盘轮子减速比
#define MOTOR_REDUCTION_RATIO 19.2f

// 速度限幅
#define VX_MAX 1.0f
#define VY_MAX 1.0f
#define WZ_MAX 5.0f

// 摇杆通道半长
#define DT7_CHANNEl_HALFLEN 660.0f

/**
 * @brief 底盘控制对象结构体
 */
typedef struct {
    float vx; // 前后运动速度 (m/s)
    float vy; // 左右平移速度 (m/s)
    float wz; // 原地旋转角速度 (rad/s)

    float total_power_w; // 估算的总功率(W)
} Chassis_Ctrl_t;

extern Chassis_Ctrl_t chassis_ctrl;
extern M3508_Object_t motor_m3508[4];

void Chassis_Init();
void Chassis_Loop_Handler();
void Chassis_Rx_Callback(uint8_t *data, uint16_t len);
void Chassis_Send_Status();

#endif // CHASSIS_CTRL_H