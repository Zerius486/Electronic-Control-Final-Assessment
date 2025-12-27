#include "chassis_ctrl.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "usart.h"
#include "alg_pid.h"
#include <math.h>
#include <stdlib.h>

// 底盘控制对象实例
Chassis_Ctrl_t chassis_ctrl;

// 引用外部句柄
extern CAN_HandleTypeDef hcan1;      // 控制电机
extern UART_HandleTypeDef huart6;    // 云台通信

/**
 * @brief 线速度转换为电机转子转速 (RPM)
 *
 * @param speed_m_s 轮子表面线速度 (m/s)
 */
static int16_t Chassis_Speed_To_MotorRPM(float speed_m_s) {
    float wheel_rpm = speed_m_s * 60.0f / (2.0f * 3.14159f * CHASSIS_WHEEL_RADIUS);
    return (int16_t)(wheel_rpm * MOTOR_REDUCTION_RATIO);
}

/**
 * @brief 底盘初始化
 */
void Chassis_Init() {
    M3508_Init_All(); // 初始化电机

    // 给轮组电机的PID控制器设置参数
    for(int i=0; i<4; i++) {
        PID_Init(&motor_m3508[i].pid_speed,
                 10.0f,
                 0.1f,
                 0.0f,
                 16384.0f,
                 2000.0f);

        motor_m3508[i].can_id = 0x201 + i; // 绑定ID
    }

    // 初始状态停车
    chassis_ctrl.vx = 0;
    chassis_ctrl.vy = 0;
    chassis_ctrl.wz = 0;
}

/**
 * @brief 解析云台数据包
 *
 * @note 协议格式: 0xA5 [Vx_L] [Vx_H] [Vy_L] [Vy_H] [Wz_L] [Wz_H] 0x5A
 */
void Chassis_Rx_Callback(uint8_t *data, uint16_t len) {
    // 检查数据包大小
    if (data == NULL || len < 8) return;

    // 检查帧头
    if (data[0] == 0xA5 && data[7] == 0x5A) {

        int16_t vx_i = (int16_t)(data[1] | (data[2] << 8));
        int16_t vy_i = (int16_t)(data[3] | (data[4] << 8));
        int16_t wz_i = (int16_t)(data[5] | (data[6] << 8));

        // 速度映射
        chassis_ctrl.vx = ((float)vx_i / DT7_CHANNEl_HALFLEN) * VX_MAX;
        chassis_ctrl.vy = -((float)vy_i / DT7_CHANNEl_HALFLEN) * VY_MAX;
        chassis_ctrl.wz = ((float)wz_i / DT7_CHANNEl_HALFLEN) * WZ_MAX;
    }
}

/**
 * @brief 回传底盘功率给云台
 *
 * @note 通过电流估算功率：P = U * I
 */
void Chassis_Send_Status() {
    // 累加所有电机的电流绝对值
    float total_current_abs = 0.0f;
    for(int i=0; i<4; i++) {
        float current_real = (float)abs(motor_m3508[i].current_raw) * 20.0f / 16384.0f;
        total_current_abs += current_real;
    }

    // 假设电池电压是24V
    chassis_ctrl.total_power_w = 24.0f * total_current_abs;

    // 打包数据发送
    // 协议: 0xA6 + 功率 + 0x6A
    uint8_t tx_buf[4];
    // 放大十倍变为整数发送
    int16_t power_int = (int16_t)(chassis_ctrl.total_power_w * 10.0f);

    tx_buf[0] = 0xA6; // 帧头
    tx_buf[1] = power_int & 0xFF;
    tx_buf[2] = (power_int >> 8) & 0xFF;
    tx_buf[3] = 0x6A; // 帧尾

    BSP_UART_Send(&huart6, tx_buf, 4);
}

/**
 * @brief 底盘循环控制函数
 *
 * @note 这里使用全向轮底盘，直接写速度变换，后续想要提升运行速度可以使用arm_math.h库中的矩阵运算进行加速
 */
void Chassis_Loop_Handler() {
    float vx = chassis_ctrl.vx;
    float vy = chassis_ctrl.vy;
    float wz = chassis_ctrl.wz;

    // 几何参数 K = 全向轮中心到车体中心的距离, 默认使用正方形布局
    float k = 0.5f * CHASSIS_WHEEL_HALF_DISTANCE;

    // 全向轮逆运动学解算
    float v_wheel[4];
    float angles[4] = {45.0f, 135.0f, 225.0f, 315.0f};
    float angle_rad;
    for(int i = 0; i < 4; i++) {
        angle_rad = angles[i] * M_PI / 180.0f;
        // 计算轮子速度
        v_wheel[i] = -vx * sinf(angle_rad) + vy * cosf(angle_rad) + k * wz;
    }

    // 计算PID
    for(int i=0; i<4; i++) {
        // 目标转速
        float target_rpm = (float)Chassis_Speed_To_MotorRPM(v_wheel[i]);

        // 实际转速
        float current_rpm = (float)motor_m3508[i].speed_rpm;

        // PID计算
        motor_m3508[i].give_current = (int16_t)PID_Calculate(&motor_m3508[i].pid_speed, target_rpm, current_rpm);
    }

    // 发送控制指令
    M3508_Send_Group1(&hcan1);
}