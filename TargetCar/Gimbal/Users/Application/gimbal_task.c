#include "gimbal_task.h"
#include "gm6020.h"
#include "vofa.h"
#include "bsp_uart.h"
#include "usart.h"
#include <string.h>
#include <stdlib.h>

// 遥控器实例
RC_Info_t rc_info;

// 功率反馈值
float chassis_power_feedback = 0.0f;

// 引用外部句柄
extern UART_HandleTypeDef huart6; // 连接底盘
extern CAN_HandleTypeDef hcan1;   // 连接云台电机

// 小陀螺模式标志位
static uint8_t spin_mode = 0;

/**
 * @brief 云台初始化
 */
void Gimbal_Init() {
    GM6020_Init();
    Vofa_Init();
    memset(&rc_info, 0, sizeof(rc_info));
}

/**
 * @brief 遥控器接收回调函数
 *
 * @param buff 接收缓冲区
 * @param len 数据长度
 */
void Gimbal_RC_Callback(uint8_t *buff, uint16_t len) {
    if (buff == NULL || len != 18) {
        return;
    }

    // 遥控器数据解析
    rc_info.ch0 = ((buff[0] | (buff[1] << 8)) & 0x07ff) - 1024;
    rc_info.ch1 = (((buff[1] >> 3) | (buff[2] << 5)) & 0x07ff) - 1024;
    rc_info.ch2 = (((buff[2] >> 6) | (buff[3] << 2) | (buff[4] << 10)) & 0x07ff) - 1024;
    rc_info.ch3 = (((buff[4] >> 1) | (buff[5] << 7)) & 0x07ff) - 1024;
    rc_info.s1 = ((buff[5] >> 4) & 0x000C) >> 2;
    rc_info.s2 = ((buff[5] >> 4) & 0x0003);

    // 加入±5的死区防止数据零漂
    if(rc_info.ch0 <= 10 && rc_info.ch0 >= -10) {
        rc_info.ch0 = 0;
    }
    if(rc_info.ch1 <= 10 && rc_info.ch1 >= -10) {
        rc_info.ch1 = 0;
    }
    if(rc_info.ch2 <= 10 && rc_info.ch2 >= -10) {
        rc_info.ch2 = 0;
    }
    if(rc_info.ch3 <= 10 && rc_info.ch3 >= -10) {
        rc_info.ch3 = 0;
    }

    // 防止数据溢出
    if ((abs(rc_info.ch0) > 660) || (abs(rc_info.ch1) > 660) || (abs(rc_info.ch2) > 660) || (abs(rc_info.ch3) > 660)) {
        memset(&rc_info, 0, sizeof(rc_info));
        return ;
    }

    // 小陀螺模式判断
    if (rc_info.s1 == 2) {
        spin_mode = 1;
    }
    else {
        spin_mode = 0;
    }

    // 板间通讯
    uint8_t tx_buf[8];
    tx_buf[0] = 0xA5;

    int16_t vx = rc_info.ch1;
    int16_t vy = rc_info.ch0;
    int16_t wz = rc_info.ch3;

    if (spin_mode) {
        wz = 400;
    }

    tx_buf[1] = vx & 0xFF;
    tx_buf[2] = (vx >> 8) & 0xFF;
    tx_buf[3] = vy & 0xFF;
    tx_buf[4] = (vy >> 8) & 0xFF;
    tx_buf[5] = wz & 0xFF;
    tx_buf[6] = (wz >> 8) & 0xFF;
    tx_buf[7] = 0x5A;

    BSP_UART_Send(&huart6, tx_buf, 8);
}

/**
 * @brief 底盘数据接收回调函数
 *
 * @param data 数据指针
 * @param len 数据长度
 */
void Gimbal_Chassis_Callback(uint8_t *data, uint16_t len) {
    if (data == NULL || len < 4) return;

    if (data[0] == 0xA6 && data[3] == 0x6A) {
        int16_t p_int = (data[1] | (data[2] << 8));
        chassis_power_feedback = (float)p_int / 10.0f;
    }
}

/**
 * @brief 云台循环控制函数
 */
void Gimbal_Loop_Handler() {
    float final_target_speed = vofa_target_speed;

    if (spin_mode) {
        final_target_speed += 50.0f;
    }

    float pid_out = PID_Calculate(&motor_gimbal.pid_speed, final_target_speed, (float)motor_gimbal.speed_rpm);
    motor_gimbal.give_current = (int16_t)pid_out;

    GM6020_Send_Current_Group1(&hcan1);

    // 给上位机发送波形
    float waves[3];
    waves[0] = final_target_speed;
    waves[1] = (float)motor_gimbal.speed_rpm;
    waves[2] = chassis_power_feedback;;

    Vofa_Send_CSV(waves, 3);
}