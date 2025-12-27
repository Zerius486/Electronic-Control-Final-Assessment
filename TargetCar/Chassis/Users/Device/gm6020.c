#include "gm6020.h"

GM6020_Object_t motor_gimbal;

/**
 * @brief 初始化GM6020电机
 */
void GM6020_Init(void) {
    // 初始化云台电机
    motor_gimbal.can_id = 0x205; 
    
    // 初始化PID控制器
    PID_Init(&motor_gimbal.pid_speed, 
             150.0f,
             5.0f,
             0.0f,
             GM6020_MAX_VOLTAGE,
             5000.0f);
}

/**
 * @brief 解析GM6020电机数据
 *
 * @param motor GM6020电机对象结构体
 * @param data 数据指针
 */
void GM6020_Update_Data(GM6020_Object_t *motor, uint8_t *data) {
    if (motor == NULL || data == NULL) return;

    motor->angle_raw      = (data[0] << 8) | data[1];
    motor->speed_rpm      = (data[2] << 8) | data[3];
    motor->torque_current = (data[4] << 8) | data[5];
    motor->temperature    = data[6];
}

/**
 * @brief 向1-4号电调写入电压(只需为1号云台电机传入具体数据)
 *
 * @param hcan
 */
void GM6020_Send_Voltage(CAN_HandleTypeDef *hcan) {
    uint8_t tx_data[8] = {0};

    tx_data[0] = (motor_gimbal.give_voltage >> 8) & 0xFF;
    tx_data[1] = motor_gimbal.give_voltage & 0xFF;

    BSP_CAN_Send(hcan, 0x1FF, tx_data, 8);
}