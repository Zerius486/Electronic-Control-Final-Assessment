#include "m3508.h"

M3508_Object_t motor_m3508[4];

/**
 * @brief M3508电机初始化
 */
void M3508_Init_All() {
    for(int i=0; i<4; i++) {
        motor_m3508[i].angle_raw = 0;
        motor_m3508[i].speed_rpm = 0;
        motor_m3508[i].current_raw = 0;
        motor_m3508[i].give_current = 0;
    }
}

/**
 * @brief 解析并更新电机数据
 *
 * @param motor M3508对象结构体指针
 * @param data 电机原数据
 */
void M3508_Update_Data(M3508_Object_t *motor, uint8_t *data) {
    if (motor == NULL || data == NULL)
        return;
    motor->angle_raw   = (int16_t)((data[0] << 8) | data[1]);
    motor->speed_rpm   = (int16_t)((data[2] << 8) | data[3]);
    motor->current_raw = (int16_t)((data[4] << 8) | data[5]);
    motor->temperature = data[6];
}

/**
 * @brief 向1-4号电调写入电流
 *
 * @param hcan CAN句柄
 */
void M3508_Send_Group1(CAN_HandleTypeDef *hcan) {
    uint8_t tx_data[8]; // CAN 数据最多8字节

    // 1号电机
    tx_data[0] = (motor_m3508[0].give_current >> 8) & 0xFF;
    tx_data[1] = motor_m3508[0].give_current & 0xFF;

    // 2号电机
    tx_data[2] = (motor_m3508[1].give_current >> 8) & 0xFF;
    tx_data[3] = motor_m3508[1].give_current & 0xFF;

    // 3号电机
    tx_data[4] = (motor_m3508[2].give_current >> 8) & 0xFF;
    tx_data[5] = motor_m3508[2].give_current & 0xFF;

    // 4号电机
    tx_data[6] = (motor_m3508[3].give_current >> 8) & 0xFF;
    tx_data[7] = motor_m3508[3].give_current & 0xFF;

    BSP_CAN_Send(hcan, 0x200, tx_data, 8);
}
