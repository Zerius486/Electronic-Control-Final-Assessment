#include "bsp_can.h"

static BSP_CAN_Object_t can1_obj = {0};
static BSP_CAN_Object_t can2_obj = {0};

/**
 * @brief 配置CAN过滤器（接收所有标准帧）
 *
 * @param hcan CAN句柄
 * @note 接收所有标准帧，然后在软件层通过switch-case分发
 */
static void CAN_Filter_Config_All(CAN_HandleTypeDef *hcan) {
    CAN_FilterTypeDef filter;

    filter.FilterIdHigh = 0;
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = 0;
    filter.FilterMaskIdLow = 0;

    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterBank = (hcan->Instance == CAN1) ? 0 : 14; // CAN1用0-13, CAN2用14-27
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(hcan, &filter);
}

/**
 * @brief 初始化CAN
 *
 * @param hcan CAN句柄
 * @param callback 回调函数指针
 */
void BSP_CAN_Init(CAN_HandleTypeDef *hcan, BSP_CAN_RxCallback_t callback) {
    BSP_CAN_Object_t *obj = (hcan->Instance == CAN1) ? &can1_obj : &can2_obj;

    obj->hcan = hcan;
    obj->callback = callback;

    CAN_Filter_Config_All(hcan);
    HAL_CAN_Start(hcan);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief CAN 发送
 */
uint8_t BSP_CAN_Send(CAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t *data, uint16_t len) {
    CAN_TxHeaderTypeDef tx_header;
    uint32_t mailbox;

    tx_header.StdId = std_id;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = len;

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0) {
        return HAL_CAN_AddTxMessage(hcan, &tx_header, data, &mailbox);
    }
    return HAL_BUSY;
}

/**
 * @brief HAL库CAN_FIFO0中断
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    BSP_CAN_Object_t *obj = (hcan->Instance == CAN1) ? &can1_obj : &can2_obj;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        if (obj->callback != NULL) {
            obj->callback(hcan, rx_header.StdId, rx_data, rx_header.DLC);
        }
    }
}
