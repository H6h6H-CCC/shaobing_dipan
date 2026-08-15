#include "bsp_can.h"

HAL_StatusTypeDef BSP_CAN_Init(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef filter = {0};

    if (hcan == NULL)
    {
        return HAL_ERROR;
    }

    /* 调试阶段接收 CAN1 上的全部标准帧，设备层再按 ID 分发。 */
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0;
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = 0;
    filter.FilterMaskIdLow = 0;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}
