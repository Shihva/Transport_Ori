#include "bsp_can.h"

ZDT_Motor_Info_t motor_info[9];


static void CAN1_Rx0Callback(CAN_RxHeaderTypeDef* rx_header, uint8_t* rx_data);
//static void CAN2_Rx0Callback(CAN_RxHeaderTypeDef* rx_header, uint8_t* rx_data);

void Motor_Can_Init()
{
	uint8_t tx_data[6];
	tx_data[0] = 0x11;                     // 功能码[cite: 8]
    tx_data[1] = 0x18;                     // 辅助码 (定时返回信息)[cite: 8]
    tx_data[2] = 0x36;                     // 信息功能码: 0x36 表示读取实时位置[cite: 8]
    tx_data[3] = (uint8_t)(INTERVAL_MS >> 8);   // 定时时间高字节 (毫秒)[cite: 8]
    tx_data[4] = (uint8_t)(INTERVAL_MS & 0xFF);  // 定时时间低字节 (毫秒)[cite: 8]
    tx_data[5] = 0x6B;
	for(uint8_t id = 5;id<7;id++)
	{
		set_can_ctrl(&hcan1, id, tx_data, sizeof(tx_data));
	}
}

void can_user_init(CAN_HandleTypeDef* hcan )
{
  CAN_FilterTypeDef  can_filter;
  can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0;
  can_filter.FilterIdLow  = 0;
  can_filter.FilterMaskIdHigh = 0;
  can_filter.FilterMaskIdLow  = 0; // Accept all IDs
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter.FilterActivation = ENABLE;
  can_filter.SlaveStartFilterBank  = 14;
  
  if (hcan->Instance == CAN1)
  {
    can_filter.FilterBank = 0;
  }
  else //CAN2
  {
    can_filter.FilterBank = 14;
  }

  HAL_CAN_ConfigFilter(hcan, &can_filter);
  HAL_CAN_Start(hcan);
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  
  Motor_Can_Init();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t             rx_data[8];

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

  if (hcan->Instance == CAN1)
	{
			CAN1_Rx0Callback(&rx_header, rx_data);
	}
//  else if (hcan->Instance == CAN2)
//	{
//			CAN2_Rx0Callback(&rx_header, rx_data);
//	}
}

void CAN1_Rx0Callback(CAN_RxHeaderTypeDef* rx_header, uint8_t* rx_data)
{
  if (rx_data[0] == 0x36 && rx_data[6] == 0x6B)
  {
	  switch(rx_header->ExtId)
	  {
		case 0x05:
			motor_info[5].total_pulse = ((uint32_t)rx_data[2] << 24) |((uint32_t)rx_data[3] << 16) | ((uint32_t)rx_data[4] << 8)  | (uint32_t)rx_data[5];
		default:
		  break;
	  }
  }
}

//void CAN2_Rx0Callback(CAN_RxHeaderTypeDef* rx_header, uint8_t* rx_data)
//{
//  switch(rx_header->StdId)
//  {
//	default:
//      break;
//  }
//}

void set_can_ctrl(CAN_HandleTypeDef* hcan,uint32_t id, uint8_t tx_data[], uint8_t dlc)
{
  CAN_TxHeaderTypeDef tx_header;
  uint32_t tx_mailbox;
//  uint8_t             tx_data[8];

  tx_header.ExtId = id << 8;
  tx_header.IDE   = CAN_ID_EXT;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = dlc;

  HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);
}

void ZDT_Send_AbsAngle(CAN_HandleTypeDef* hcan, uint32_t id,uint8_t dir, float pos_t, float vel_t, float acc_t)
{
	uint8_t tx_data[8];
	
    int32_t current_pulse = motor_info[id].total_pulse;
    int32_t ppr = (int32_t)PULSE_PER_REV;

    int32_t current_single = current_pulse % ppr;
    if (current_single < 0) current_single += ppr;

    int32_t target_single = (int32_t)(pos_t * (PULSE_PER_REV / 360.0f)) % ppr;
    if (target_single < 0) target_single += ppr;

    int32_t delta_pulse = target_single - current_single;

    if (dir == 0)//cw
    {
        if (delta_pulse <= 0) {
            delta_pulse += ppr;
        }
    }
    else//ccw逆时针
    {
        if (delta_pulse >= 0) {
            delta_pulse -= ppr; 
        }
    }
	
	int32_t final_target_pulse = current_pulse + delta_pulse;
    uint32_t abs_pulse = (final_target_pulse >= 0) ? (uint32_t)final_target_pulse : (uint32_t)(-final_target_pulse);

    tx_data[0] =  0xF5;//绝对位置
    tx_data[1] = (final_target_pulse >= 0) ? 0x00 : 0x01;//0为正1为负
    tx_data[2] = (uint8_t)acc_t;
    tx_data[3] = (uint8_t)((abs_pulse >> 24) & 0xFF);
    tx_data[4] = (uint8_t)((abs_pulse >> 16) & 0xFF);
    tx_data[5] = (uint8_t)((abs_pulse >> 8)  & 0xFF);
    tx_data[6] = (uint8_t)(abs_pulse & 0xFF);
    tx_data[7] = 0x6B; 
	set_can_ctrl(hcan, id, tx_data, sizeof(tx_data));
}
