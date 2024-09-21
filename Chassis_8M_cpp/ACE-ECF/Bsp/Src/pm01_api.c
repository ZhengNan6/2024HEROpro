/*
 ***********************************************************************************
 *
 * @file     pm01_api.c
 * @brief    PM01�������ݿ��������Ƴ���API
 * @author   hepeng(1130001324@qq.com)
 *
 * Copyright (c) 2017-2021, �������������Ƽ����޹�˾ All rights reserved. 
 * 
 ***********************************************************************************
 * History:
 * 2021-07-08    hepeng   ����v1.0
 ***********************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "pm01_api.h"

pm01_od_t pm01_od;
static CAN_HandleTypeDef* pm01_hcan;
volatile uint16_t  pm01_access_id;    /* ���ڷ��ʵı�ʶ��     */
volatile uint16_t  pm01_response_flg; /* ��������Ӧ�ɹ���־λ */

uint16_t g_cmd_set   = 2;
uint16_t g_power_set = 5000;
uint16_t g_vout_set  = 2400;
uint16_t g_iout_set  = 500;
pm01_od_t* get_superpower_point(void)
{
    return &pm01_od;
}

/**
  * @brief          ���������
  * @param[in]      new_cmd   0x00: ͣ��
															0x01: ���У�����������ؿ��أ�ֻ���������ݳ�磩
															0x02: ���У���������ؿ��أ���������ʹ�ø�ָ�
	                  save_flg: 0x00: ��������EEPROM  0x01: ������EEPROM
  * @retval         none
  */
void pm01_cmd_send( uint16_t new_cmd, uint8_t save_flg )
{
	uint32_t send_mail_box;

	CAN_TxHeaderTypeDef  power_tx_message;
	uint8_t              power_can_send_data[8];

	power_tx_message.StdId = 0x600;
	power_tx_message.IDE   = CAN_ID_STD;
	power_tx_message.RTR   = CAN_RTR_DATA;
	power_tx_message.DLC   = 0x04;
	power_can_send_data[0] = (uint8_t)(new_cmd >> 8   );
	power_can_send_data[1] = (uint8_t)(new_cmd &  0xFF);
	power_can_send_data[2] = 0x00;
	power_can_send_data[3] = (save_flg == 0x01);			

	HAL_CAN_AddTxMessage(&pm01_hcan, &power_tx_message, power_can_send_data, &send_mail_box);
	
}

/**
  * @brief          ���ù���
  * @param[in]      new_power���µĹ���ֵ
                    save_flg: 0x00: ��������EEPROM  0x01: ������EEPROM
  * @retval         none
  */
void pm01_power_set( uint16_t new_power, uint8_t save_flg )
{
	uint32_t send_mail_box;

	CAN_TxHeaderTypeDef  power_tx_message;
	uint8_t              power_can_send_data[8];

	power_tx_message.StdId = 0x601;
	power_tx_message.IDE   = CAN_ID_STD;
	power_tx_message.RTR   = CAN_RTR_DATA;
	power_tx_message.DLC   = 0x04;
	power_can_send_data[0] = (uint8_t)(new_power >> 8   );
	power_can_send_data[1] = (uint8_t)(new_power &  0xFF);
	power_can_send_data[2] = 0x00;
	power_can_send_data[3] = (save_flg == 0x01);			

	HAL_CAN_AddTxMessage(&pm01_hcan, &power_tx_message, power_can_send_data, &send_mail_box);
	
}
/**
  * @brief          ���������ѹ
  * @param[in]      new_volt���µĵ�ѹֵ
                    save_flg: 0x00: ��������EEPROM  0x01: ������EEPROM
  * @retval         none
  */
void pm01_voltage_set( uint16_t new_voltage, uint8_t save_flg )
{
	uint32_t send_mail_box;

	CAN_TxHeaderTypeDef  power_tx_message;
	uint8_t              power_can_send_data[8];


	power_tx_message.StdId = 0x602;
	power_tx_message.IDE   = CAN_ID_STD;
	power_tx_message.RTR   = CAN_RTR_DATA;
	power_tx_message.DLC   = 0x04;
	power_can_send_data[0] = (uint8_t)(new_voltage >> 8   );
	power_can_send_data[1] = (uint8_t)(new_voltage &  0xFF);
	power_can_send_data[2] = 0x00;
	power_can_send_data[3] = (save_flg == 0x01);			


	HAL_CAN_AddTxMessage(&pm01_hcan, &power_tx_message, power_can_send_data, &send_mail_box);
	
}
/**
  * @brief          ���������ѹ
  * @param[in]      new_volt���µĵ�ѹֵ
                    save_flg: 0x00: ��������EEPROM  0x01: ������EEPROM
  * @retval         none
  */
void pm01_current_set( uint16_t new_current, uint8_t save_flg )
{
	uint32_t send_mail_box;

	CAN_TxHeaderTypeDef  power_tx_message;
	uint8_t              power_can_send_data[8];

	power_tx_message.StdId = 0x603;
	power_tx_message.IDE   = CAN_ID_STD;
	power_tx_message.RTR   = CAN_RTR_DATA;
	power_tx_message.DLC   = 0x04;
	power_can_send_data[0] = (uint8_t)(new_current >> 8   );
	power_can_send_data[1] = (uint8_t)(new_current &  0xFF);
	power_can_send_data[2] = 0x00;
	power_can_send_data[3] = (save_flg == 0x01);			

	HAL_CAN_AddTxMessage(&pm01_hcan, &power_tx_message, power_can_send_data, &send_mail_box);
	
}//610//611
/**
  * @brief          ��ѯ����PM01����������100HZƵ�ʵ���
  * @param[in]      none
  * @retval         none
  */
void pm01_access_poll(void)
{
	
	static uint8_t   m_state   = 0x00;   /* ״̬    */
	static uint16_t  m_can_id  = 0x600;  /* ��ʶ��  */
	static uint32_t  m_timeout = 0;      /* ��ʱ    */
	
	static uint16_t  i = 0;
	
	CAN_TxHeaderTypeDef  power_tx_message;

	uint8_t              power_can_send_data[8];

	uint32_t             send_mail_box;

	switch( m_state )
	{
		case 0x00:  /* �������� */
		
		  m_timeout = 0;   /* �����ʱ��ʱ�� */		

		  m_state = 0x01;  /* �л����ȴ�״̬ */

		  switch(i)
			{
					case 0:
					case 1:
					case 2:
					case 3:
					case 4:
					case 5:
					case 6:
					case 7:
							pm01_access_id = m_can_id; /* ��¼���η��ʵı�ʶ�� */

							power_tx_message.StdId = m_can_id;       /* ������Ӧ�ı�ʶ�� */
							power_tx_message.IDE   = CAN_ID_STD;     /* ��׼֡           */
							power_tx_message.RTR   = CAN_RTR_REMOTE; /* Զ��֡           */
							power_tx_message.DLC   = 0x00;

							HAL_CAN_AddTxMessage(&pm01_hcan, &power_tx_message, power_can_send_data, &send_mail_box);
					  break;
					case 8:
					
							pm01_access_id = m_can_id; /* ��¼���η��ʵı�ʶ�� */					
					
						  pm01_cmd_send( g_cmd_set, 0x00 );
					  break;
					case 9:
					
							pm01_access_id = m_can_id; /* ��¼���η��ʵı�ʶ�� */					
					
						  pm01_power_set( g_power_set, 0x00 );
					  break;
					case 10:
					
							pm01_access_id = m_can_id; /* ��¼���η��ʵı�ʶ�� */					
					
						  pm01_voltage_set( g_vout_set, 0x00 );
					  break;
					case 11:
					
							pm01_access_id = m_can_id; /* ��¼���η��ʵı�ʶ�� */					
					
						  pm01_current_set( g_iout_set, 0x00 );
					  break;						
			}
		
			break;
		case 0x01: /* �ȴ���Ӧ */
		
	
      if( pm01_response_flg == 0x01 )
			{
				
				pm01_response_flg = 0x00; /* ��λӦ���־λ */
				
				 /* ���ʳɹ�������������һ����ʶ�� */
				
				i = (i+1) % 12;
				

				switch( i )
				{
					case 0: m_can_id = 0x600; break;
					case 1: m_can_id = 0x601; break;
					case 2: m_can_id = 0x602; break;
					case 3: m_can_id = 0x603; break;
					case 4: m_can_id = 0x610; break;
					case 5: m_can_id = 0x611; break;
					case 6: m_can_id = 0x612; break;
					case 7: m_can_id = 0x613; break;
				  case 8: m_can_id = 0x600; break; /* д */
					case 9: m_can_id = 0x601; break; /* д */
					case 10: m_can_id = 0x602; break; /* д */
					case 11: m_can_id = 0x603; break; /* д */

				}
				
				m_state = 0x00;  /* ������һ�ַ��� */							
				
			}
			else
			{
				m_timeout++;
			}

		  /* ��ʱ1s�����·���  */
      if( m_timeout > 100 )
			{
				m_state = 0x00;	
			}				
		
			break;
		default:
			m_state = 0x00;
	
	}

}
/**
  * @brief          Ӧ��������CAN�����ж��е���
  * @param[in]      none
  * @retval         none
  */
void pm01_response_handle(CAN_RxHeaderTypeDef  *can_rx_header, uint8_t *can_rx_data )
{

	uint16_t m_tmp;
	
	pm01_response_flg = ( pm01_access_id == can_rx_header->StdId );
	
	if( can_rx_header->RTR == CAN_RTR_REMOTE )return;
	
	switch( can_rx_header->StdId )
	{
		case 0x600:
		
      m_tmp = (uint16_t)can_rx_data[0] << 8 | can_rx_data[1];	
		
		  pm01_od.ccr = m_tmp;
		
			break;
		case 0x601:
			
		  m_tmp = (uint16_t)can_rx_data[0] << 8 | can_rx_data[1];	

			pm01_od.p_set = m_tmp;
		
			break;
		case 0x602:
			
		  m_tmp = (uint16_t)can_rx_data[0] << 8 | can_rx_data[1];	

			pm01_od.v_set = m_tmp;
					
			break;
		case 0x603:
			
		  m_tmp = (uint16_t)can_rx_data[0] << 8 | can_rx_data[1];	

			pm01_od.i_set = m_tmp;
					
			break;
		case 0x610:
			
		  m_tmp = (uint16_t)can_rx_data[0] << 8 | can_rx_data[1];	

			pm01_od.sta_code.all = m_tmp;
		
		  m_tmp = (uint16_t)can_rx_data[2] << 8 | can_rx_data[3];	

			pm01_od.err_code = m_tmp;
		
			break;
		case 0x611:
			
		  m_tmp = (uint16_t)can_rx_data[0] << 8 | can_rx_data[1];	

			pm01_od.p_in = m_tmp;
		
		  m_tmp = (uint16_t)can_rx_data[2] << 8 | can_rx_data[3];	

			pm01_od.v_in = m_tmp;	
		
		  m_tmp = (uint16_t)can_rx_data[4] << 8 | can_rx_data[5];	

			pm01_od.i_in = m_tmp;
		
			break;
		case 0x612:
			
		  m_tmp = (uint16_t)can_rx_data[0] << 8 | can_rx_data[1];	

			pm01_od.p_out = m_tmp;
		
		  m_tmp = (uint16_t)can_rx_data[2] << 8 | can_rx_data[3];	

			pm01_od.v_out = m_tmp;	
		
		  m_tmp = (uint16_t)can_rx_data[4] << 8 | can_rx_data[5];	

			pm01_od.i_out = m_tmp;
					
			break;
		case 0x613:
			
		  m_tmp = (uint16_t)can_rx_data[0] << 8 | can_rx_data[1];	

			pm01_od.temp = m_tmp;
		
		  m_tmp = (uint16_t)can_rx_data[2] << 8 | can_rx_data[3];	

			pm01_od.total_time = m_tmp;	
		
		  m_tmp = (uint16_t)can_rx_data[4] << 8 | can_rx_data[5];	

			pm01_od.run_time = m_tmp;
					
			break;
	}


}
/** 
 *@brief 主动读取超电输出功率
 */
void Read_SuperPower_Vout(void)
{
    CAN_TxHeaderTypeDef CAN1_Txmessage;
    uint8_t CAN1_Tx_Data[8]={0};
    uint32_t send_mail_box;
    CAN1_Txmessage.StdId =0x612;	  
    CAN1_Txmessage.IDE = CAN_ID_STD;
    CAN1_Txmessage.RTR = CAN_RTR_REMOTE; //远程帧读取数据
    CAN1_Txmessage.DLC   = 0x00;

    HAL_CAN_AddTxMessage(&pm01_hcan, &CAN1_Txmessage, CAN1_Tx_Data, &send_mail_box); //将一段数据通过 CAN 总线发送
}

/**
  *@brief 超点信息接收回调函数
  */
Pm01_Info_t Pm01_Info;
void SuperPower_V_responed(CAN_RxHeaderTypeDef *CAN_Rxmessage, uint8_t *can_rx_data)
{
    uint16_t m_tmp; 
    if (CAN_Rxmessage->StdId != 0x612)  return;
    
    m_tmp = (uint16_t)can_rx_data[0] << 8 | can_rx_data[1];	
    Pm01_Info.p_out = m_tmp/100.0f;

    m_tmp = (uint16_t)can_rx_data[2] << 8 | can_rx_data[3];	
    Pm01_Info.v_out = m_tmp/100.0f;

    m_tmp = (uint16_t)can_rx_data[4] << 8 | can_rx_data[5];	
    Pm01_Info.i_out = m_tmp/100.0f;

	Pm01_Info.energy = ((float)(Pm01_Info.p_out-12)/(float)(24-12)) * 100;//13v以下容易触发电调低压保护
}

/**
 * @brief 获取超电控制板信息
 * @param hcan 位于哪路can
 */
Pm01_Info_t *Get_Pm01_Info_Point(bsp_can_e hcan)
{
	if (hcan == Bsp_Can1)	pm01_hcan = &hcan1;
	else 					pm01_hcan = &hcan2;
	ECF_CAN_Rx_Callback_Register(hcan, Bsp_Stdid, 0X612, SuperPower_V_responed);
	Pm01_Info.Get_Info = Read_SuperPower_Vout;
	Pm01_Info.Cmd_send = pm01_cmd_send;
	Pm01_Info.Current_set = pm01_current_set;
	Pm01_Info.Voltage_set = pm01_voltage_set;
	Pm01_Info.Power_set = pm01_power_set;
	return &Pm01_Info;
}
/*
 *  [] END OF FILE 
 */
