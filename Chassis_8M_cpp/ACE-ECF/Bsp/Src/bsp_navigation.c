/************************** Dongguan-University of Technology -ACE**************************
* @file bsp_navigation.c
* @brief
* @author love_sorrow ������ (2567715886@qq.com)
* @version 1.0
* @date 2024-1-20
*
*
* @history
* <table>
* Date       Version Author Description
* 2024-1-20  1.0   ������  ���յ�������
* ʹ�÷�����ʹ��Get������ȡ�ٶȽṹ��ָ��,�������ǵö�����������ʵʱ����
* ע�������Ҫ�Լ��������⴮�ڲ��ҵ� usbd_cdc_if.c ��������պ�������������һЩ����
* @verbatim
* ==============================================================================
*/

#include "bsp_navigation.h"
#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

VcpRx_t temp1 = {
    .rxlen =0,
    .flag = 0
};

navi_t navi_data;

extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

Chassis_Speed Speed;

extern USBD_HandleTypeDef hUsbDeviceFS;

//������ٶ�����ͣ����
int count = 0;
//���û�н��յ�����ͣ����
int Receive_count = 0;

/************************** Dongguan-University of Technology*-ACE**************************
 * @brief��� �ٶȽṹ��ָ���ȡ������z���ٶ��ǡ�/s
 * ע����� �� x��y �ٶȷŴ� 0~9000����ע���޷�
 ************************** Dongguan-University of Technology*-ACE************************** */
Chassis_Speed* Get_Navigation_Point(void)
{
	return &Speed;
}

/************************** Dongguan-University of Technology*-ACE**************************
 * @brief��� �������ݽ��㺯��
 * ���ܣ� �����������ƺ͵��߼��
 ************************** Dongguan-University of Technology*-ACE************************** */
void USB_DataRecive(void)
{
	Receive_count++;
  if(temp1.flag)
  {
   // usb_printf("%s\r\n", UserRxBufferFS);//������ط������μ���
    temp1.flag = 0;
    temp1.rxlen = 0;
    for(uint8_t i=0;i<4;i++)
    {
      navi_data.x.byte[0+i]=UserRxBufferFS[1+i];
    }
   for(uint8_t i=0;i<4;i++)
    {
      navi_data.y.byte[0+i]=UserRxBufferFS[5+i];
    }
    for(uint8_t i=0;i<4;i++)
    {
      navi_data.z.byte[0+i]=UserRxBufferFS[9+i];
    }
    memset(UserRxBufferFS, 0, APP_RX_DATA_SIZE);
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS); 
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
		
//		//�����������������Ƿ���0����
//		if(abs(navi_data.x.speed ) < USB_Dead && abs(navi_data.y.speed ) < USB_Dead)
//      count++;
//		else 
//			count = 0;
//		
//		//����0.5s�����ж�Ϊͣ��������
//		if(count > 500)
//		{
//			//�ü�������Ҫ��������
//			count = 501;

//			navi_data.x.speed = 0;
//			navi_data.y.speed = 0;
//			navi_data.z.speed = 0;
//		}
//  }
//	
//	  if(Receive_count > 500)
//		{
//			Receive_count = 501;
//			navi_data.x.speed = 0;
//			navi_data.y.speed = 0;
//			navi_data.z.speed = 0;
//		}
		
		Speed_Enlarge();
    }
}

void Speed_Enlarge(void)
{
	Speed.x = - navi_data.y.speed * Move_Speed_Factor;
	Speed.y = navi_data.x.speed * Move_Speed_Factor;
//	Speed.z = navi_data.z.speed * Rotate_Speed_Factor;
	Speed.z = navi_data.z.speed * Gimbal_Speed_Factor;
}

/************************** Dongguan-University of Technology*-ACE**************************
 * @brief��� USB �жϺ���
 * ע����� �� usbd_cdc_if.c ���滻���ͬ������
 ************************** Dongguan-University of Technology*-ACE************************** */

//static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
//{
//  /* USER CODE BEGIN 6 */
//   temp1.rxlen = temp1.rxlen + (*Len);
//    
//    if(temp1.rxlen < APP_RX_DATA_SIZE && UserRxBufferFS[0] != 0xFF  //������0XFF��ͷ
//        &&  UserRxBufferFS[temp1.rxlen - 1] != 0xFE)//***����ʱÿ�η��ͱ�����0xFE��β***
//    {
//       //---��������---------------
//       USBD_CDC_SetRxBuffer(&hUsbDeviceFS,UserRxBufferFS  + temp1.rxlen); 
//       USBD_CDC_ReceivePacket(&hUsbDeviceFS);
//    }
//    else 
//    {
//       temp1.flag = 1;    //�������
//       UserRxBufferFS[temp1.rxlen] = 0;
//			 Receive_count = 0;
//    }
//    return (USBD_OK);	
//  /* USER CODE END 6 */
//}

/************************** Dongguan-University of Technology*-ACE**************************
 * @brief��� ���ñ���
 * ע����� �� usbd_cdc_if.c ��������²���
 ************************** Dongguan-University of Technology*-ACE************************** */
 
//#include "bsp_navigation.h" 
//extern VcpRx_t temp1;
//extern navi_t navi_data;
//extern int Receive_count;
