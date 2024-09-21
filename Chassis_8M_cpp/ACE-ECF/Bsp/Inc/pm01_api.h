/*
 *
 * pm01_api.h
 *
 * Created on: 2021��07��08��
 *     Author: hepeng
 *
 * Copyright (c) 2021, �������������Ƽ����޹�˾ All rights reserved. 
 *
 */
 
#ifndef __PM01_API_H
#define __PM01_API_H

#include "can.h"
#include "stdint.h"
#include "bsp_can.h"

typedef union 
{

  uint16_t all;
  struct {
      uint16_t rdy:   1;  /*!< bit0    ����     */
      uint16_t run:   1;  /*!< bit1    ����     */
      uint16_t alm:   1;  /*!< bit2    ����     */
      uint16_t pwr:   1;  /*!< bit3    ��Դ���� */
      uint16_t load:  1;  /*!< bit4    ���ؿ��� */
      uint16_t cc:    1;  /*!< bit5    ����     */
      uint16_t cv:    1;  /*!< bit6    ��ѹ     */
      uint16_t cw:    1;  /*!< bit7    �㹦��   */
      uint16_t revd:  7;  /*!< bit8-14 ����     */
      uint16_t flt:   1;  /*!< bit15   ����     */
  }bit;

}csr_t;

typedef struct mb_reg_type{

   uint16_t ccr;         /*!< 8000H ���ƼĴ���     */
   uint16_t p_set;       /*!< 8001H ���빦������   */  
   uint16_t v_set;       /*!< 8002H �����ѹ����   */
   uint16_t i_set;       /*!< 8003H �����������   */
   csr_t    sta_code;    /*!< 8100H ״̬��־λ     */  
   uint16_t err_code;    /*!< 8101H ���ϴ���       */
   int16_t  v_in;        /*!< 8102H �����ѹ       */
   int16_t  i_in;        /*!< 8103H �������       */
   int16_t  p_in;        /*!< 8104H ���빦��       */
   int16_t  v_out;       /*!< 8105H �����ѹ       */
   int16_t  i_out;       /*!< 8106H �������       */
   int16_t  p_out;       /*!< 8107H �������       */
   int16_t  temp;        /*!< 8108H �¶�           */
   uint16_t total_time;  /*!< 8109H �ۼ�ʱ��       */
   uint16_t run_time;    /*!< 810AH ����ʱ��       */
   int16_t  energy;
}pm01_od_t;

typedef struct
{
    float energy;
    float p_out;
    float v_out;		
    float i_out;
    void (*Power_set)(uint16_t, uint8_t);
    void (*Voltage_set)(uint16_t, uint8_t);
    void (*Current_set)(uint16_t, uint8_t);
    void (*Cmd_send)(uint16_t, uint8_t);
    void (*Get_Info)(void);

}Pm01_Info_t;


extern void pm01_cmd_send         ( uint16_t new_cmd, uint8_t save_flg );
extern void pm01_voltage_set      ( uint16_t new_voltage, uint8_t save_flg );
extern void pm01_current_set      ( uint16_t new_voltage, uint8_t save_flg );
extern void pm01_power_set        ( uint16_t new_power, uint8_t save_flg );
extern void pm01_access_poll      ( void );
extern void pm01_response_handle  ( CAN_RxHeaderTypeDef *can_rx_header, uint8_t *can_rx_data );
pm01_od_t* get_superpower_point(void);
Pm01_Info_t *Get_Pm01_Info_Point(bsp_can_e hcan);
#endif

/*
 *  [] END OF FILE 
 */
