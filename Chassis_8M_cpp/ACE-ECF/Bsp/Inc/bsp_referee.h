#ifndef __BSP_REFEREE_H
#define __BSP_REFEREE_H

#include "CRC.h"
#include "stdint.h"
#include "string.h"
#include "stm32f4xx_hal.h"

#define NULL 0
#define Referee_Data_len 128

//֡ͷ����
#define HEADER_LEN 5
//ָ���
#define CMDID_LEN 2
//CRC�����볤��1
#define CRC16_LEN 2

//���ݶγ���
#define DATA_STATUS_LEN													11						//!����״̬���ݳ���(�ٷ�����)
#define DATA_RESULT_LEN													1						 	//����������ݳ���
#define DATA_ROBOT_HP_LEN												32					 	//!����������Ѫ�����ݳ���(�ٷ�����)
//#define DATA_DART_STATUS_LEN									3							//���ڷ���״̬����
#define DATA_EVENT_DATA_LEN											4					 		//�����¼����ݳ���
#define DATA_SUPPLY_PROJECTILE_ACTION_LEN				4		 					//!���ز���վ������ʶ���ݳ���(�ٷ�����)
#define DATA_REFEREE_WARNING_LEN								3				 			//���о������ݳ���
#define DATA_DART_REMAINING_TIME_LEN						3			 				//���ڷ���ڵ���ʱ
#define DATA_ROBOT_STATUS_LEN										13				 		//!������״̬����(�ٷ�����)
#define DATA_POWER_HEAT_DATA_LEN								16						//!ʵʱ������������(�ٷ�����)
#define DATA_ROBOT_POS_LEN											12					 	//������λ������
#define DATA_BUFF_LEN														1							//��������������
#define DATA_AERIAL_ROBOT_ENERGY_LEN						2							//!���л���������״̬����,ֻ�п��л��������ط���(�ٷ�����)
#define DATA_ROBOT_HURT_LEN											1							//�˺�״̬����
#define DATA_SHOOT_DATA_LEN											7					 		//!ʵʱ�������(�ٷ�����)
#define DATA_BULLET_REMAINING_LEN								6							//!�ӵ�ʣ�෢����(�ٷ�����)
#define DATA_RFID_STATUS_LEN										4							//������ RFID ״̬
#define DATA_DART_CLIENT_CMD_LEN								6							//���ڻ����˿ͻ���ָ����
#define DATA_ROBOT_POSITION_LEN 	40
#define DATA_RADAR_MARK_LEN 6
#define DATA_SENTRY_INFO_LEN 4
//#define DATA_STUDENT_INTERACTIVE_HEADER_DATA  6             //UI
#define DATA_DIY_CONTROLLER                     30            //�Զ��������
#define DATA_CLIENT_DOWMLOAD                    15            //С��ͼ�·�λ����Ϣ
#define DATA_PICTURE_TRANSMISSION               12            //ͼ��ң����Ϣ
#define DATA_CLIENT_RECEIVE                     10            //С��ͼ����λ����Ϣ

//������ID
#define ID_STATE 																0x0001				//����״̬����
#define ID_RESULT 															0x0002				//�����������
#define ID_ROBOT_HP 														0x0003				//���������˻�����Ѫ������
//#define ID_DART_STATUS 												0x0004				//���ڷ���״̬
#define ID_EVENT_DATA 													0x0101				//�����¼�����
#define ID_SUPPLY_PROJECTILE_ACTION 						0x0102	   		//���ز���վ������ʶ����
#define ID_SUPPLY_PROJECTILE_BOOKING 						0x0103	   		//���ز���վԤԼ�ӵ�����
#define ID_REFEREE_WARNING		 									0x0104			  //���о�������
#define ID_DART_REMAINING_TIME 									0x0105		   	//���ڷ���ڵ���ʱ
#define ID_ROBOT_STATE 													0x0201				//������״̬����
#define ID_POWER_HEAT_DATA	 										0x0202			  //ʵʱ������������
#define ID_ROBOT_POS 														0x0203				//������λ������
#define ID_BUFF 																0x0204				//��������������
#define ID_AERIAL_ROBOT_ENERGY 									0x0205		   	//���л���������״̬����
#define ID_ROBOT_HURT 													0x0206				//�˺�״̬����
#define ID_SHOOT_DATA 													0x0207				//ʵʱ�������
#define ID_BULLET_REMAINING											0x0208			  //�ӵ�ʣ�෢����
#define ID_RFID_STATUS 													0x0209				//������ RFID ״̬
#define ID_DART_CLIENT_CMD                      0x020A        //���ڻ����˿ͻ���ָ������
#define ID_GROUND_ROBOT_POSITION 		0X020B 
#define ID_RARD_MRAK_DATA 		0X020C
#define ID_SENTRY 		0X020D 
#define ID_RADAR 	0x020E

//#define ID_STUDENT_INTERACTIVE_HEADER_DATA    0x0301       // UI
#define ID_DIY_CONTROLLER  											0x0302 				//�Զ��������
#define ID_CLIENT_DOWMLOAD  										0x0303     		//С��ͼ�·�λ����Ϣ
#define ID_PICTURE_TRANSMISSION 								0x0304				//ͼ��ң����Ϣ
#define ID_CLIENT_RECEIVE  											0x0305     		//С��ͼ����λ����Ϣ



/*����״̬����*///0x0001
typedef __packed struct
{
	uint8_t	 game_type : 4;
	uint8_t  game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
	uint8_t  error;
} Game_Type_Data;

/*�����������*///0x0002
typedef __packed struct
{
	uint8_t winner;
	uint8_t error;
}	Game_Result;

/*Ѫ������*///0x0003
typedef __packed struct
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t red_5_robot_HP;
	uint16_t red_7_robot_HP;
	uint16_t red_outpost_HP;
	uint16_t red_base_HP;
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP;
	uint16_t blue_4_robot_HP;
	uint16_t blue_5_robot_HP;
	uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
	uint8_t error;
} Robot_Hp_Data;

/*�����¼�����*///0x0101
typedef __packed struct
{
//   uint32_t event_type;
//   uint32_t BloodPoint_1 : 1;
//   uint32_t BloodPoint_2 : 1;
//   uint32_t BloodPoint_3 : 1;	
//   uint32_t ENERGY : 3;
//   uint32_t Annular_HighLand : 1;
//   uint32_t R3_or_B3_HighLand : 1;
//   uint32_t R4_or_B4_HighLand : 1;
  uint32_t BloodPoint_us_front : 1;
  uint32_t BloodPoint_us_inside : 1;
  uint32_t BloodPoint_us : 1;
  uint32_t Buff_us:1;
  uint32_t BuffSmall_us:1;
  uint32_t BuffBig_us:1;
  uint32_t Upland_Annular_us:2;
  uint32_t Upland_Trapezium_us:2;
  uint32_t Upland_Trapezium_us_:2;
  uint32_t Virtual_Sheild_us:7;
  uint32_t Dart_hitPost_afterTime:9;
  uint32_t Dart_AimType:2;
  uint32_t BloodPoint_Mid:2;
  uint8_t error;
} Area_Data;

/*����վ������ʶ*///0x0102
typedef __packed struct
{
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num;
	uint8_t error;
} Supply_Data;

/*���о�����Ϣ*///0x0104
typedef __packed struct
{
	uint8_t level;
	uint8_t foul_robot_id;
	uint8_t offend_conut;
	uint8_t error;
}Referee_Warning;

/*���ڷ��������*///0x0105
typedef __packed struct
{
	uint8_t dart_remaining_time;
	uint16_t latest_aim:2;
	uint16_t aim_count:3;
	uint16_t aim_target:2;
	uint16_t reserved:9;
	uint8_t error;
} Dart_Launch_Data;

/*������״̬����*///0x0201
typedef __packed struct
{
  uint8_t robot_id; 
  uint8_t robot_level; 
  uint16_t current_HP;  
  uint16_t maximum_HP; 
  uint16_t shooter_barrel_cooling_value; 
  uint16_t shooter_barrel_heat_limit; //��ǰǹ����������
  uint16_t chassis_power_limit;  
  uint8_t power_management_gimbal_output : 1; 
  uint8_t power_management_chassis_output : 1;  
  uint8_t power_management_shooter_output : 1; 
  uint8_t reserved : 5; 
  uint8_t error;
} Robot_Situation_Data;

/*������������*///0x0202
typedef __packed struct
{
	uint16_t chassis_volt;
	uint16_t chassis_current;
	float chassis_power;
	uint16_t chassis_power_buffer;//���ʻ���
	uint16_t shooter_id1_17mm_cooling_heat;//ǹ������
	uint16_t shooter_id2_17mm_cooling_heat;
	uint16_t shooter_id1_42mm_cooling_heat;
	uint8_t error;
} Robot_Power_Heat_Data;

/*������λ��*///0x0203
typedef __packed struct
{
	float x;
	float y;
	float angle;
	uint8_t error;
} Robot_Position_Data;

/*��������������*///0x0204
typedef __packed struct
{
  uint8_t power_rune_buff;
	uint8_t error;
} Area_Buff_Data;

/*���л���������״̬*///0x0205
typedef __packed struct
{
	uint8_t attack_time;
	uint8_t error;
} UAV_Data;

/*�˺�״̬*///0x0206
typedef __packed struct
{
	uint8_t armor_id : 4;
	uint8_t hurt_type : 4;
	uint8_t error;
} Robot_Hurt_Data;

/*ʵʱ�����Ϣ*///0x0207
typedef __packed struct
{
	uint8_t bullet_type;
	uint8_t shooter_id;
	uint8_t bullet_freq;
	float bullet_speed;//������ٶ�
	uint8_t error;
} Robot_Shoot_Data;

/*�ӵ�ʣ�෢����*///0x0208
typedef __packed struct
{
	uint16_t bullet_remaining_num_17mm; //17mm �ӵ�ʣ�෢����Ŀ
	uint16_t bullet_remaining_num_42mm; //42mm �ӵ�ʣ�෢����Ŀ
	uint16_t coin_remaining_num;		//ʣ��������
	uint8_t  error;
} Robot_RaminingBullet_Data;

/*RFID״̬*///0x0209
typedef __packed struct
{
	uint32_t BaseLand : 1;
	uint32_t HighLand_us : 1;
	uint32_t HighLand_enm : 1;
	uint32_t RorB3_us : 1;
	uint32_t RorB3_enm : 1;
	uint32_t RorB4_us : 1;
	uint32_t RorB4_enm : 1;
	uint32_t Buff :1 ;
	uint32_t OverSlope_before_us : 1;
	uint32_t OverSlope_after_us : 1;
	uint32_t OverSlope_before_enm : 1;
	uint32_t  OverSlope_after_enm : 1;
	uint32_t Outpost_us:1;
	uint32_t BloodPooint_us:1;
	uint32_t Patrol_area_us:1;
	uint32_t Patrol_area_enm:1;
	uint32_t Resource_area_us:1;
	uint32_t Resource_area_enm:1;
	uint32_t Exchange_area_us:1;
	uint32_t CentralGain_Point:1;
	uint32_t other : 12;
	uint8_t error;
} RFID_Situation_Data;

/*���ڻ����˿ͻ���ָ������*///0x020A
typedef __packed struct
{
  uint8_t dart_launch_opening_status;
  uint8_t dart_attack_target;
  uint16_t target_change_time;
  uint16_t operate_launch_cmd_time;
	uint8_t error;
} Dart_Client_Cmd;

/*������λ������*/
typedef __packed struct 
{ 
  float hero_x;  
  float hero_y;  
  float engineer_x;  
  float engineer_y;  
  float standard_3_x;  
  float standard_3_y;  
  float standard_4_x;  
  float standard_4_y;  
  float standard_5_x;  
  float standard_5_y; 
  uint8_t error;
}ground_robot_position_t; 

/*�״��ǽ���*/
typedef __packed struct 
{ 
  uint8_t mark_hero_progress;  
  uint8_t mark_engineer_progress;  
  uint8_t mark_standard_3_progress;  
  uint8_t mark_standard_4_progress; 
  uint8_t mark_standard_5_progress; 
  uint8_t mark_sentry_progress; 
  uint8_t error;
}radar_mark_data_t; 

/*�������ݽ�����Ϣ*/
typedef __packed struct
{
  uint16_t data_cmd_id;
  uint16_t sender_ID;
  uint16_t receiver_ID;
//uint16_t data[];	
	uint8_t error;
} student_interactive_header_data_t;

/*ѧ�������˼�ͨ��*/
//δʹ�ù���������
//������UI�ͳ���ͨ��
typedef __packed struct
{
	uint8_t data[1];
	uint8_t error;
} robot_interactive_data_t;

/*�ͻ����·���Ϣ*/
typedef __packed struct
{
	float target_position_x;
	float target_position_y;
	float target_position_z;
	uint8_t commd_keyboard;
	uint16_t target_robot_ID;
	uint8_t error;
} Robot_Command;

/*�ͻ��˽�����Ϣ*/
//�״�վ���͵�������Ϣ���Ա����м����������ڵ�һ�ӽ�С��ͼ������
typedef __packed struct
{
	uint16_t target_robot_ID;
	float target_position_x;
	float target_position_y;
	uint8_t error;
} Client_Map_Command_Data;

typedef __packed struct
{
	uint32_t sentry_bullet:11;
	uint32_t sentry_bullet_count:4;
	uint32_t sentry_blood_count:4;
	uint32_t reserved:13;
	uint8_t error;
}sentry_info_t;


/*����ϵͳ����*/
typedef __packed struct
{
	uint8_t RefereeData[256];
	uint8_t RealData[45];
	int16_t DataLen;
	int16_t RealLen;
	int16_t Cmd_ID;
	uint8_t RECEIVE_FLAG;
	Game_Type_Data 															Game_Status;
	Game_Result                 							  Game_Result;
	Robot_Hp_Data 															Robot_HP;
	Area_Data 																	Event_Data;
	Supply_Data 																Supply_Action;
	Referee_Warning             							  Referee_Warning;
	Dart_Launch_Data 														Dart_Remaining_Time;
	Robot_Situation_Data 												Robot_Status;
	Robot_Power_Heat_Data 											Power_Heat;
	Robot_Position_Data         							  Robot_Position;
	Area_Buff_Data 															Buff;
	UAV_Data 																		Aerial_Energy;
	Robot_Hurt_Data 														Robot_Hurt;
	Robot_Shoot_Data 														Shoot_Data;
	Robot_RaminingBullet_Data 									Bullet_Num;
	RFID_Situation_Data 												RFID_Status;
	Dart_Client_Cmd             							  Dart_Client;
	student_interactive_header_data_t           Interact_Header;
	robot_interactive_data_t                    Interact_Data;
	Robot_Command         			     					  Client_Data;
	Client_Map_Command_Data          					  ClientMapData;
	ground_robot_position_t Robot_Position_Al;
	radar_mark_data_t radar_mark;
	sentry_info_t sentry;
} REFEREE_t;

//����ϵͳ��ʼ��
void referee_uart_init(UART_HandleTypeDef *huart,uint8_t *rx_buffer1, uint8_t *rx_buffer2, uint8_t buff_num);
void ECF_referee_uart_init(void);
void REFEREE_UART_IRQHandler(UART_HandleTypeDef *huart);
REFEREE_t *Get_referee_Address(void);

#endif
