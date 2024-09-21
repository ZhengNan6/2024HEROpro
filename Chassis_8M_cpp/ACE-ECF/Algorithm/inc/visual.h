#ifndef __VISUAL_H
#define __VISUAL_H

#include "stdint.h"

//��������Ԫ�صĸ���
#define ARR_SIZE( a ) ( sizeof( (a) ) / sizeof( (a[0]) ) )



typedef union
{
    float f;
    unsigned char c[4];
} float2uchar;

/*****�Ӿ���ؽṹ��****/
typedef struct
{
    float auto_pitch_angle; //�Ӿ�����P����
    float auto_yaw_angle;   //�Ӿ�����Y����

    float auto_kalman_pitch_angle; //�Ӿ�����P����
    float auto_kalman_yaw_angle;   //�Ӿ�����Y����
    float last_auto_pitch_angle;   //��һ���Ӿ�����P����
    float last_auto_yaw_angle;     //��һ���Ӿ�����Y����

    float len;   //�Ӿ����ؾ���
    int8_t fire; //�Ƿ񿪻�

    float auto_yaw_angular_velocity;     //�Ӿ��ش���������Ľ��ٶ�
    float auto_last_yaw_angular_velocity;     //�Ӿ��ش������������һ�εĽ��ٶ�
    float auto_yaw_angular_acceleration; //�Ӿ��ش���������ĽǼ��ٶ�

    float auto_pitch_sum;      //����P��ǶȻ���
    float auto_pitch_angle_kf; //��ؿ�����������P����
    float auto_yaw_angle_kf;   //��ؿ�����������Y����
		float auto_pitch_speed;
	
	
    int16_t auto_lost_data_pit_count; //��ʧĿ�����
    int16_t auto_lost_data_pit_flag;  //��ʧĿ���־λ
    int16_t auto_lost_data_yaw_count; //��ʧĿ�����
    int16_t auto_lost_data_yaw_flag;  //��ʧĿ���־λ
    int16_t auto_yaw_zero_flag;   //yaw��0ֵ��־λ

    float pitch_control_data;      //P���Ӿ�������
    float yaw_control_data;        //y���Ӿ�������
    float last_pitch_control_data; //��һ��P���Ӿ�������
    float last_yaw_control_data;   //��һ��y���Ӿ�������
		
		float xxx_speed; 

		float auto_pitch_angle_speed;
    int32_t world_time1;
		int32_t last_world_time1;
} vision_auto_data_t;



void USART_Send(void);
void visual_send_data(uint8_t data, uint8_t mode, uint8_t shoot_speed,float yaw,float pitch);
void Gyro_Send_Task(void *pvParameters);
float invSqrt(float x) ;
void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);
extern float hex2Float(uint8_t HighByte, uint8_t LowByte);
extern void visual_data_reception(void);
vision_auto_data_t *get_auto_control_point(void);
#endif
