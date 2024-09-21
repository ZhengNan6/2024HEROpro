#ifndef __DJI_MOTOR_HPP__
#define __DJI_MOTOR_HPP__
#include "pid.hpp"
#include <stdint.h>
#include "stm32f4xx_hal.h"
#define FIRE_3508_CURRENT_LIMIT 16000
#define MOTOR_3508_CURRENT_LIMIT 15000
#define MOTOR_2006_CURRENT_LIMIT 10000
#define MOTOR_6020_CURRENT_LIMIT 30000
/*电机种类枚举*/
typedef enum
{
    M3508,
    GM6020,
    M2006,
    Type_End
} Encoder_Type_e;

/*数据状态枚举*/
typedef enum
{
    NORM,  // 正常
    BLOCK, // 堵转
    WRONG  // 异常
} STATE_e;

typedef enum
{
    Position_Speed_PID,
    Speed_PID,
    Current_PID,
    No_Current
} Using_PID_e;

typedef enum
{
    PID_Work,
    Send_Set,
    No
} work_e;
typedef struct
{
    int16_t position;       // 转子位置
    int16_t speed;          // rpm (转/分钟)
    int16_t Torque_current; // 仅C620电调M3508 M6020电机有
    int16_t Output_torque;  // 仅C610电调M2006电机有
    uint8_t temperature;    // 温度
} DJIMotor_feedback_data_t;

typedef struct
{
    int16_t current_input; // 电调输出力矩电流，仅2006和3508有,-10000到100000，对应-10A到10A
    int16_t voltage_input; // 驱动器的电压输出，仅6020有  -30000到30000 对应-24V到24V
} DJIMotor_Control_Iutput_t;

class DJI_Motor_Object
{
private:
    static uint8_t ALL_DJI_Motor_Register_Flag;
    CAN_HandleTypeDef *CAN_Handlef;
    CAN_TxHeaderTypeDef CAN_TxHeader_DJI;
    int16_t Send_Message;
    uint32_t send_mail_box;
    uint8_t Send_data_idx = 0;
    uint8_t *Send_data_point = nullptr;
    int16_t *DJIMotor_Control_Iutput_point = nullptr;
    int16_t DJIMotor_Control_Iutput_Limit;
    void Update_Myself_Control_Data(int16_t Send_Control_Message);
public:
    class DJI_Motor_Encoder
    {
    public:
        int32_t Encode_Record_Val; // 累积码盘值(从0开始记)
        int32_t Encode_Actual_Val; // 真实码盘值(从当前电机编码器值开始记）

        uint32_t radius;        // 轮胎半径 mm
        float linear_speed[2];  // 线速度 0为旧速度，1为新速度
        float Acc_linear_speed; // 线加速度
        float Radian;           // 真实弧度
        float Actual_Angle;     // 圆心角
        float Record_Angle;

        int16_t Speed[2];            // rpm (转/分钟) Can回传速度 0为旧速度，1为新速度
        int16_t AccSpeed;            // 加速度
        int16_t position;            // 未处理的Can原始码盘
        int16_t last_position;       // 未处理的上次的Can原始码盘
        int16_t lap_encoder;         // 编码器单圈码盘值（8192=12bit）
        Encoder_Type_e Encoder_Type; // 编码器种类
                                     //    bool Block_Detect_Enable;    // 堵转检测开启否
        STATE_e State;               // 电机状态
        int16_t gear_Ratio;          // 减速比
        int16_t Max_Block_Angle_Num; // 堵转最大角度容忍值，可以调整这个来控制堵转灵敏度
        int16_t Max_Block_Num;       // 堵转计数器最大值
        int32_t Block_Count;
        void (*User_Fun)(void); // 用户自定义函数
        void DJI_Motor_Encoder_Data_Init(Encoder_Type_e Encoder_Type, uint16_t radius);
        int16_t angle_limiting_int16(int16_t Angl_Err, int16_t lap_encoder);
        void DJI_Motor_Encoder_Data_Deal(int16_t position, int16_t rpm);
        void Clear(void);
    };

    // can发送数据域，因为电机一般四个一组，使用静态实现批量发送
    static uint8_t DJI_Motor_Tx_Data_200[8]; // 标识符0x200  M3508 M2006 1-4号电机
    static uint8_t DJI_Motor_Tx_Data_1FF[8]; // 标识符0x1FF  M3508 M2006 5-8号电机 GM6020 1-4号电机
    static uint8_t DJI_Motor_Tx_Data_2FF[8]; // 标识符0x2FF GM6020 5-7号电机

    uint8_t Motor_id; // Can通信id
    uint32_t Can_Rx_id;
    bool reverse_flag;           // 偏置反转
    Encoder_Type_e Encoder_type; // 电机种类
    DJI_Motor_Encoder Encoder;
    DJIMotor_feedback_data_t DJIMotor_feedback_data;
    DJIMotor_Control_Iutput_t DJIMotor_Control_Iutput;

    // 注册CAN接收回调函数部分
    void (*CanRxCallBack)(void);
    DJI_Motor_Object(CAN_HandleTypeDef *CAN_Handlef, uint8_t Motor_id, bool reverse_flag, Encoder_Type_e Encoder_type, uint32_t radius, 
                    float speed_kp,    float speed_ki,    float speed_kd,    uint32_t speed_mode , 
                    float position_kp, float position_ki, float position_kd, uint32_t position_mode,
                    void (*CanRxCallBackfun)(void));
    DJI_Motor_Object(CAN_HandleTypeDef *CAN_Handlef, uint8_t Motor_id, bool reverse_flag, Encoder_Type_e Encoder_type, uint32_t radius, 
                    float speed_kp,    float speed_ki,    float speed_kd,    uint32_t speed_mode , 
                    float position_kp, float position_ki, float position_kd, uint32_t position_mode);
    DJI_Motor_Object(CAN_HandleTypeDef *CAN_Handlef, uint8_t Motor_id, bool reverse_flag, Encoder_Type_e Encoder_type, uint32_t radius, 
                    float speed_kp,    float speed_ki,    float speed_kd,    uint32_t speed_mode, 
                    void (*CanRxCallBackfun)(void));
    DJI_Motor_Object(CAN_HandleTypeDef *CAN_Handlef, uint8_t Motor_id, bool reverse_flag, Encoder_Type_e Encoder_type, uint32_t radius, 
                    float speed_kp,    float speed_ki,    float speed_kd,    uint32_t speed_mode);
    DJI_Motor_Object(CAN_HandleTypeDef *CAN_Handlef, uint8_t Motor_id, bool reverse_flag, Encoder_Type_e Encoder_type, uint32_t radius, void (*CanRxCallBackfun)(void));
    DJI_Motor_Object(CAN_HandleTypeDef *CAN_Handlef, uint8_t Motor_id, bool reverse_flag, Encoder_Type_e Encoder_type, uint32_t radius);    uint8_t DJI_Motor_Data_Deal(CAN_RxHeaderTypeDef *Rxmessage, uint8_t *DJI_Motor_Rx_Data);
    void DJI_Motor_Send_Only_Oneself(int16_t Send_Control_Message);
    void DJI_Motor_Send_Only_Oneself(void);
    void DJI_Motor_Send_Groud(int16_t Send_Control_Message);
    void DJI_Motor_Send_Groud(void);
    
    Pid_c               Control_PID;
    Using_PID_e         Using_PID;    //使用哪个环
    void Ctrl_Calc(void);
    void Set_PID_Mode(Using_PID_e  Using_PID);
    void Set_PID_Val(int16_t PID_Val);
    void Set_Keep(void);
    int64_t                set_position;
    int16_t                set_speed;       //设定线速度
    int64_t                set_control;
};

class MotorManager {
public:
    MotorManager();
    static void AddMotor(DJI_Motor_Object* motor, CAN_HandleTypeDef* canHandle);
    DJI_Motor_Object* GetMotor(int index, CAN_HandleTypeDef* canHandle) const;
};

#endif