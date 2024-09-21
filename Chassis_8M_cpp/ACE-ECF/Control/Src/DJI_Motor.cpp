#include "DJI_Motor.hpp"
#include "maths.h"
#include <string.h>
extern "C"
{
#include "Bsp_Can.h"
}
#include "stm32f4xx_hal.h"
#ifndef PI
#define PI 3.1415926f
#endif
void ALL_DJI_CAN1_Rx_Callback_Register(CAN_RxHeaderTypeDef *Rxmessage, uint8_t *DJI_Motor_Rx_Data);
void ALL_DJI_CAN2_Rx_Callback_Register(CAN_RxHeaderTypeDef *Rxmessage, uint8_t *DJI_Motor_Rx_Data);
static DJI_Motor_Object *DJI_Motor_List[2][10] = {nullptr};



void MotorManager::AddMotor(DJI_Motor_Object* motor, CAN_HandleTypeDef* canHandle) {
    int index = (canHandle == &hcan1) ? 0 : 1;
    for (int i = 0; i < 10; ++i) {
        if (DJI_Motor_List[index][i] == nullptr) {
            DJI_Motor_List[index][i] = motor;
            return;
        }
    }
}


void DJI_Motor_Object::DJI_Motor_Encoder::Clear(void)
{
   this->Encode_Record_Val = 0;
   this->Encode_Actual_Val = 0;
   this->Radian = 0;
   this->State = NORM;
   this->last_position = 9000;
}
/**
 *@brief DJI_Motor_Encoder_Data_Init      编码器初始化
 *@param Encoder_Type_e                  电机类型
 *@param radius                半径mm
 *@note  默认不开启堵转检测
 */
void DJI_Motor_Object::DJI_Motor_Encoder::DJI_Motor_Encoder_Data_Init(Encoder_Type_e Encoder_Type, uint16_t radius)
{

    memset((void *)this, 0, sizeof(DJI_Motor_Encoder));
    this->radius = radius;
    this->Encoder_Type = Encoder_Type;
    this->State = NORM; // 状态
                        //    this->Block_Detect_Enable = 0;
    switch (Encoder_Type)
    {
    case M2006:
    {
        this->lap_encoder = 8192; // 编码器单圈码盘值
        this->gear_Ratio = 36;    // 2006减速比
        this->Max_Block_Angle_Num = 10;
        this->Max_Block_Num = 256;
    }
    break;

    case M3508:
    {
        this->lap_encoder = 8192; // 编码器单圈码盘值
        this->gear_Ratio = 19;    // 3508减速比
        this->Max_Block_Angle_Num = 10;
        this->Max_Block_Num = 256;
    }
    break;

    case GM6020:
    {
        this->lap_encoder = 8192; // 编码器单圈码盘值
        this->gear_Ratio = 1;     // 减速比
    }
    break;
    default:
        break;
    }
}
// 临角处理16位（对应角度正值）
int16_t DJI_Motor_Object::DJI_Motor_Encoder::angle_limiting_int16(int16_t Angl_Err, int16_t lap_encoder)
{
    //|当前值 - 上一次值| > 编码器最大值/2 时说明向上溢出
    if (Angl_Err < -(lap_encoder / 2))
    {
        Angl_Err += (lap_encoder - 1);
    }
    if (Angl_Err > (lap_encoder / 2))
    {
        Angl_Err -= (lap_encoder - 1);
    }
    return Angl_Err;
}
void DJI_Motor_Object::DJI_Motor_Encoder::DJI_Motor_Encoder_Data_Deal(int16_t position, int16_t rpm)
{
    DJI_Motor_Encoder *Encoder = this;
    Encoder->position = position; // 未处理的Can原始码盘
    /*速度处理*/
    Encoder->Speed[1] = rpm;                                                                                    // mm转cm
    Encoder->linear_speed[1] = (float)(rpm / Encoder->gear_Ratio) * PI * 2.0f * (Encoder->radius / 10) / 60.0f; // 算出来理论上是cm/s
    // 加速度计算
    Encoder->AccSpeed = Encoder->Speed[1] - Encoder->Speed[0];
    Encoder->Speed[0] = Encoder->Speed[1];
    Encoder->Acc_linear_speed = Encoder->linear_speed[1] - Encoder->linear_speed[0];
    Encoder->linear_speed[0] = Encoder->linear_speed[1];
    // 编码值范围为0-8192，若last_position为9000，则证明是第一次接收到信息
    if (Encoder->last_position == 9000)
    {
        Encoder->last_position = Encoder->position;
        Encoder->Encode_Actual_Val = Encoder->position;
    }

    // 多圈码盘值，不做360度后归零处理
    volatile int erro = angle_limiting_int16(Encoder->position - Encoder->last_position, Encoder->lap_encoder); // 差值累加

    Encoder->Encode_Record_Val += erro;
    //    用于360度的限幅（好像是超过360归零的，但是我不需要）
    //    Encoder->Encode_Actual_Val = check_codevalue(Encoder->Encode_Actual_Val , radio, Encoder->lap_encoder);             //过临界值复位码盘值

    Encoder->Encode_Actual_Val += erro;

    Encoder->last_position = Encoder->position;

    Encoder->Actual_Angle = (float)(Encoder->Encode_Actual_Val) / (Encoder->gear_Ratio * Encoder->lap_encoder) * 360;
    Encoder->Record_Angle = (float)(Encoder->Encode_Record_Val) / (Encoder->gear_Ratio * Encoder->lap_encoder) * 360;

    // 电机堵转检测
    // Encoder->State = Block_Detect(erro, Encoder);
    erro = 0;
}

uint8_t DJI_Motor_Object::ALL_DJI_Motor_Register_Flag = 0;
uint8_t DJI_Motor_Object::DJI_Motor_Tx_Data_200[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t DJI_Motor_Object::DJI_Motor_Tx_Data_1FF[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t DJI_Motor_Object::DJI_Motor_Tx_Data_2FF[8] = {0, 0, 0, 0, 0, 0, 0, 0};

DJI_Motor_Object ::DJI_Motor_Object(CAN_HandleTypeDef *CAN_Handlef, uint8_t Motor_id, bool reverse_flag, Encoder_Type_e Encoder_type, uint32_t radius,
                                    float speed_kp, float speed_ki, float speed_kd, uint32_t speed_mode,
                                    float position_kp, float position_ki, float position_kd, uint32_t position_mode,
                                    void (*CanRxCallBackfun)(void))
: Control_PID(speed_kp,     speed_ki,       speed_kd,       speed_mode,
              position_kp,  position_ki,    position_kd,    position_mode)
{
    DJI_Motor_Object(CAN_Handlef, Motor_id, reverse_flag, Encoder_type, radius, CanRxCallBackfun);
}

DJI_Motor_Object ::DJI_Motor_Object(CAN_HandleTypeDef *CAN_Handlef, uint8_t Motor_id, bool reverse_flag, Encoder_Type_e Encoder_type, uint32_t radius,
                                    float speed_kp, float speed_ki, float speed_kd, uint32_t speed_mode,
                                    float position_kp, float position_ki, float position_kd, uint32_t position_mode)
:Motor_id(Motor_id), // 使用初始化列表初始化
CAN_Handlef(CAN_Handlef), // 初始化
reverse_flag(reverse_flag), // 确保所有成员变量都在初始化列表中被初始化
Encoder_type(Encoder_type),
Control_PID(speed_kp,     speed_ki,       speed_kd,       speed_mode,
              position_kp,  position_ki,    position_kd,    position_mode)
{
    // CAN发送部分初始化
    this->CAN_Handlef = CAN_Handlef;
    this->CAN_TxHeader_DJI.IDE = CAN_ID_STD;
    this->CAN_TxHeader_DJI.RTR = CAN_RTR_DATA;
    this->CAN_TxHeader_DJI.DLC = 0x08;
    // CAN接收ID初始化
    if (Encoder_type == GM6020) // GM6020反馈ID从 204起
    {
        this->DJIMotor_Control_Iutput_point = &this->DJIMotor_Control_Iutput.voltage_input;
        this->DJIMotor_Control_Iutput_Limit = 30000;
        this->Can_Rx_id = 0x204 + Motor_id;
        if (Motor_id <= 4)
        {
            this->Send_data_point = DJI_Motor_Tx_Data_1FF;
            this->CAN_TxHeader_DJI.StdId = 0x1FF;
            this->Send_data_idx = (this->Motor_id - 1) * 2;
        }
        else
        {
            this->Send_data_point = DJI_Motor_Tx_Data_2FF;
            this->CAN_TxHeader_DJI.StdId = 0x2FF;
            this->Send_data_idx = (this->Motor_id - 5) * 2;
        }
    }
    else
    {
        this->DJIMotor_Control_Iutput_point = &this->DJIMotor_Control_Iutput.current_input;
        this->DJIMotor_Control_Iutput_Limit = 16000;
        this->Can_Rx_id = 0x200 + Motor_id;
        if (Motor_id <= 4)
        {
            this->Send_data_point = DJI_Motor_Tx_Data_200;
            this->CAN_TxHeader_DJI.StdId = 0x200;
            this->Send_data_idx = (this->Motor_id - 1) * 2;
        }
        else
        {
            this->Send_data_point = DJI_Motor_Tx_Data_1FF;
            this->CAN_TxHeader_DJI.StdId = 0x1FF;
            this->Send_data_idx = (this->Motor_id - 5) * 2;
        }
    }

    Encoder.DJI_Motor_Encoder_Data_Init(Encoder_type, radius);
    // 电机属性初始化
    this->reverse_flag = reverse_flag;
    if (!ALL_DJI_Motor_Register_Flag)
    {
        ECF_CAN_Rx_Callback_Register(Bsp_Can1, Bsp_Stdid, 0, ALL_DJI_CAN1_Rx_Callback_Register);
        ECF_CAN_Rx_Callback_Register(Bsp_Can2, Bsp_Stdid, 0, ALL_DJI_CAN2_Rx_Callback_Register);
        ALL_DJI_Motor_Register_Flag = true;
    }
}

DJI_Motor_Object ::DJI_Motor_Object(CAN_HandleTypeDef *CAN_Handlef, uint8_t Motor_id, bool reverse_flag, Encoder_Type_e Encoder_type, uint32_t radius,
                                    float speed_kp, float speed_ki, float speed_kd, uint32_t speed_mode,
                                    void (*CanRxCallBackfun)(void))
: Control_PID(speed_kp, speed_ki, speed_kd, speed_mode)
{
    DJI_Motor_Object(CAN_Handlef, Motor_id, reverse_flag, Encoder_type, radius, CanRxCallBackfun);
}

DJI_Motor_Object ::DJI_Motor_Object(CAN_HandleTypeDef *CAN_Handlef, uint8_t Motor_id, bool reverse_flag, Encoder_Type_e Encoder_type, uint32_t radius,
                                    float speed_kp, float speed_ki, float speed_kd, uint32_t speed_mode)
: Control_PID(speed_kp, speed_ki, speed_kd, speed_mode)
{
    DJI_Motor_Object(CAN_Handlef, Motor_id, reverse_flag, Encoder_type, radius);
}

DJI_Motor_Object ::DJI_Motor_Object(CAN_HandleTypeDef *CAN_Handlef, uint8_t Motor_id, bool reverse_flag, Encoder_Type_e Encoder_type, uint32_t radius, void (*CanRxCallBackfun)(void))
{
    this->CanRxCallBack = CanRxCallBackfun;
    DJI_Motor_Object(CAN_Handlef, Motor_id, reverse_flag, Encoder_type, radius);
}

DJI_Motor_Object ::DJI_Motor_Object(CAN_HandleTypeDef *CAN_Handlef, uint8_t Motor_id, bool reverse_flag, Encoder_Type_e Encoder_type, uint32_t radius)
: Motor_id(Motor_id), // 使用初始化列表初始化
CAN_Handlef(CAN_Handlef), // 初始化
reverse_flag(reverse_flag), // 确保所有成员变量都在初始化列表中被初始化
Encoder_type(Encoder_type)
{
    // CAN发送部分初始化
    this->CAN_Handlef = CAN_Handlef;
    this->CAN_TxHeader_DJI.IDE = CAN_ID_STD;
    this->CAN_TxHeader_DJI.RTR = CAN_RTR_DATA;
    this->CAN_TxHeader_DJI.DLC = 0x08;
    // CAN接收ID初始化
    if (Encoder_type == GM6020) // GM6020反馈ID从 204起
    {
        this->DJIMotor_Control_Iutput_point = &this->DJIMotor_Control_Iutput.voltage_input;
        this->DJIMotor_Control_Iutput_Limit = 30000;
        this->Can_Rx_id = 0x204 + Motor_id;
        if (Motor_id <= 4)
        {
            this->Send_data_point = DJI_Motor_Tx_Data_1FF;
            this->CAN_TxHeader_DJI.StdId = 0x1FF;
            this->Send_data_idx = (this->Motor_id - 1) * 2;
        }
        else
        {
            this->Send_data_point = DJI_Motor_Tx_Data_2FF;
            this->CAN_TxHeader_DJI.StdId = 0x2FF;
            this->Send_data_idx = (this->Motor_id - 5) * 2;
        }
    }
    else
    {
        this->DJIMotor_Control_Iutput_point = &this->DJIMotor_Control_Iutput.current_input;
        this->DJIMotor_Control_Iutput_Limit = 16000;
        this->Can_Rx_id = 0x200 + Motor_id;
        if (Motor_id <= 4)
        {
            this->Send_data_point = DJI_Motor_Tx_Data_200;
            this->CAN_TxHeader_DJI.StdId = 0x200;
            this->Send_data_idx = (this->Motor_id - 1) * 2;
        }
        else
        {
            this->Send_data_point = DJI_Motor_Tx_Data_1FF;
            this->CAN_TxHeader_DJI.StdId = 0x1FF;
            this->Send_data_idx = (this->Motor_id - 5) * 2;
        }
    }

    Encoder.DJI_Motor_Encoder_Data_Init(Encoder_type, radius);
    // 电机属性初始化
    this->reverse_flag = reverse_flag;
    if (!ALL_DJI_Motor_Register_Flag)
    {
        ECF_CAN_Rx_Callback_Register(Bsp_Can1, Bsp_Stdid, 0, ALL_DJI_CAN1_Rx_Callback_Register);
        ECF_CAN_Rx_Callback_Register(Bsp_Can2, Bsp_Stdid, 0, ALL_DJI_CAN2_Rx_Callback_Register);
        ALL_DJI_Motor_Register_Flag = true;
    }
    // for (int i = 0; i < 10; i++)
    // {
    //     if (DJI_Motor_List[CAN_Handlef == &hcan1 ? 0 : 1][i] == nullptr)
    //     {
    //         DJI_Motor_List[CAN_Handlef == &hcan1 ? 0 : 1][i] = this;
    //         break;
    //     }
    // }
}
void DJI_Motor_Object::Ctrl_Calc(void)
{
    //计算PID闭环
    if (this->Using_PID == Using_PID_e::Position_Speed_PID)
        *this->DJIMotor_Control_Iutput_point = (int16_t)this->Control_PID.PidCalculate(this->set_position, this->Encoder.Encode_Record_Val, this->DJIMotor_feedback_data.speed);
    else if (this->Using_PID == Using_PID_e::Speed_PID)
        *this->DJIMotor_Control_Iutput_point = (int16_t)this->Control_PID.PidCalculate(this->set_speed, this->DJIMotor_feedback_data.speed);    
    else if (this->Using_PID == Using_PID_e::Current_PID)
        *this->DJIMotor_Control_Iutput_point = this->set_control;
    this->Update_Myself_Control_Data(*this->DJIMotor_Control_Iutput_point);
}

uint8_t DJI_Motor_Object::DJI_Motor_Data_Deal(CAN_RxHeaderTypeDef *Rxmessage, uint8_t *DJI_Motor_Rx_Data)
{
    if (Rxmessage->StdId != this->Can_Rx_id)
        return 0; // 非本电机信息

    // 电机基本数据处理
    this->DJIMotor_feedback_data.position = (int16_t)(DJI_Motor_Rx_Data[0] << 8 | DJI_Motor_Rx_Data[1]);
    this->DJIMotor_feedback_data.speed = (int16_t)(DJI_Motor_Rx_Data[2] << 8 | DJI_Motor_Rx_Data[3]);
    // 不同电机接收的信息不一样
    if (this->Encoder_type == M2006)
        this->DJIMotor_feedback_data.Output_torque = (int16_t)(DJI_Motor_Rx_Data[4] << 8 | DJI_Motor_Rx_Data[5]);
    else
        this->DJIMotor_feedback_data.Torque_current = (int16_t)(DJI_Motor_Rx_Data[4] << 8 | DJI_Motor_Rx_Data[5]);
    this->DJIMotor_feedback_data.temperature = DJI_Motor_Rx_Data[6];

    // 编码盘处理
    this->Encoder.DJI_Motor_Encoder_Data_Deal(this->DJIMotor_feedback_data.position, this->DJIMotor_feedback_data.speed);

    if (this->CanRxCallBack != nullptr)
        this->CanRxCallBack(); // 调用接收回调函数
    return 1;
}

void DJI_Motor_Object::Set_PID_Mode(Using_PID_e Using_PID)
{
    this->Using_PID = Using_PID;
}
/**
 * @brief 设定电机PID外环目标值
 * @param PID_Val 外环目标值
 * @note  速度环会根据电机反转标志位决定是否取反，但是位置环不会
 */
void DJI_Motor_Object::Set_PID_Val(int16_t PID_Val)
{
    if (this->Using_PID == Using_PID_e::Position_Speed_PID)
        this->set_position = PID_Val;
    else if (this->Using_PID == Using_PID_e::Speed_PID)
        this->set_speed = PID_Val * (this->reverse_flag == true? (-1):(1));
    else if (this->Using_PID == Using_PID_e::Current_PID)
        this->set_control = PID_Val;
    

}

void DJI_Motor_Object::Set_Keep(void)
{
    if (this->Using_PID == Position_Speed_PID)
        this->set_position = this->Encoder.Encode_Record_Val;
    else 
        this->set_speed = this->DJIMotor_feedback_data.speed;
}

void DJI_Motor_Object::Update_Myself_Control_Data(int16_t Send_Control_Message)
{
    //限幅
    *this->DJIMotor_Control_Iutput_point = Send_Control_Message;
    if (*this->DJIMotor_Control_Iutput_point > this->DJIMotor_Control_Iutput_Limit)
        *this->DJIMotor_Control_Iutput_point = this->DJIMotor_Control_Iutput_Limit;
    else if (*this->DJIMotor_Control_Iutput_point < -this->DJIMotor_Control_Iutput_Limit)
             *this->DJIMotor_Control_Iutput_point = -this->DJIMotor_Control_Iutput_Limit;

    this->Send_data_point[this->Send_data_idx]   = *this->DJIMotor_Control_Iutput_point >> 8;
    this->Send_data_point[this->Send_data_idx+1] = *this->DJIMotor_Control_Iutput_point;
}

void DJI_Motor_Object::DJI_Motor_Send_Groud(void)
{
    HAL_CAN_AddTxMessage(this->CAN_Handlef, &this->CAN_TxHeader_DJI, this->Send_data_point, &this->send_mail_box);
}
void DJI_Motor_Object::DJI_Motor_Send_Groud(int16_t Send_Control_Message)
{
    this->Update_Myself_Control_Data(Send_Control_Message);
    this->DJI_Motor_Send_Groud();
}
void DJI_Motor_Object::DJI_Motor_Send_Only_Oneself(void)
{
    uint8_t Control_Message[8] = {0};
    Control_Message[this->Send_data_idx]   = this->Send_data_point[this->Send_data_idx];
    Control_Message[this->Send_data_idx+1] = this->Send_data_point[this->Send_data_idx+1];
    HAL_CAN_AddTxMessage(this->CAN_Handlef, &this->CAN_TxHeader_DJI, this->Send_data_point, &this->send_mail_box);
}
void DJI_Motor_Object::DJI_Motor_Send_Only_Oneself(int16_t Send_Control_Message)
{
    this->Update_Myself_Control_Data(Send_Control_Message);
    this->DJI_Motor_Send_Only_Oneself();
}


void ALL_DJI_CAN1_Rx_Callback_Register(CAN_RxHeaderTypeDef *Rxmessage, uint8_t *DJI_Motor_Rx_Data)
{
    for (int i = 0; i < 10; i++)
    {
        if (DJI_Motor_List[0][i] == nullptr)
            break;
        DJI_Motor_List[0][i]->DJI_Motor_Data_Deal(Rxmessage, DJI_Motor_Rx_Data);
    }
}
void ALL_DJI_CAN2_Rx_Callback_Register(CAN_RxHeaderTypeDef *Rxmessage, uint8_t *DJI_Motor_Rx_Data)
{
    for (int i = 0; i < 10; i++)
    {
        if (DJI_Motor_List[1][i] == nullptr)
            break;
        DJI_Motor_List[1][i]->DJI_Motor_Data_Deal(Rxmessage, DJI_Motor_Rx_Data);
    }
}