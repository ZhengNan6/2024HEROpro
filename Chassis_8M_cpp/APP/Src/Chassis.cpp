#include "Chassis.hpp"
#include "RC.hpp"

extern "C"
{
#include "bsp_dwt.h"
#include "can.h"
#include "maths.h"
#include "math.h"
#include "bsp_can.h"
}
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
/** 
*@brief 设定基于底盘解算方式
*@param Solution_method 解算方式
*/
void Speed_c::Set_Solution_method(Solution_method_e Solution_method)
{
    this->Solution_method = Solution_method;
}
/** 
*@brief 设定基于底盘坐标系的速度
*@param Forward_speed 向右速度
*@param Right_speed   向前速度
*@note  按照设定的解算模式
*/
void Speed_c::Set(int16_t Forward, int16_t Right, float Different_Angle_with_Gimbal, Chassis_Mode_e Chassis_Mode,  bool Fast_Mode)
{
    this->Forward = Forward;
    this->Right = Right;
    switch(Chassis_Mode)
    {   
        case NO_FOLLOW:
            this->Spin = 0;
			break;
        case FOLLOW:
            this->Spin = -this->Z_speed_pid.PidCalculate(0, Different_Angle_with_Gimbal);
			break;
		case SPIN:
            this->Spin = 100;
			break;			
		case NO_FORCE:
            this->Right = 0;
            this->Forward = 0;
            this->Spin = 0;
			break;
        default:
            this->Right = 0;
            this->Forward = 0;
            this->Spin = 0;
            break;
    }
    this->LF = ( this->Right + this->Forward + this->Spin) * Fast_Mode?15:10;
    this->RF = (-this->Right + this->Forward - this->Spin) * Fast_Mode?15:10;
    this->LB = (-this->Right + this->Forward + this->Spin) * Fast_Mode?15:10;
    this->RB = ( this->Right + this->Forward - this->Spin) * Fast_Mode?15:10;

    //电机最大速度9000，若超过则等比例缩放到9000，防止运动失真
    uint16_t max_speed = max_abs(max_abs(this->LF, this->RF), max_abs(this->LB, this->RB));
    if (max_speed > 9000)
    {
        float divisor = 9000.0f/((float)max_speed);
        
        this->LF *=  divisor;
        this->RF *=  divisor;
        this->LB *=  divisor;
        this->RB *=  divisor;
    }

}

/**
 * @brief 更新底盘云台差角
 */

void CHASSIS_c::Update_Different_Angle_with_Gimbal(void)
{
    static int16_t yaw_encoder_zero = 1925;
    int16_t yaw_encoder_value = this->Yaw_Motor.DJIMotor_feedback_data.position;
    this->Different_Angle_with_Gimbal = (yaw_encoder_value - yaw_encoder_zero) * 360.0f / 8192.0f;
    this->Different_Angle_with_Gimbal = loop_fp32_constrain(this->Different_Angle_with_Gimbal, -180.0f, 180.0f);
}
void CHASSIS_c::Fire_Motor_Empty(void)
{
    this->FIRE_Motor.Set_PID_Mode(Current_PID);
    this->FIRE_Motor.Set_PID_Val(500);
    this->FIRE_Motor.Ctrl_Calc();
    this->FIRE_Motor.DJI_Motor_Send_Only_Oneself();
}
void CHASSIS_c::Fire_Motor_Work(void)
{
    this->FIRE_Motor.Set_PID_Mode(Position_Speed_PID);
    this->FIRE_Motor.Set_Keep();
    this->FIRE_Motor.Ctrl_Calc();
    this->FIRE_Motor.DJI_Motor_Send_Only_Oneself();
}
void CHASSIS_c::Fire_Motor_Work(int64_t SetEncoder)
{
    this->FIRE_Motor.Set_PID_Mode(Position_Speed_PID);
    this->FIRE_Motor.Set_PID_Val(SetEncoder);
    this->FIRE_Motor.Ctrl_Calc();
    this->FIRE_Motor.DJI_Motor_Send_Only_Oneself();
}
void CHASSIS_c::Fire_Motor_Nowork(void)
{
    this->FIRE_Motor.Set_PID_Mode(No_Current);
    this->FIRE_Motor.Encoder.Clear();
    this->FIRE_Motor.Ctrl_Calc();
    this->FIRE_Motor.DJI_Motor_Send_Only_Oneself();
}

/**
 * @brief 模式设置
 * @param Chassis_Mode 底盘模式
 * @param Fast_Mode    超速模式
 */
void CHASSIS_c::SetMode(Chassis_Mode_e Chassis_Mode, bool Fast_Mode)
{
    this->Chassis_Mode = Chassis_Mode;
    this->Fast_Mode = Fast_Mode;
}
/**
 * @brief 底盘驱动轮电机计算
 * @note  调用电机类提供的闭环计算方法
 */
void CHASSIS_c::MotorCalc(void)
{
    static uint8_t Set_Lock_Potision = 1;//用于停下后首次设定锁定位置
    if ((user_abs(this->LF_Motor.DJIMotor_feedback_data.speed) < 10) && (user_abs(this->LF_Motor.DJIMotor_feedback_data.speed) < 10) && (user_abs(this->LF_Motor.DJIMotor_feedback_data.speed) < 10) && (user_abs(this->LF_Motor.DJIMotor_feedback_data.speed) < 10)
     && (user_abs(this->Speed.LF) < 10) && (user_abs(this->Speed.LF) < 10) && (user_abs(this->Speed.LF) < 10)  && (user_abs(this->Speed.LF) < 10)
    )
    {
        this->Chassis_State = Chassis_State_e::LOCK;
    }    
    else
    {
        this->Chassis_State = Chassis_State_e::MOVE;
        Set_Lock_Potision = 1;
    }
    
    this->Chassis_State = Chassis_State_e::MOVE;//debug
    if (this->Chassis_State == Chassis_State_e::MOVE)
    {
        this->LF_Motor.Set_PID_Mode(Speed_PID);
        this->RF_Motor.Set_PID_Mode(Speed_PID);
        this->LB_Motor.Set_PID_Mode(Speed_PID);
        this->RB_Motor.Set_PID_Mode(Speed_PID);

        this->LF_Motor.Set_PID_Val(this->Speed.LF * 10.0f);
        this->RF_Motor.Set_PID_Val(this->Speed.RF * 10.0f);
        this->LB_Motor.Set_PID_Val(this->Speed.LB * 10.0f);
        this->RB_Motor.Set_PID_Val(this->Speed.RB * 10.0f);   
    }
    else
    {
        if (Set_Lock_Potision)
        {
            Set_Lock_Potision = 0;
            this->LF_Motor.Set_PID_Mode(Position_Speed_PID);
            this->RF_Motor.Set_PID_Mode(Position_Speed_PID);
            this->LB_Motor.Set_PID_Mode(Position_Speed_PID);
            this->RB_Motor.Set_PID_Mode(Position_Speed_PID);

            this->LF_Motor.Set_Keep();
            this->RF_Motor.Set_Keep();
            this->LB_Motor.Set_Keep();
            this->RB_Motor.Set_Keep(); 
        }
    }
    this->LF_Motor.Ctrl_Calc();
    this->RF_Motor.Ctrl_Calc();
    this->LB_Motor.Ctrl_Calc();
    this->RB_Motor.Ctrl_Calc();
}
/**
 * @brief 底盘驱动轮电机控制
 * @note  发送实际控制量
 */
void CHASSIS_c::MotorCtrl(void)
{
    this->LF_Motor.DJI_Motor_Send_Groud();
}
/**
 * @brief 设定云台坐标系下的底盘速度
 * @param Forward   云台向前速度
 * @param Right     云台向右速度
 * @note  云台坐标系速度将转换为底盘速度坐标系作为设定值
 */
void CHASSIS_c::SetSpeed(int16_t Gimbal_Forward, int16_t Gimbal_Right)
{
    this->Update_Different_Angle_with_Gimbal();
    int16_t Chassis_Right = -Gimbal_Forward * cos_calculate(this->Different_Angle_with_Gimbal)
                            -Gimbal_Right * sin_calculate(this->Different_Angle_with_Gimbal);
    int16_t Chassis_Forward = - Gimbal_Forward * sin_calculate(this->Different_Angle_with_Gimbal) 
                              + Gimbal_Right * cos_calculate(this->Different_Angle_with_Gimbal);
    this->Speed.Set(Chassis_Forward, Chassis_Right, this->Different_Angle_with_Gimbal, this->Chassis_Mode, this->Fast_Mode);
}

/**
 * @param 底盘电容控制，计算电容输出输入
 */
void CHASSIS_c::CapCtrl(void)
{
    if (this->REFEREE->Robot_Status.chassis_power_limit != 0)//更新底盘功率上限
        this->RefereePowerLimit = this->REFEREE->Robot_Status.chassis_power_limit;

    static float Get_Dwt_ms = DWT_GetTimeline_ms();
    static float Last_Get_Info_ms = 0;
    static float Last_Set_Power_ms = 0;

    if (Get_Dwt_ms - Last_Get_Info_ms > 10)//1000ms / 10ms = 100hz 读取超电电压
    {
        Last_Get_Info_ms = Get_Dwt_ms;
        this->Pm01_Info->Get_Info();

        if (this->Pm01_Info->energy >= 30)   this->CapPowerLimit_Out = this->Fast_Mode ? 160:120;//160w功率
        if (this->Pm01_Info->energy >= 20 && this->Pm01_Info->energy < 30)   this->CapPowerLimit_Out = this->RefereePowerLimit;//输入功率
        if (this->Pm01_Info->energy < 20 && this->Pm01_Info->energy >= 0)  this->CapPowerLimit_Out = this->RefereePowerLimit + this->Pm01_Info->energy - 20;//输入功率缩放
    }
    if (Get_Dwt_ms - Last_Set_Power_ms > 100)//10hz 根据剩余缓冲能量设定超点输入功率
    {
        Last_Set_Power_ms = Get_Dwt_ms;
        float CapPowerLimit_In = this->RefereePowerLimit + (this->REFEREE->Power_Heat.chassis_power_buffer - 10.0f)/10.0f;//吃满缓冲能量
        
        this->Pm01_Info->Power_set((CapPowerLimit_In)*100,0x00);
        this->Pm01_Info->Cmd_send(0x02, 0x01);
    }
}
/**
 * @brief 底盘功率控制
 */
void CHASSIS_c::Power_Control()
{
    float MaxPowerLimit = this->CapPowerLimit_Out;
    float Chassis_Power_All = 0;//底盘总功率
    int16_t speed[4];//转子转速
    int16_t I_out[4];//力矩电流
    float P_out[4];//电机输出功率
    float P_in[4];//电机输入功率
    float P_back_electromotive_force[4];//电机反电动势损耗
    float P_heat[4];//电机发热损耗
        
    uint8_t RF = 0;
    uint8_t RB = 1;
    uint8_t LB = 2;
    uint8_t LF = 3;
        
    float P_C620 = 1.25f;//电调功率损耗
    float K = 1.99688994e-6f; // (20/16384)  电调电流与真实电流比     *(0.3)*(187/3591)/9.55   M3509扭矩比例参数   电调值转化为力矩
    float K_1 = 1.453e-07;                 // k1  已经除 K
    float a = 1.23e-07;                     // k2

    /*---转子转速---*/
    //前走 L正 R负
    speed[RF] = this->RF_Motor.DJIMotor_feedback_data.speed;
    speed[RB] = this->RB_Motor.DJIMotor_feedback_data.speed;
    speed[LB] = this->LB_Motor.DJIMotor_feedback_data.speed;
    speed[LF] = this->LF_Motor.DJIMotor_feedback_data.speed;
    /*---当前计算出的力矩电流---*/
    I_out[RF] = this->RF_Motor.DJIMotor_feedback_data.Torque_current;
    I_out[RB] = this->RB_Motor.DJIMotor_feedback_data.Torque_current;
    I_out[LB] = this->LB_Motor.DJIMotor_feedback_data.Torque_current;
    I_out[LF] = this->LF_Motor.DJIMotor_feedback_data.Torque_current;
    /*---输出功率---*/
    P_out[RF] = K * I_out[RF] * speed[RF];
    P_out[RB] = K * I_out[RB] * speed[RB];
    P_out[LB] = K * I_out[LB] * speed[LB];
    P_out[LF] = K * I_out[LF] * speed[LF];
    /*---反电动势损耗---*/
    P_back_electromotive_force[RF] = K_1*speed[RF]*speed[RF];
    P_back_electromotive_force[RB] = K_1*speed[RB]*speed[RB];
    P_back_electromotive_force[LB] = K_1*speed[LB]*speed[LB];
    P_back_electromotive_force[LF] = K_1*speed[LF]*speed[LF];
    /*---电流发热损耗---*/
    P_heat[RF] = a*I_out[RF]*I_out[RF];
    P_heat[RB] = a*I_out[RB]*I_out[RB];
    P_heat[LB] = a*I_out[LB]*I_out[LB];
    P_heat[LF] = a*I_out[LF]*I_out[LF];
    /*---输入功率---*/
    P_in[RF] = P_out[RF] + P_back_electromotive_force[RF]  + P_heat[RF] + P_C620;
    P_in[RB] = P_out[RB] + P_back_electromotive_force[RB]  + P_heat[RB] + P_C620;
    P_in[LB] = P_out[LB] + P_back_electromotive_force[LB]  + P_heat[LB] + P_C620;
    P_in[LF] = P_out[LF] + P_back_electromotive_force[LF]  + P_heat[LF] + P_C620;

       /*---底盘输出功率总和---*/
    for(int i = RF;i <= LF; i++)
    {
        if ( P_in[i] > 0 )  Chassis_Power_All += P_in[i];//忽略瞬时负功率 反充电
    }    

    if ( Chassis_Power_All > MaxPowerLimit )//若总和超功率，则缩放
    {
        float divisor = MaxPowerLimit / Chassis_Power_All;//缩放因子
        /*---目标输入功率等比例缩小---*/
        P_in[RF] = P_in[RF] * divisor;
        P_in[RB] = P_in[RB] * divisor;
        P_in[LB] = P_in[LB] * divisor;
        P_in[LF] = P_in[LF] * divisor;
        /*---非线性缩放计算该输入功率下的输出力矩---*/
        for (int i = RF; i <= LF; i++)
        {
            if ( P_in[i] < 0 )    continue;
            float b = K * speed[i];
            float c = K_1 * speed[i] * speed[i] - P_in[i] + P_C620;
            if ( I_out[i] > 0 )
            {
                float temp = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);//维持原有运动状态
                if (temp > MOTOR_3508_CURRENT_LIMIT)    I_out[i] = MOTOR_3508_CURRENT_LIMIT;
                else                                    I_out[i] = temp;
            }
            else 
            {
                float temp = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);//减速
                if (temp < -MOTOR_3508_CURRENT_LIMIT)    I_out[i] = -MOTOR_3508_CURRENT_LIMIT;
                else                                     I_out[i] = temp;
            }
        }
        /*---更改发送給电调的数据---*/
        this->RF_Motor.DJIMotor_Control_Iutput.current_input =  I_out[RF];
        this->RB_Motor.DJIMotor_Control_Iutput.current_input =  I_out[RB];
        this->LF_Motor.DJIMotor_Control_Iutput.current_input =  I_out[LF];
        this->LB_Motor.DJIMotor_Control_Iutput.current_input =  I_out[LB];
    }
}

CHASSIS_c::CHASSIS_c(void):
LF_Motor(&hcan1, 1, false, M3508, 3500, 15, 0, 0.5, Output_Limit | StepIn, 0.3, 0, 0, NONE),
RF_Motor(&hcan1, 4, true, M3508, 3500, 15, 0, 0.5, Output_Limit | StepIn, 0.3, 0, 0, NONE),
LB_Motor(&hcan1, 3, false, M3508, 3500, 15, 0, 0.5, Output_Limit | StepIn, 0.3, 0, 0, NONE),
RB_Motor(&hcan1, 2, true, M3508, 3500, 15, 0, 0.5, Output_Limit | StepIn, 0.3, 0, 0, NONE),
FIRE_Motor(&hcan1, 5, false, M3508, 3500, 1, 0, 0, Output_Limit, 0.8, 0.3, 100, Integral_Limit | Output_Limit),
Yaw_Motor(&hcan2, 1, false, GM6020, 3500)
{
    LF_Motor.Control_PID.InitSpeedPidMode(Output_Limit, MOTOR_3508_CURRENT_LIMIT, 0);
    RF_Motor.Control_PID.InitSpeedPidMode(Output_Limit, MOTOR_3508_CURRENT_LIMIT, 0);
    LB_Motor.Control_PID.InitSpeedPidMode(Output_Limit, MOTOR_3508_CURRENT_LIMIT, 0);
    RB_Motor.Control_PID.InitSpeedPidMode(Output_Limit, MOTOR_3508_CURRENT_LIMIT, 0);

    LF_Motor.Control_PID.InitSpeedPidMode(StepIn, 12, 0);
    RF_Motor.Control_PID.InitSpeedPidMode(StepIn, 12, 0);
    LB_Motor.Control_PID.InitSpeedPidMode(StepIn, 12, 0);
    RB_Motor.Control_PID.InitSpeedPidMode(StepIn, 12, 0);

    FIRE_Motor.Control_PID.InitSpeedPidMode(Output_Limit, MOTOR_3508_CURRENT_LIMIT, 0);
    FIRE_Motor.Control_PID.InitPositionPidMode(Output_Limit, FIRE_3508_CURRENT_LIMIT, 0);
    FIRE_Motor.Control_PID.InitPositionPidMode(Integral_Limit, 150 , 0);

    MotorManager::AddMotor(&LF_Motor, &hcan1);
    MotorManager::AddMotor(&RF_Motor, &hcan1);
    MotorManager::AddMotor(&LB_Motor, &hcan1);
    MotorManager::AddMotor(&RB_Motor, &hcan1);
    MotorManager::AddMotor(&FIRE_Motor, &hcan1);
    MotorManager::AddMotor(&Yaw_Motor, &hcan2);

    this->REFEREE = Get_REFFEREE();//获取裁判系统数据指针
    this->Pm01_Info = Get_Pm01_Info_Point(bsp_can_e::Bsp_Can1);//超电控制板挂载在can1
    this->Different_Angle_with_Gimbal = 0;
    this->Chassis_State = Chassis_State_e::LOCK;
    this->Chassis_Mode = Chassis_Mode_e::NO_FORCE;
}

CHASSIS_c *CHASSIS_c::Get_Chassis_Instance()
{
    return Chassis_instance;
}
CHASSIS_c *CHASSIS_c::Chassis_instance = new CHASSIS_c; // 构造单例底盘类

