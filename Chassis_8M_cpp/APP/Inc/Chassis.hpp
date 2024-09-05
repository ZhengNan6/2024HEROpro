#ifndef __CHASSIS_HPP__
#define __CHASSIS_HPP__
#include "pid.hpp"
#include "DJI_Motor.hpp"
#include "RC.hpp"
typedef enum
{
    MaiKeNaMu,
    QuanXiang
}Solution_method_e;

typedef enum
{
    MOVE,//移动
    LOCK,//锁车
}Chassis_State_e;

typedef enum
{
    NO_FOLLOW,
    FOLLOW,
    SPIN,
    NO_FORCE,
}Chassis_Mode_e;

class Speed_c
{
public:
    Pid_c Z_speed_pid;
    Solution_method_e Solution_method = MaiKeNaMu;
    int16_t Forward = 0;
    int16_t Right = 0;
    int16_t Spin = 0;
    int16_t RF = 0;
    int16_t RB = 0;
    int16_t LF = 0;
    int16_t LB = 0;
    Speed_c() : Z_speed_pid(1.0, 0.1, 0.01, Output_Limit | Integral_Limit | Deadzone)
    {
        Z_speed_pid.InitMode(Output_Limit, 660, 0);
        Z_speed_pid.InitMode(Integral_Limit, 300, 0);
        Z_speed_pid.InitMode(Deadzone, 5, 0);
    } // 传入合适的参数;

    void Set_Solution_method(Solution_method_e Solution_method);
    void Set(int16_t Forward, int16_t Right, float Different_Angle_with_Gimbal, Chassis_Mode_e Chassis_Mode);
private:
    bool Lock_Flag = 0;
};

// 单例设计模式构造底盘类
class CHASSIS_c
{
public:
    float Different_Angle_with_Gimbal;
    uint16_t MaxPowerLimit;
    Speed_c Speed;
    DJI_Motor_Object Yaw_Motor;
    DJI_Motor_Object RF_Motor;
    DJI_Motor_Object RB_Motor;
    DJI_Motor_Object LF_Motor;
    DJI_Motor_Object LB_Motor;
    DJI_Motor_Object FIRE_Motor;
    Chassis_Mode_e Chassis_Mode;
    Chassis_State_e Chassis_State;
    // 公共接口获取唯一实例
    static CHASSIS_c *Get_Chassis_Instance();
    void SetMode(Chassis_Mode_e Chassis_Mode);
    void SetSpeed(int16_t x, int16_t y);
    void MotorCalc(void);
    void MotorCtrl(void);
    void Power_Control(float MaxPowerLimit);
    void Fire_Motor_Nowork(void);
    void Fire_Motor_Empty(void);
    void Fire_Motor_Work(void);
    void Fire_Motor_Work(int64_t SetEncoder);

private:
    // 私有构造函数
    CHASSIS_c(void);
    void Update_Different_Angle_with_Gimbal(void);
    static CHASSIS_c *Chassis_instance;
};
#endif