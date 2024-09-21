#ifndef __GIMBAL_HPP__
#define __GIMBAL_HPP__
#include "DJI_Motor.hpp"
#include "RC.hpp"
#include "imu_task.h"

typedef enum
{
    NotLock,
    IMU,
    Encoder
}Lock_Mode_e;

class Motor_Control_c{
    public:
    Lock_Mode_e Lock_Mode = NotLock;
    float       Set_Encoder;
    float       Set_IMU;
    const float *Actual_Encoder;
    const float *Actual_IMU;
    Motor_Control_c();
    Motor_Control_c(const float * Actual_Encoder, const float * Actual_IMU);
    void Set_Control(Lock_Mode_e Set_Lock_Mode, float Set_Value);
    void Get_Actual_Encoder_Point(const float * Actual_Encoder);
    void Get_Actual_IMU(const float * Actual_IMU);
};

class Fire_c{
public:
    DJI_Motor_Object FIRE_RH_Motor;
    DJI_Motor_Object FIRE_RL_Motor;
    DJI_Motor_Object FIRE_LH_Motor;
    DJI_Motor_Object FIRE_LL_Motor;

    bool Fire_Star;
    bool Fire_Ready;
    uint16_t Set_Speed;
    Fire_c();
    void SendFireReady(void);
    void SetSpeed(uint16_t FireSpeed);
    void SetStar(bool Set);
};

typedef enum
{
    Mammual,
    LongShot,
    Auto
}Gimbal_Mode_e;

// 单例设计模式构造云台类
class GIMBAL_c
{
public:
    Fire_c Fire;
    Gimbal_Mode_e Mode;
    Motor_Control_c  Pitch_Ctrl;
    Motor_Control_c  Yaw_Ctrl;
    DJI_Motor_Object Pitch_Motor;
    DJI_Motor_Object Yaw_Motor;
    // 公共接口获取唯一实例
    static GIMBAL_c *Get_Gimbal_Instance();
private:
    // 私有构造函数
    GIMBAL_c(void);
    static GIMBAL_c *Gimbal_instance;
};
#endif