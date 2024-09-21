#include "Gimbal.hpp"
#include "can.h"
extern CAN_HandleTypeDef hcan1;


Fire_c::Fire_c():
FIRE_RH_Motor(&hcan1, 1, false, GM6020, 1),
FIRE_RL_Motor(&hcan1, 1, false, GM6020, 1),
FIRE_LH_Motor(&hcan1, 1, false, GM6020, 1),
FIRE_LL_Motor(&hcan1, 1, false, GM6020, 1)
{
    ;
}

Motor_Control_c::Motor_Control_c()
{
    ;
}
/**
 * @brief 填入电机控制类实际变量地址
 * @param Actual_Encoder 实际控制电机编码值的地址
 * @param Actual_IMU     实际控制电机轴陀螺仪欧拉角的地址
 */
Motor_Control_c::Motor_Control_c(const float * Actual_Encoder, const float * Actual_IMU)
{
    this->Actual_Encoder = Actual_Encoder;
    this->Actual_IMU = Actual_IMU;
}

GIMBAL_c::GIMBAL_c():
Pitch_Motor(&hcan1, 1, false, GM6020, 1),
Yaw_Motor(&hcan2, 1, false, GM6020, 1)
{
}