#include "Gimbal.hpp"

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
