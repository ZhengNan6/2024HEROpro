/*************************** Dongguan-University of Technology -ACE**************************
 * @file    pid.h
 * @author  郑杰瀚
 * @version V1.0.3
 * @date    2022/11/19
 * @brief
 ******************************************************************************
 * @verbatim
 *  已经把常用的pid模式都加进去了，还有滤波器一些没有实现
 *  使用方法：
 *      先创建一个pid结构体。
 *      初始化：
 *            先使用PidInit，注意模式是在这里设置，用 | 链接
 *            然后使用PidInitMode将每个模式逐个赋值
 *      使用：PidCalculate
 *            pid_clear
 *            User_Fun_Callback_Register
 *  demo：
 *       pid_parameter_t motor6020pid_s;//定义一个pid结构体
 *       PidInit(&motor6020pid_s,20,0,0,Output_Limit | Integral_Limit);//使用输出限幅和积分限幅模式
 *       PidInitMode(&motor6020pid_s,Integral_Limit,1000,0);//积分限幅模式设置
 *	    PidInitMode(&motor6020pid_s,Output_Limit,30000,0);//输出限幅模式设置
 *          while(1)
 *          {
 *             ActualValue = sersor();                     //获取数据
 *             setvalue  = setclaculate();                 //获取数据
 *             PidCalculate(&motor6020pid_s,setvalue,ActualValue);   //计算
 *          }
 *
 * @attention
 *      请确保User_Fun_Callback_Register注册了函数再使用结构体中的自定义函数
 *      死区默认是使用的，如果不使用，不用理会就行，使用了会在死区内清除pid。
 * @version
 * v1.0.1 添加了步进式PID，所有功能待验证
 * V1.0.2 添加了很多注释，Integral_Limit ，Output_Limit ，StepIn模式已验证，Derivative_On_Measurement似乎有点问题
 * V1.0.3 添加了积分和输出滤波，待验证
 ************************** Dongguan-University of Technology -ACE***************************/
#ifndef __PID_HPP
#define __PID_HPP
#include <stdint.h>

typedef enum
{
    NONE = 0X00,                      // 0000 0000 无
    Deadzone = 0x01,                  // 0000 0001 死区
    Integral_Limit = 0x02,            // 0000 0010 积分限幅
    Output_Limit = 0x04,              // 0000 0100 输出限幅
    Derivative_On_Measurement = 0x08, // 0000 1000 微分先行 TODO:
    Separated_Integral = 0x10,        // 0001 0000 积分分离
    ChangingIntegrationRate = 0x20,   // 0010 0000 变积分
    OutputFilter = 0x40,              // 0100 0000 输出滤波
    DerivativeFilter = 0x80,          // 1000 0000 微分滤波
    StepIn = 0x0100,                  // 0000 0001 0000 0000 步进式
} PID_mode_e;

// 一阶低通滤波参数
typedef struct
{
    float input;      // 输入数据
    float last_input; // 上次数据
    float out;        // 滤波输出的数据
    float num;        // 滤波参数
} first_order_filter_t;

typedef struct Pid_parameter_t // pid结构体变量
{
    float Kp;
    float Ki;
    float Kd;

    float SetValue;
    float LastSetValue;
    float LastActualValue;
    float ActualValue;

    float Ierror;
    float Pout;
    float Iout;
    float Dout;
    float out;

    float Derror; // 微分项
    float LastDerror;
    float LastLastDerror;
    float error; // 误差项
    float LastError;

    float max_out; // 最大输出

    uint32_t mode; // pid模式

    /* 积分限幅 */
    float max_Ierror; // 最大积分输出
    /* 误差死区 */
    float deadband;
    /* 积分分离 */
    float threshold_max; // 积分分离最大值
    float threshold_min; // 积分分离最小值
    /* 抗积分饱和 */
    // float maximum; //最大值
    // float minimum; //最小值
    /* 变积分 */
    float errorabsmax; // 偏差绝对值最大值
    float errorabsmin; // 偏差绝对值最小值
    /* 微分先行 */
    float gama; // 微分先行滤波系数
    /* 微分滤波 */
    first_order_filter_t d_filter; // 微分滤波结构体
    /* 输出滤波 */
    first_order_filter_t out_filter; // 输出滤波结构体

    /* 步进数 */
    float stepIn;

    void (*User_Fun)(struct Pid_parameter_t *);
} pid_parameter_t;

typedef enum
{
    Normal,
    Double
} PID_Num_e;

class Pid_c
{
public:
    pid_parameter_t Speed_PID;
    pid_parameter_t Position_PID;
    PID_Num_e PID_Num;
    Pid_c(void);
    Pid_c(float kp, float ki, float kd, uint32_t mode);
    Pid_c(float speed_kp, float speed_ki, float speed_kd, uint32_t speed_mode,
          float position_kp, float position_ki, float position_kd, uint32_t position_mode);
    void InitMode(uint32_t mode, float num1, float num2);
    void InitSpeedPidMode(uint32_t mode, float num1, float num2);
    void InitPositionPidMode(uint32_t mode, float num1, float num2);

    void pid_clear();

    float PidCalculate(float SetValue, float ActualValue);
    float PidCalculate(float Outer_PID_Set_Value , float Outer_PID_ActualValue, float Inner_loop_PID_ActualValue);

    int16_t motor_speed_control(float setSpeed, float actualSpeed);
    int16_t motor_position_speed_control(float setPosition, float actual_position, float actual_speed);

private:
    void PidInit(pid_parameter_t *PID, float kp, float ki, float kd, uint32_t mode);
    void PidInitMode(pid_parameter_t *pid, uint32_t mode, float num1, float num2);
};

#endif
