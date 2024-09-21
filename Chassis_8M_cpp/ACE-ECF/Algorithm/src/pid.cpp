#include "pid.hpp"
#include <string.h>


#define abs(x) ((x) > (0) ? (x) : (-(x)))


/*--------------------函数声明-----------------------*/
static void this_pid_clear(pid_parameter_t *pid);
static float this_PidCalculate(pid_parameter_t *pid, float SetValue, float ActualValue);
static float first_order_filter(first_order_filter_t *first_order_filter_type, float input);
static void f_Separated_Integral(pid_parameter_t *pid);
static void f_Integral_Limit(pid_parameter_t *pid);
static void f_Derivative_On_Measurement(pid_parameter_t *pid);
static void Changing_Integration_Rate(pid_parameter_t *pid);
static void f_Output_Limit(pid_parameter_t *pid);
static void f_StepIn(pid_parameter_t *pid);

void Pid_c::PidInit(pid_parameter_t *pid, float kp, float ki, float kd, uint32_t mode)
{
    memset(pid, 0, sizeof(pid_parameter_t));
    pid->User_Fun = nullptr;
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->mode = mode;
}
void Pid_c::PidInitMode(pid_parameter_t *pid, uint32_t mode, float num1, float num2)
{
    switch (mode)
    {
    case NONE:
        return;
    case Integral_Limit:
        pid->max_Ierror = num1;
        return;
    case Derivative_On_Measurement:
        pid->gama = num1;
        return;
    case Separated_Integral:
        pid->threshold_max = num1;
        pid->threshold_min = num2;
        return;
    case Output_Limit:
        pid->max_out = num1;
        return;
    case OutputFilter:
        pid->out_filter.num = num1;
        return;
    case ChangingIntegrationRate:
        pid->errorabsmax = num1;
        pid->errorabsmin = num2;
        return;
    case DerivativeFilter:
        pid->d_filter.num = num1;
        return;
    case Deadzone:
        pid->deadband = num1;
        return;
    case StepIn:
        pid->stepIn = num1;
        return;
    }
    return;
}
Pid_c::Pid_c(void)
{
    ;
}

Pid_c::Pid_c(float kp, float ki, float kd, uint32_t mode)
{
    this->PidInit(&this->Speed_PID, kp, ki, kd, mode);
}

Pid_c:: Pid_c(float speed_kp,    float speed_ki,    float speed_kd,    uint32_t speed_mode,
              float position_kp, float position_ki, float position_kd, uint32_t position_mode)
{
    this->PidInit(&this->Speed_PID, speed_kp, speed_ki, speed_kd, speed_mode);
    this->PidInit(&this->Position_PID, position_kp, position_ki, position_kd, position_mode);
}
void Pid_c::InitMode(uint32_t mode, float num1, float num2)
{
    InitSpeedPidMode(mode, num1, num2);
}
void Pid_c::InitSpeedPidMode(uint32_t mode, float num1, float num2)
{
    PidInitMode(&this->Speed_PID, mode, num1, num2);
}
void Pid_c::InitPositionPidMode(uint32_t mode, float num1, float num2)
{
    PidInitMode(&this->Position_PID, mode, num1, num2);
}


void Pid_c::pid_clear()
{
    this_pid_clear(&this->Speed_PID);
    this_pid_clear(&this->Position_PID);
}

float Pid_c::PidCalculate(float SetValue, float ActualValue)
{
    return this_PidCalculate(&this->Speed_PID, SetValue, ActualValue);
}
float Pid_c::PidCalculate(float Outer_PID_Set_Value, float Outer_PID_ActualValue, float Inner_loop_PID_ActualValue)
{
    this->Speed_PID.SetValue = this_PidCalculate(&this->Position_PID, Outer_PID_Set_Value, Outer_PID_ActualValue); // 速度环设定值由位置环处理
    return this_PidCalculate(&this->Speed_PID, this->Speed_PID.SetValue, Inner_loop_PID_ActualValue);              // 电机输出量
}

/**
 * @brief          电机速度环控制算法
 * @param[in]      SetValue 设定速度值
 * @param[in]      ActualValue 实际值
 * @retval         计算值
 */
int16_t Pid_c::motor_speed_control(float SetValue, float ActualValue)
{
    return this_PidCalculate(&this->Speed_PID, SetValue, ActualValue);
}
/**
 * @brief          电机速度位置环串级控制算法
 * @param[in]      速度环pid结构体
 * @param[in]      位置环pid结构体
 * @param[in]      设定位置值
 * @param[in]      实际位置值
 * @param[in]      实际速度值
 * @retval         计算值
 */
int16_t Pid_c::motor_position_speed_control(float setPosition, float actual_position, float actual_speed)
{
    this->Speed_PID.SetValue = this_PidCalculate(&this->Position_PID, setPosition, actual_position); // 速度环设定值由位置环处理
    return this_PidCalculate(&this->Speed_PID, this->Speed_PID.SetValue, actual_speed);              // 电机输出量
}
/*------------------------------以下为内部函数---------------------------------*/
static float this_PidCalculate(pid_parameter_t *pid, float SetValue, float ActualValue)
{
    pid->SetValue = SetValue;
    // 步进式pid
    if (pid->mode & StepIn)
        f_StepIn(pid);
    pid->ActualValue = ActualValue;
    pid->error = pid->SetValue - pid->ActualValue;
    pid->Derror = pid->error - pid->LastError;
    if (abs(pid->error) > pid->deadband) // 死区
    {
        pid->Pout = pid->error * pid->Kp;
        pid->Ierror += pid->error;

        // 微分先行
        if (pid->mode & Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);
        else
            pid->Dout = pid->Kd * pid->Derror;

        // 变积分
        if (pid->mode & ChangingIntegrationRate)
            Changing_Integration_Rate(pid);

        // 积分限幅
        if (pid->mode & Integral_Limit)
            f_Integral_Limit(pid);
        pid->Iout = pid->Ki * pid->Ierror;

        // 积分分离 注意需要放在iout计算后
        if (pid->mode & Separated_Integral)
            f_Separated_Integral(pid);

        // 微分滤波
        if (pid->mode & DerivativeFilter)
            pid->Dout = first_order_filter(&pid->d_filter, pid->Dout);

        pid->out = pid->Pout + pid->Iout + pid->Dout;

        // 输出滤波
        if (pid->mode & OutputFilter)
            pid->out = first_order_filter(&pid->out_filter, pid->out);

        // 输出限幅
        if (pid->mode & Output_Limit)
            f_Output_Limit(pid);
    }
    else
    {
        pid->out = 0;
        //this_pid_clear(pid);
    }

    pid->LastActualValue = pid->ActualValue;
    pid->LastSetValue = pid->SetValue;
    pid->LastDerror = pid->Derror;
    pid->LastError = pid->error;

    return pid->out;
}
/**
 * @brief          PID清除
 * @param[in]      待清除的pid结构体
 * @retval         none
 * @attention      只是清除所有计算的数据，不会清除pid或者模式的数据
 */
static void this_pid_clear(pid_parameter_t *pid)
{
    pid->error = pid->LastError = 0.0f;
    pid->Derror = pid->LastDerror = pid->LastLastDerror = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = pid->Ierror = 0.0f;
    pid->ActualValue = pid->SetValue = pid->LastActualValue = pid->LastSetValue = 0.0f;
    pid->d_filter.last_input = pid->out_filter.last_input = 0;
}

/**
 * @brief          一阶低通滤波计算
 * @param[in]      低通滤波结构体
 * @param[in]      输入值
 * @retval         输出值
 * @attention      none
 */
float first_order_filter(first_order_filter_t *first_order_filter_type, float input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out = first_order_filter_type->input * first_order_filter_type->num + (1 - first_order_filter_type->num) * first_order_filter_type->last_input;
    first_order_filter_type->last_input = first_order_filter_type->out;

    return first_order_filter_type->out;
}
/**
 * @brief          微分先行pid
 * @param[in]      pid结构体
 * @retval         none
 * @attention      似乎存在问题 详见 https://blog.csdn.net/foxclever/article/details/80633275
 */
void f_Derivative_On_Measurement(pid_parameter_t *PID)
{
    float c1, c2, c3, temp;

    temp = PID->gama * PID->Kd + PID->Kp;
    c3 = PID->Kd / temp;
    c2 = (PID->Kd + PID->Kp) / temp;
    c1 = PID->gama * c3;
    PID->Dout = c1 * PID->Dout + c2 * PID->ActualValue + c3 * PID->LastActualValue;
}
/**
 * @brief          步进式pid
 * @param[in]      pid结构体
 * @retval         none
 * @attention      详见 https://blog.csdn.net/foxclever/article/details/81151898
 */
void f_StepIn(pid_parameter_t *pid)
{
    float kFactor = 0.0f;
    //    if ((pid->LastSetValue - pid->SetValue <= pid->stepIn) && (pid->LastSetValue - pid->SetValue >= pid->stepIn))
    //    {
    //        return;
    //    }
    if (abs(pid->LastSetValue - pid->SetValue) <= pid->stepIn)
    {
        return;
    }
    else
    {
        if ((pid->LastSetValue - pid->SetValue) > 0.0f)
        {
            kFactor = -1.0f;
        }
        else if ((pid->LastSetValue - pid->SetValue) < 0.0f)
        {
            kFactor = 1.0f;
        }
        else
        {
            kFactor = 0.0f;
        }
        pid->SetValue = pid->LastSetValue + kFactor * pid->stepIn;
    }
}
/**
 * @brief          积分分离
 * @param[in]      pid结构体
 * @retval         none
 * @attention      当超过阈值的时候吧iout清零就行
 */
void f_Separated_Integral(pid_parameter_t *pid)
{
    if (pid->threshold_min > pid->error && pid->error < pid->threshold_max)
        pid->Iout = 0;
}
/**
 * @brief          积分限幅
 * @param[in]      pid结构体
 * @retval         none
 * @attention      当超过阈值的时候吧iout清零就行
 */
void f_Integral_Limit(pid_parameter_t *pid)
{
    if (pid->Ierror > pid->max_Ierror)
    {
        pid->Ierror = pid->max_Ierror;
    }
    if (pid->Ierror < -(pid->max_Ierror))
    {
        pid->Ierror = -(pid->max_Ierror);
    }
}
/**
 * @brief          变积分系数处理函数，实现一个输出0和1之间的分段线性函数
 * @param[in]      pid结构体
 * @retval         none
 * @attention      当偏差的绝对值小于最小值时，输出为1；当偏差的绝对值大于最大值时，输出为0
 *                 当偏差的绝对值介于最大值和最小值之间时，输出在0和1之间线性变化
 */
void Changing_Integration_Rate(pid_parameter_t *pid)
{
    if (abs(pid->error) <= pid->errorabsmin) // 最小值
    {
        return;
    }
    else if (abs(pid->error) > pid->errorabsmax) // 最大值
    {
        pid->Ierror = 0.0f;
    }
    else
    {
        pid->Ierror *= ((pid->errorabsmax - abs(pid->error)) / (pid->errorabsmax - pid->errorabsmin));
    }
}
/**
 * @brief          输出限幅
 * @param[in]      pid结构体
 * @retval         none
 * @attention      none
 */
void f_Output_Limit(pid_parameter_t *pid)
{
    if (pid->out > pid->max_out)
    {
        pid->out = pid->max_out;
    }
    if (pid->out < -(pid->max_out))
    {
        pid->out = -(pid->max_out);
    }
}

