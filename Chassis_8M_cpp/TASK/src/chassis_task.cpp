#include "chassis_task.h"
#include "Chassis.hpp"
#include "RC.hpp"
class Control_Move_c
{
public:
    ECF_RC *RC;
    CHASSIS_c* chassis;
    float Max_Power_Limit = 55;
    float Forward_Speed;//前进为正
    float Right_Speed;//右移为正
    Chassis_Mode_e Chassis_Mode;//底盘模式
    Control_Move_c();
    void Get_Cmd(void);
    void Set_Cmd(void);
    void Cmd_Calc(void);
    void Cmd_Ctrl(void);
};

/**
 * @brief 底盘控制类构造初始化
 * @note  获取关键上位机指针以及底盘实例类
 */
Control_Move_c::Control_Move_c(void)
{
    chassis =  CHASSIS_c::Get_Chassis_Instance();
    RC = ECF_RC::Get_ECF_RC_Instance();
}
/**
 * @brief 获取 来自Dr16 或 图传模块的上位机指令
 * @note  
 */
void Control_Move_c::Get_Cmd(void)
{
    this->Forward_Speed = ((this->RC->ctrl.rc.ch[0] ) + (-this->RC->ctrl.kb.bit.A + this->RC->ctrl.kb.bit.D) * 660 );
    this->Right_Speed = -((this->RC->ctrl.rc.ch[1] ) + (-this->RC->ctrl.kb.bit.S + this->RC->ctrl.kb.bit.W) * 660 );
    
    static Chassis_Mode_e last_Chassis_Mode = NO_FOLLOW;
    static Chassis_Mode_e RC_Chassis_Mode = NO_FOLLOW;//上一次遥控设定的模式
    /*手柄*/
    switch(RC->ctrl.rc.s1)
    {
        case RC_SW_DOWN://遥控器左侧开关状态为[下]
            RC_Chassis_Mode = NO_FOLLOW;//底盘不跟随云台
            break;
        case RC_SW_MID://遥控器左侧开关状态为[中]
            RC_Chassis_Mode = FOLLOW;//底盘跟随云台
            break;
        case RC_SW_UP://遥控器左侧开关状态为[上]
            RC_Chassis_Mode = SPIN;//底盘小陀螺模式
            break;
        default://通道值错误
            RC_Chassis_Mode = NO_FORCE;
            break;
    }
    //检测到挡把改变后再修改底盘模式，避免用键盘设置的模式数据在松开按键后被手柄的挡把数据覆盖
    if(last_Chassis_Mode != RC_Chassis_Mode)
    {
        last_Chassis_Mode = RC_Chassis_Mode;
        this->Chassis_Mode = RC_Chassis_Mode;
    }
    /*键盘*/
    if(this->RC->ctrl.kb.bit.Q == 1)  this->Chassis_Mode = FOLLOW;
    if(this->RC->ctrl.kb.bit.E == 1)  this->Chassis_Mode = SPIN;
    if(this->RC->ctrl.kb.bit.R == 1)  this->Chassis_Mode = NO_FOLLOW; 

}

void Control_Move_c::Set_Cmd(void)
{
    this->chassis->SetMode(this->Chassis_Mode);
    this->chassis->SetSpeed(this->Forward_Speed, this->Right_Speed);
}

void Control_Move_c::Cmd_Calc(void)
{
    this->chassis->MotorCalc();
    this->chassis->Power_Control(this->Max_Power_Limit);
}

void Control_Move_c::Cmd_Ctrl(void)
{
    this->chassis->MotorCtrl();
}

Control_Move_c Control_Move;
void chassis_task(void const *argument)
{
    while (1)
    {
        Control_Move.Get_Cmd();//获取上位机指令
        Control_Move.Set_Cmd();//设定控制机体
        Control_Move.Cmd_Calc();//机体控制运算
        Control_Move.Cmd_Ctrl();//发送控制电流
        vTaskDelay(1);
    }
}