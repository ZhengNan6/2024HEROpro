#include "Chassis.hpp"
#include "RC.h"

#ifdef  __cplusplus
extern "C"
{
#endif
    
#include "fire_task.h"
#include "bsp_can.h"
    
#ifdef  __cplusplus
}
#endif

#define ABS(X) ((X)<0?(-X):(X))
typedef enum
{
    Ready_Fire,   //开摩擦轮
    NO_Fire_FORCE,//关摩擦轮
    On_Empty      //退弹中
}Fire_State_e;//发弹状态机

typedef struct
{
    bool Fire_Ready = false;
    bool Allow_Shoot = false;
}Gimbal_Fire_Info_t;

class Control_Fire_c
{
public:
    ECF_RC *RC;
    CHASSIS_c* chassis;
    Fire_State_e Fire_State;
    Gimbal_Fire_Info_t Gimbal_Fire_Info;
    uint8_t Shoot_One;
    int64_t set_position;
    int64_t empty_position;
    bool    Save_Empty_position = false;
    bool    Delay_Shoot_this = true;
    Control_Fire_c(void);
    void Get_Cmd(void);
    void Set_Cmd(void);
private:
    void To_Shoot_Position(uint8_t num);
    void To_Empty_Position();
    void Clear_Empty_Position();
    void Clear_Set_Position();
    void Save_Empty_Position();
};
Control_Fire_c Control_Fire;
/*
云台下发内容：
摩擦轮开关状态
自瞄是否认为可开火
由于仅2bit信息，因此由云台的遥控帧控制
canID 0x01通道已被 RC转发占用

在摩擦轮关状态下，无论是否自瞄认为可开火都不允许开火
采用 0x02 - 0x04通道表达 3种状态
*/
void Deal_Gimbal_Fire_Info_0x02(CAN_RxHeaderTypeDef *Rxmessage, uint8_t *DATA)
{
    Control_Fire.Gimbal_Fire_Info.Fire_Ready = false;
    Control_Fire.Gimbal_Fire_Info.Allow_Shoot = false;
}
void Deal_Gimbal_Fire_Info_0x03(CAN_RxHeaderTypeDef *Rxmessage, uint8_t *DATA)
{
    Control_Fire.Gimbal_Fire_Info.Fire_Ready = true;
    Control_Fire.Gimbal_Fire_Info.Allow_Shoot = false;
}
void Deal_Gimbal_Fire_Info_0x04(CAN_RxHeaderTypeDef *Rxmessage, uint8_t *DATA)
{
    Control_Fire.Gimbal_Fire_Info.Fire_Ready = true;
    Control_Fire.Gimbal_Fire_Info.Allow_Shoot = true;
}

/**
 * @brief 开火控制类构造初始化
 * @note  获取关键上位机指针以及底盘实例类
 */
Control_Fire_c::Control_Fire_c(void)
{
    chassis =  CHASSIS_c::Get_Chassis_Instance();
    RC = ECF_RC::Get_ECF_RC_Instance();
    ECF_CAN_Rx_Callback_Register(Bsp_Can2, Bsp_Stdid, 0X02, Deal_Gimbal_Fire_Info_0x02);
    ECF_CAN_Rx_Callback_Register(Bsp_Can2, Bsp_Stdid, 0X03, Deal_Gimbal_Fire_Info_0x03);
    ECF_CAN_Rx_Callback_Register(Bsp_Can2, Bsp_Stdid, 0X04, Deal_Gimbal_Fire_Info_0x04);
}
/**
 * @brief 更改为发射指定颗数后的拨弹盘位置
 * @param num 发射颗数
 */
void Control_Fire_c::To_Shoot_Position(uint8_t num)
{
    this->set_position -= 69632*num;
}
/**
 * @brief 更改为退弹指令前拨弹盘设定位置
 */
void Control_Fire_c::To_Empty_Position(void)
{
    this->set_position = this->empty_position;
}
/**
 * @brief 清除退弹指令前拨弹盘设定位置
 */
void Control_Fire_c::Clear_Empty_Position(void)
{
    this->empty_position = 0;
    this->Save_Empty_position = false;
}
/**
 * @brief 拨弹盘设定位置
 */
void Control_Fire_c::Clear_Set_Position(void)
{
    this->set_position = 0;
}
/**
 * @brief 保存退弹指令前拨弹盘设定位置
 */
void Control_Fire_c::Save_Empty_Position(void)
{
    this->empty_position = this->chassis->FIRE_Motor.Encoder.Encode_Record_Val;
    this->Save_Empty_position = true;
}
/**
 * @brief 根据云台摩擦轮开关状态、自瞄开火指令，以及上位机开火指令设定拨弹盘状态、设定位置
 */
void Control_Fire_c::Get_Cmd(void)
{
    if (this->RC->ctrl.rc.ch[4] <-200 || this->RC->ctrl.kb.bit.M == 1)
    {
        this->Fire_State = On_Empty;//拨弹盘倒退
    }   
    else 
    {
        if (this->Gimbal_Fire_Info.Fire_Ready)
            this->Fire_State = Ready_Fire;//拨弹盘有力
        else
            this->Fire_State = NO_Fire_FORCE;//拨弹盘无力
    }

    static bool Fire_cmd_reset = true;     
    if (this->RC->ctrl.rc.ch[4] <200 && this->RC->ctrl.mouse.press_l == 0) 
        Fire_cmd_reset = true;//回正检测
        
    if ( (this->RC->ctrl.rc.ch[4] >= 550 || this->RC->ctrl.mouse.press_l) 
          && Fire_cmd_reset 
          && this->Gimbal_Fire_Info.Allow_Shoot
          && this->Fire_State == Ready_Fire
          //&& this->Delay_Shoot_this
        )
    {
        Fire_cmd_reset = false;
        this->Shoot_One++;
        this->Delay_Shoot_this = false;
    }
}
/**
 * @brief 设定拨弹盘位置，计算、发送电调控制量
 */
void Control_Fire_c::Set_Cmd(void)
{
    switch (this->Fire_State)
    {
    case Ready_Fire:
        //存在待发弹
        if (this->Shoot_One > 0)
        {
            if (Save_Empty_position)//之前存在退弹指令
                To_Empty_Position();//前往退弹指令前未到达位置
            else
                To_Shoot_Position(this->Shoot_One);//否则正常打出一颗弹
                
            this->Shoot_One = 0;
            Clear_Empty_Position();//清除指令保存的未到达位置
            this->chassis->Fire_Motor_Work(this->set_position);//更新到发射位置
        } 
        else
            this->chassis->Fire_Motor_Work();

        if( ABS(this->chassis->FIRE_Motor.Encoder.Encode_Record_Val - this->set_position) < 5000)//检查进弹行程
            this->Delay_Shoot_this = true;
        break;
    case NO_Fire_FORCE://无力
        Clear_Empty_Position();
        Clear_Set_Position();

        this->chassis->Fire_Motor_Nowork();
    case On_Empty://拨弹盘倒退，停止后定在原地
        if (!Save_Empty_position) //保存首次倒退位置
            Save_Empty_Position();

        this->chassis->Fire_Motor_Empty();
    default:
        break;
    }
}


void fire_task(void const *argument)
{
    while (1)
    {
        Control_Fire.Get_Cmd();
        Control_Fire.Set_Cmd();
        vTaskDelay(1);
    }
}