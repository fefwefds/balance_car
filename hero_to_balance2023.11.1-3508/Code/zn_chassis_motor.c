/**
  ****************************(C) 智 能 引 擎 战 队 ****************************
  * @file       zn_chassis_motor.c/h
  * @brief      底盘电机控制
  * @note       
  * @history
  *  Version    Date                Author                        Modification
  *  V1.0.0     Jan-7-2022         Intelligent Engine             1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) Intelligent Engine ****************************
**/

#include "zn_chassis_motor.h"
#include "cmsis_os.h"
#include "main.h"
#include "zn_pid.h"
#include "zn_interaction.h"

pid_type_def chassis_follow;

/**
  * @brief  底盘随动pid
  * @param  void
  * @retval void
  * @attention
  */
float Chassis_Follow(pid_type_def *ptr)
{
  ptr->error = Robot1.Mechanical.YAW_Motor_Middle - yaw_motor.data.angle;

     //过零处理,统一成劣弧
  if(ptr->error>( 8192/2))ptr->error = ptr->error - 8192;
  if(ptr->error<(-8192/2))ptr->error = ptr->error + 8192;   
  
  ptr->error_all += ptr->error;
     
   //积分限幅
//  Limit(ptr->error_all,0.2,-0.2);   
     
   ptr->P_out = ptr->kp * ptr->error;
   ptr->I_out = ptr->ki * ptr->error_all;
   ptr->D_out = ptr->kd * (ptr->error - ptr->error_last);
     
   ptr->error_last = ptr->error;
     
   ptr->All_out = ptr->P_out + ptr->I_out + ptr->D_out;
     
   //输出限幅
    Limit(ptr->All_out,3,-3);   
    
    return ptr->All_out;
}

