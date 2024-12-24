#include "zn_gimbal_motor.h"
#include "INS_task.h"

pid_type_def YAW_imu_angle ;//yaw电机陀螺仪角度环pid参数
pid_type_def YAW_imu_speed ;

float YAW_imuangle_cal(pid_type_def *ptr,float yaw_angle,float yaw_angle_set)
{ 
  ptr->error = yaw_angle_set - yaw_angle;

  //过零处理,统一成劣弧
  if(ptr->error>=( 360/2))ptr->error = ptr->error - 360;
  if(ptr->error< (-360/2))ptr->error = ptr->error + 360;
  
  ptr->error_all += ptr->error;//偏差积分
  
  //积分限幅
  
  ptr->P_out = ptr->kp * ptr->error;
  ptr->I_out = ptr->ki * ptr->error_all;
  ptr->D_out = ptr->kd * (ptr->error - ptr->error_last);
  
  ptr->error_last = ptr->error;
  
  ptr->All_out = ptr->P_out + ptr->I_out + ptr->D_out;
  
  //输出限幅
  //死区限制
  
   return ptr->All_out;
}


//陀螺仪模式 电机速度内环
float Motor_imuspeed_cal(pid_type_def *ptr,float imuspeed,float imuspeed_set)
{
  ptr->error = imuspeed_set - imuspeed;
  
  ptr->error_all += ptr->error;//偏差积分
    //积分限幅
//  Limit(ptr->error_all,ptr->IntegralLimit,-ptr->IntegralLimit);
  
  ptr->P_out = ptr->kp * ptr->error;
  ptr->I_out = ptr->ki * ptr->error_all;
  ptr->D_out = ptr->kd * (ptr->error - ptr->error_last);

  ptr->error_last = ptr->error;
  
  ptr->All_out = ptr->P_out + ptr->I_out + ptr->D_out;
  
  //输出限幅
  Limit(ptr->All_out,25000, -25000);//30000
  //死区限制
  
   return ptr->All_out;     
}

int16_t afsfds;
void YAW_Angle_Set(Motor_type_def *motor,pid_type_def *angle_pid,pid_type_def *speed_pid,float yaw_angle_set)
{
  YAW_imuangle_cal(angle_pid,INS_angle[0],yaw_angle_set);
  Motor_imuspeed_cal(speed_pid,motor->data.speed_rpm,(int16_t)angle_pid->All_out);//速度环计算
  CAN_Set_Current(motor,(int16_t)speed_pid->All_out);//yaw轴电流输出
//  CAN_Set_Current(motor,afsfds);
}