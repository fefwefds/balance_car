#include "zn_chassis_task.h"
#include "bsp_laser.h"
#include "ANO_DT.h"
#include "remote_control.h"
#include "math.h"
#include "zn_gimbal_task.h"
#include "zn_interaction.h"
#include "string.h"
#include "zn_chassis_motor.h"
#include "zn_judge.h"
#include "zn_pid.h"
#include "zn_motor.h"
#include "INS_task.h"



CurrentLimit fCurrentLimit;

void ZN_chassis_task(void const *pvParameters)
{	
  static portTickType currentTime;
  
  ROBOT_Para_Init();
  
  Motor_All_Para_Init();//所有电机参数初始化
  
  chassis_follow.kp = 0.002;
  chassis_follow.kd = 0.01;
  
  chassis_contrl.Balance_angle_offset=-3;
  chassis_contrl.torque_const=8;//转矩常数
  chassis_contrl.Diameter_weel=1.2f;
  
  PID_init(&LQR_k1,PID_POSITION,pid_LQR_k1,PID_LQR_k1_MAX_OUT,PID_LQR_k1_MAX_OUT);
  PID_init(&LQR_k2,PID_POSITION,pid_LQR_k2,PID_LQR_k2_MAX_OUT,PID_LQR_k2_MAX_OUT);
  PID_init(&LQR_k3,PID_POSITION,pid_LQR_k3,PID_LQR_k3_MAX_OUT,PID_LQR_k3_MAX_OUT);
  PID_init(&LQR_k4,PID_POSITION,pid_LQR_k4,PID_LQR_k4_MAX_OUT,PID_LQR_k4_MAX_OUT);
  PID_init(&LQR_k5,PID_POSITION,pid_LQR_k5,PID_LQR_k5_MAX_OUT,PID_LQR_k5_MAX_OUT);
  PID_init(&LQR_k6,PID_POSITION,pid_LQR_k6,PID_LQR_k6_MAX_OUT,PID_LQR_k6_MAX_OUT);
  PID_init(&Offset_PID,PID_DELTA,pid_Offset,PID_Offset_MAX_OUT,PID_Offset_MAX_IOUT);
  PID_init(&SlideBlock_PID[0],PID_POSITION,chassis_SlideBlock_PID,PID_SlideBlock_Speed_MAX_OUT,PID_SlideBlock_Speed_MAX_IOUT);
  PID_init(&SlideBlock_PID[1],PID_POSITION,chassis_SlideBlock_PID,PID_SlideBlock_Speed_MAX_OUT,PID_SlideBlock_Speed_MAX_IOUT);
  
  vTaskDelayUntil(&currentTime, 1000);//等待电调上电 
  
  while(1)
  {
   currentTime = xTaskGetTickCount();	//获取当前系统时间
    /*code begin*/   
   chassis_state_update();
   
//   ALL_Key_Process();
   
   Remote_TASK_ALL(DJI_RC.rc.switch_L,DJI_RC.rc.switch_R);
   
   Chassis_Mode_Change(Robot1.Status.actChassis);
   
   
   //Chassis_Power_Limit();//功率限制
   
   CHASSIS_Single_Loop_Out();//输出


  /*code end*/
  vTaskDelayUntil(&currentTime, 2);//绝对延时 2ms
  }
}



/**
  * @brief  切换底盘控制模式
  * @param  模式切换标志位
  * @retval none
  * @attention
  */
void Chassis_Mode_Change(eChassisAction mode)
{
    switch(mode) 
    {
    case CHASSIS_LOCK: //锁定
//          Robot1.Movement.Chassis_speed_out.vx = 0;  //X
//          Robot1.Movement.Chassis_speed_out.vy = 0;  //Y 
//          Robot1.Movement.Chassis_speed_out.vw = 0;  //WZ 
//          Absolute_Cal(&Robot1.Movement.Balance_Chassis_speed_out,0);//计算各个电机的目标速度
          chassis_nurmal();
          break;      
      
    case CHASSIS_SEPARATE://独立
//          Absolute_Cal(&Robot1.Movement.Chassis_speed_out,((yaw_motor.data.angle - Robot1.Mechanical.YAW_Motor_Middle)*0.043945f));//计算各个电机的目标速度    
          break;
          
    case CHASSIS_FOLLOW_GIMBAL : //跟随
          chassis_nurmal();
//          Robot1.Movement.Chassis_speed_out.vw  = Chassis_Follow(&chassis_follow);    //随动pid         
//         Absolute_Cal(&Robot1.Movement.Balance_Chassis_speed_out,0);//计算各个电机的目标速度    
          break;    
          
    case CHASSIS_GYROSCOPE : //小陀螺
//          Robot1.Movement.Chassis_speed_out.vw = Robot1.Movement.gyroscope_speed;//小陀螺速度
//          Absolute_Cal(&Robot1.Movement.Chassis_speed_out,((yaw_motor.data.angle - Robot1.Mechanical.YAW_Motor_Middle)*0.043945f));//计算各个电机的目标速度    
          break;   
          
    default:
        break;          
    }
}



/**
  * @brief  将云台坐标转换为底盘坐标
  * @param  absolute_speed 绝对坐标需要的速度 
  * @param  angle 云台相对于底盘的角度
  * @retval 偏差角，角度制
  * @attention
  */
void Absolute_Cal(Chassis_Speed* absolute_speed, fp32 angle)
{
//    fp32 angle_hd=angle* PI / 180;
//    Chassis_Speed temp_speed;
//    temp_speed.vw = absolute_speed->vw; 
//    temp_speed.vx = absolute_speed->vx * cos(angle_hd) - absolute_speed->vy * sin(angle_hd);
//    temp_speed.vy = absolute_speed->vx * sin(angle_hd) + absolute_speed->vy * cos(angle_hd);
//    
//    mecanum_calc(&temp_speed, Robot1.Movement.chassis_3508_setspeed);
}

/**
  * @brief  麦克纳姆轮底盘解算
  * @param  三个自由度设定值 （绝对坐标）
  * @retval 四个轮子的转速
  * @attention
  */
void mecanum_calc(Chassis_Speed *speed, int16_t *out_speed)
{
    float wheel_rpm[4];
    float wheel_rpm_ratio;

    wheel_rpm_ratio = 60.0f / (Robot1.Mechanical.WHEEL_PERIMETER * PI) * Robot1.Mechanical.CHASSIS_DECELE_RATIO; 

    wheel_rpm[0] = (+speed->vx + speed->vy + speed->vw * (Robot1.Mechanical.LENGTH_A + Robot1.Mechanical.LENGTH_B)) * wheel_rpm_ratio;
    wheel_rpm[1] = (+speed->vx - speed->vy + speed->vw * (Robot1.Mechanical.LENGTH_A + Robot1.Mechanical.LENGTH_B)) * wheel_rpm_ratio;
    wheel_rpm[2] = (-speed->vx + speed->vy + speed->vw * (Robot1.Mechanical.LENGTH_A + Robot1.Mechanical.LENGTH_B)) * wheel_rpm_ratio;
    wheel_rpm[3] = (-speed->vx - speed->vy + speed->vw * (Robot1.Mechanical.LENGTH_A + Robot1.Mechanical.LENGTH_B)) * wheel_rpm_ratio;

    for(int i=0;i<4;i++)//memcpy有问题
    {
      Limit(wheel_rpm[i],Robot1.Movement.chassis_3508_maxspeed[i],-Robot1.Movement.chassis_3508_maxspeed[i]);
      Robot1.Movement.chassis_3508_setspeed[i] = (int16_t)wheel_rpm[i];
    }
}

/**
  * @brief  底盘电机输出
  * @param  void
  * @retval void
  * @attention
  */
void CHASSIS_Single_Loop_Out()
{
//  chassis_balance();
//  chassis_nurmal();
   Assign_To_Motor_speed(&balance_L, Robot1.Movement.chassis_3508_setspeed[balance_L_num]);
   Assign_To_Motor_speed(&balance_R, Robot1.Movement.chassis_3508_setspeed[balance_R_num]);
}



/**
  * @brief  底盘电机输出功率算法
  * @param  void
  * @retval void
  * @attention 在底盘输出计算后调用，主要是比例的算法，ICRA
  */
void Chassis_Power_Limit(void)
{
  float    kLimit = 0;                  //功率限制系数
  float    chassis_totaloutput = 0;     //统计底盘总输出电流
  float    gimbal_totaloutput = 0;     //统计云台总输出电流
  float    Joule_Residue = 0;           //剩余焦耳缓冲能量
  float    fChasCurrentLimit = CHAS_CURRENT_LIMIT;    //限制四个轮子的速度总和
  static   int32_t judgDataError_Time = 0;
  
  fCurrentLimit.judgDataCorrect = JUDGE_sGetDataState();      //裁判系统数据是否可用
  Joule_Residue   = JUDGE_fGetRemainEnergy();   //剩余焦耳能量
  
  chassis_totaloutput = zn_abs(Robot1.Movement.chassis_3508_setspeed[FL_num]) + zn_abs(Robot1.Movement.chassis_3508_setspeed[FR_num])
                      + zn_abs(Robot1.Movement.chassis_3508_setspeed[BL_num]) + zn_abs(Robot1.Movement.chassis_3508_setspeed[BR_num]);
                      
  gimbal_totaloutput  = zn_abs(yaw_motor.speed_pid.All_out)  + zn_abs(pitch_motor.speed_pid.All_out);
    
  if(fCurrentLimit.judgDataCorrect == JUDGE_DATA_ERROR)       //裁判系统无效时强制限速
    {
      judgDataError_Time++;
      if(judgDataError_Time > 100)
      {
        fCurrentLimit.fTotalCurrentLimit = 6000;//大的1/4
      }
    }
  else
    {
        judgDataError_Time = 0;
	//剩余焦耳量过小，开始限制输出，限制系数为平方关系
	if(Joule_Residue < WARNING_REMAIN_POWER)
	{
          kLimit = (float)(Joule_Residue / WARNING_REMAIN_POWER)
		 * (float)(Joule_Residue / WARNING_REMAIN_POWER);
			
          fCurrentLimit.fTotalCurrentLimit = kLimit * (fChasCurrentLimit - gimbal_totaloutput);
	}
        //焦耳能量恢复到一定数值
	else   
	{
          fCurrentLimit.fTotalCurrentLimit = fChasCurrentLimit;
	}
        
    }
    //底盘各电机电流重新分配
    if (chassis_totaloutput > fCurrentLimit.fTotalCurrentLimit )
    {
	Robot1.Movement.chassis_3508_setspeed[FL_num] = (int16_t)(Robot1.Movement.chassis_3508_setspeed[FL_num] / chassis_totaloutput * fCurrentLimit.fTotalCurrentLimit);
	Robot1.Movement.chassis_3508_setspeed[FR_num] = (int16_t)(Robot1.Movement.chassis_3508_setspeed[FR_num] / chassis_totaloutput * fCurrentLimit.fTotalCurrentLimit);
	Robot1.Movement.chassis_3508_setspeed[BL_num] = (int16_t)(Robot1.Movement.chassis_3508_setspeed[BL_num] / chassis_totaloutput * fCurrentLimit.fTotalCurrentLimit);
	Robot1.Movement.chassis_3508_setspeed[BR_num] = (int16_t)(Robot1.Movement.chassis_3508_setspeed[BR_num] / chassis_totaloutput * fCurrentLimit.fTotalCurrentLimit);	
    }
  
}

/**
  * @brief  底盘平衡模式
  * @param  none
  * @retval none
  */
void chassis_balance()
{
  chassis_contrl.target_pose_x=chassis_contrl.pose_x;
  chassis_contrl.pose_x=0;
  chassis_contrl.target_speed_x=chassis_contrl.vx_set_out;
  chassis_contrl.target_turn_w=chassis_contrl.wz_set_out;
  
  /* 平衡pid */
  PID_calc(&LQR_k1,chassis_contrl.angle_x,0);
  PID_calc(&LQR_k2,chassis_contrl.gyro_x,0);
  
  /* 前进转向pid */
  PID_calc(&LQR_k4,chassis_contrl.speed_x,0);
  PID_calc(&LQR_k5,chassis_contrl.pose_x,0);
  PID_calc(&LQR_k6,chassis_contrl.turn_w,0);
  
  chassis_contrl.torque_balance=(LQR_k1.out+LQR_k2.out)*chassis_contrl.torque_const;
  chassis_contrl.torque_speed=(LQR_k4.out+LQR_k5.out)*chassis_contrl.torque_const;
  
  PID_calc(&Offset_PID,0,chassis_contrl.torque_balance+chassis_contrl.torque_speed);
  
  Robot1.Movement.chassis_3508_setspeed[balance_L_num]=-Offset_PID.out+LQR_k6.out*chassis_contrl.torque_const;
  Robot1.Movement.chassis_3508_setspeed[balance_R_num]=Offset_PID.out+LQR_k6.out*chassis_contrl.torque_const;
  
  stop_balance();
}

/**
  * @brief  底盘正常模式
  * @param  none
  * @retval none
  */
void chassis_nurmal()
{
  chassis_contrl.target_pose_x=chassis_contrl.pose_x;
  chassis_contrl.pose_x=0;
  chassis_contrl.target_speed_x=chassis_contrl.vx_set_out;
  chassis_contrl.target_turn_w=chassis_contrl.wz_set_out;
  
  /* 平衡pid */
  PID_calc(&LQR_k1,chassis_contrl.angle_x,0);
  PID_calc(&LQR_k2,chassis_contrl.gyro_x,0);
  
  /* 前进转向pid */
  PID_calc(&LQR_k4,chassis_contrl.speed_x,chassis_contrl.target_speed_x);
  PID_calc(&LQR_k5,chassis_contrl.pose_x,chassis_contrl.target_pose_x);
  PID_calc(&LQR_k6,chassis_contrl.turn_w,chassis_contrl.target_turn_w);
  
  chassis_contrl.torque_balance=(LQR_k1.out+LQR_k2.out)*chassis_contrl.torque_const;
  chassis_contrl.torque_speed=(LQR_k4.out+LQR_k5.out)*chassis_contrl.torque_const;
  
  PID_calc(&Offset_PID,0,chassis_contrl.torque_balance+chassis_contrl.torque_speed);
  
  Robot1.Movement.chassis_3508_setspeed[balance_L_num]=-Offset_PID.out+LQR_k6.out*chassis_contrl.torque_const;
  Robot1.Movement.chassis_3508_setspeed[balance_R_num]=Offset_PID.out+LQR_k6.out*chassis_contrl.torque_const;
  
  stop_balance();
}

float stop_flag;
void stop_balance()
{
  if(INS_angle[1]>=60||INS_angle[1]<=-60)stop_flag=1;
  if(stop_flag==1)
  {
     Robot1.Movement.chassis_3508_setspeed[balance_L_num]=0;
     Robot1.Movement.chassis_3508_setspeed[balance_R_num]=0;
  }
}

/**
  * @brief  底盘状态更新
  * @param  none
  * @retval none
  */
void chassis_state_update()
{
  
  chassis_contrl.angle_x=INS_angle[1]+chassis_contrl.Balance_angle_offset;
  chassis_contrl.gyro_x=INS_gyro[1];
  chassis_contrl.turn_w=INS_gyro[2];
  chassis_contrl.speed_x=(balance_L.data.speed_rpm-balance_R.data.speed_rpm) /57.29578f*chassis_contrl.Diameter_weel/2.0f/2.0f + chassis_contrl.gyro_x * 0.03;
  chassis_contrl.pose_x+=chassis_contrl.speed_x*0.01f;

  chassis_contrl.move_speed=sqrtf(chassis_contrl.vx_set*chassis_contrl.vx_set+chassis_contrl.vy_set*chassis_contrl.vy_set);
  chassis_contrl.move_direction = atan2f(chassis_contrl.vy_set,chassis_contrl.vx_set);
  
  
}

/**
  * @brief  速度控制
  * @param  输入速度,运动方向
  * @retval 输出速度
  */
float speed_control(float speed_in,float direction)
{
	fp32 accel_K1;
	fp32 accel_K2;
	if(fabs(direction)>1.39&&fabs(direction)<1.75)
	{
		accel_K1 = ACCEL_1_Y;
		accel_K2 = ACCEL_2_Y;
	}
	else 
	{
		accel_K1 = ACCEL_1_X;
		accel_K2 = ACCEL_2_X;
	}
	if (speed_in >= 0)
	{
		fp32 accel_1 = accel_K1*(speed_in - chassis_contrl.speed_x);
		fp32 accel_2 = accel_K2*(speed_in - chassis_contrl.speed_x);
		if (chassis_contrl.speed_x >= 0)
		{
			return chassis_contrl.speed_x + accel_1;
		}
		else if (chassis_contrl.speed_x < 0)
			return chassis_contrl.speed_x + accel_2;
	}
	if (speed_in < 0)
	{
		fp32 accel_1 = accel_K1*(speed_in - chassis_contrl.speed_x);
		fp32 accel_2 = accel_K2*(speed_in - chassis_contrl.speed_x);
		if (chassis_contrl.speed_x < 0)
		{
			return chassis_contrl.speed_x - accel_1;
		}
		else if (chassis_contrl.speed_x >= 0)
			return chassis_contrl.speed_x - accel_2;
	}
}



