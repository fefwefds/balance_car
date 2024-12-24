#include "zn_motor.h"
#include "CAN_receive.h"

Motor_type_def yaw_motor;//云台
Motor_type_def pitch_motor;

Motor_type_def  drive_FL;//左前
Motor_type_def  drive_FR;//右前
Motor_type_def  drive_BL;//左后
Motor_type_def  drive_BR;//右后

Motor_type_def rotate_motor;//拨弹
Motor_type_def rub_motor_L;//摩擦轮
Motor_type_def rub_motor_R;
Motor_type_def rub_motor_2006;

Motor_type_def balance_L;//左轮
Motor_type_def balance_R;//右轮
Motor_type_def slideblock_R;//右滑块
Motor_type_def slideblock_L;//左滑块

bal_pid_type_def LQR_k1;
bal_pid_type_def LQR_k2;
bal_pid_type_def LQR_k3;
bal_pid_type_def LQR_k4;
bal_pid_type_def LQR_k5;
bal_pid_type_def LQR_k6;
bal_pid_type_def Offset_PID;
bal_pid_type_def SlideBlock_PID[2];

float chassis_SlideBlock_PID[3]={PID_SlideBlock_Speed_KP, PID_SlideBlock_Speed_KI, PID_SlideBlock_Speed_KD};
float pid_LQR_k1[3] = {PID_LQR_k1_KP, PID_LQR_k1_KI, PID_LQR_k1_KD};
float pid_LQR_k2[3] = {PID_LQR_k2_KP, PID_LQR_k2_KI, PID_LQR_k2_KD};
float pid_LQR_k3[3] = {PID_LQR_k3_KP, PID_LQR_k3_KI, PID_LQR_k3_KD};
float pid_LQR_k4[3] = {PID_LQR_k4_KP, PID_LQR_k4_KI, PID_LQR_k4_KD};
float pid_LQR_k5[3] = {PID_LQR_k5_KP, PID_LQR_k5_KI, PID_LQR_k5_KD};
float pid_LQR_k6[3] = {PID_LQR_k6_KP, PID_LQR_k6_KI, PID_LQR_k6_KD};
float pid_Offset[3] = {PID_Offset_KP, PID_Offset_KI, PID_Offset_KD};

//电机所有参数初始化
void Motor_All_Para_Init()
{ 
  //左前
  balance_L.mes.CAN = ZN_CAN1;
  balance_L.mes.KIND = M3508;
  balance_L.mes.ID = 1;
  balance_L.speed_pid.kp = 1;//2
  balance_L.speed_pid.ki = 0.05;//0.15
  balance_L.angle_pid.kp = 0;//0.8
  
  //右前  
  balance_R.mes.CAN = ZN_CAN1;
  balance_R.mes.KIND = M3508;
  balance_R.mes.ID = 2;
  balance_R.speed_pid.kp = 1;//2
  balance_R.speed_pid.ki = 0.05;//0.15
  balance_R.angle_pid.kp = 0;//0.8
  
  //右动量块
  slideblock_R.mes.CAN = ZN_CAN1;
  slideblock_R.mes.KIND = M6020;
  slideblock_R.mes.ID = 2;
//  slideblock_R.speed_pid.kp = 5;//350  100
//  slideblock_R.speed_pid.ki = 0.4;  //0.1  0.001
//  slideblock_R.speed_pid.kd = 4; //30  30
  slideblock_R.angle_pid.kp = 4;//0.1  0.05
  slideblock_R.angle_pid.ki=0.1;
  
  //左动量块
  slideblock_L.mes.CAN = ZN_CAN1;
  slideblock_L.mes.KIND = M6020;
  slideblock_L.mes.ID = 1;
//  slideblock_L.speed_pid.kp = 5;//350  100
//  slideblock_L.speed_pid.ki = 0.4;  //0.1  0.001
//  slideblock_L.speed_pid.kd = 4; //30  30
  slideblock_L.angle_pid.kp = 4;//0.1  0.05
  slideblock_L.angle_pid.ki=0.1;
  
  
  Motor_message_Init(&balance_L);
  Motor_message_Init(&balance_R);
  Motor_message_Init(&slideblock_R);
  Motor_message_Init(&slideblock_L);
// //yaw轴
//  yaw_motor.mes.CAN = ZN_CAN1;
//  yaw_motor.mes.KIND = M6020;
//  yaw_motor.mes.ID = 1;
//  yaw_motor.speed_pid.kp = 100;//350  100
//  yaw_motor.speed_pid.ki = 0.1;  //0.1  0.001
//  yaw_motor.speed_pid.kd = 30; //30  30
//  yaw_motor.angle_pid.kp = 0.55;//0.1  0.05
//  
////  pid_paragram_init(&yaw_motor.speed_pid,350,0.1,30);
////  pid_paragram_init(&yaw_motor.angle_pid,0.1,0,0);
//  
//  //pitch轴
//  pitch_motor.mes.CAN = ZN_CAN2;
//  pitch_motor.mes.KIND = M6020;
//  pitch_motor.mes.ID = 2;
////  pitch_motor.speed_pid.kp = 350;
////  pitch_motor.speed_pid.ki = 0.1;  
////  pitch_motor.speed_pid.kd = 30; 
////  pitch_motor.angle_pid.kp = 0.1;
////  pitch_motor.angle_pid.ki = 0;
//  
//  pitch_motor.speed_pid.kp = 100;//100
//  pitch_motor.speed_pid.ki = 0.05;  //0.04
//  pitch_motor.speed_pid.kd = 0; 
//  pitch_motor.angle_pid.kp = 1.8;//1
//  pitch_motor.angle_pid.ki = 0.001;//0.001
//  
//  pitch_motor.speed_pid.IntegralLimit = 40000;
//  
//  
// 
//  //左前
//  drive_FL.mes.CAN = ZN_CAN1;
//  drive_FL.mes.KIND = M3508;
//  drive_FL.mes.ID = 1;
//  drive_FL.speed_pid.kp = 2;
//  drive_FL.speed_pid.ki = 0.15;
//  drive_FL.angle_pid.kp = 0.8;
//  drive_FL.angle_pid.ki = 0.001;    
//  //右前  
//  drive_FR.mes.CAN = ZN_CAN1;
//  drive_FR.mes.KIND = M3508;
//  drive_FR.mes.ID = 2;
//  drive_FR.speed_pid.kp = 2;
//  drive_FR.speed_pid.ki = 0.15;
//  drive_FR.angle_pid.kp = 0.8;
//  drive_FR.angle_pid.ki = 0.001; 
//  //左后 
//  drive_BL.mes.CAN = ZN_CAN1;
//  drive_BL.mes.KIND = M3508;
//  drive_BL.mes.ID = 4;
//  drive_BL.speed_pid.kp = 2;
//  drive_BL.speed_pid.ki = 0.15;
//  drive_BL.angle_pid.kp = 0.8;
//  drive_BL.angle_pid.ki = 0.001; 
//  //右后 
//  drive_BR.mes.CAN = ZN_CAN1;
//  drive_BR.mes.KIND = M3508;
//  drive_BR.mes.ID = 3;
//  drive_BR.speed_pid.kp = 2;
//  drive_BR.speed_pid.ki = 0.15;
//  drive_BR.angle_pid.kp = 0.8;
//  drive_BR.angle_pid.ki = 0.001;

//  //拨弹
//  rotate_motor.mes.CAN = ZN_CAN1;
//  rotate_motor.mes.KIND = M3508;
//  rotate_motor.mes.ID = 6;
//  rotate_motor.speed_pid.kp = 10;
//  rotate_motor.speed_pid.ki = 0;
//  rotate_motor.angle_pid.kp = 1.5;
//  rotate_motor.angle_pid.ki = 0;    
//  
//  //摩擦轮
//  rub_motor_L.mes.CAN = ZN_CAN2;
//  rub_motor_L.mes.KIND = M3508;
//  rub_motor_L.mes.ID = 2;
//  rub_motor_L.speed_pid.kp = 2;
//  rub_motor_L.speed_pid.ki = 0.1;
//  rub_motor_L.angle_pid.kp = 0.8;
//  rub_motor_L.angle_pid.ki = 0.001; 
//  
//  rub_motor_R.mes.CAN = ZN_CAN2;
//  rub_motor_R.mes.KIND = M3508;
//  rub_motor_R.mes.ID = 3;
//  rub_motor_R.speed_pid.kp = 2;
//  rub_motor_R.speed_pid.ki = 0.1;
//  rub_motor_R.angle_pid.kp = 0.8;
//  rub_motor_R.angle_pid.ki = 0.001;    
//   
//  rub_motor_2006.mes.CAN=ZN_CAN2;
//  rub_motor_2006.mes.KIND=M3508;
//  rub_motor_2006.mes.ID=1;
//  rub_motor_2006.speed_pid.kp=2;
//  rub_motor_2006.speed_pid.ki=0.1;
//  rub_motor_2006.angle_pid.kp = 0.8;
//  rub_motor_2006.angle_pid.ki = 0.001;
    
    
//  Motor_message_Init(&yaw_motor);
//  Motor_message_Init(&pitch_motor);
  
//  Motor_message_Init(&drive_FL);
//  Motor_message_Init(&drive_FR);
//  Motor_message_Init(&drive_BL);
//  Motor_message_Init(&drive_BR);  
 
//  Motor_message_Init(&rotate_motor);
//  Motor_message_Init(&rub_motor_L);
//  Motor_message_Init(&rub_motor_R);  
//  Motor_message_Init(&rub_motor_2006);

}

//电机速度设定 3508 or 6020 or 2006
void Assign_To_Motor_speed(Motor_type_def *ptr,int16_t speed)
{
//  Motor_speed_cal(ptr,speed);//速度环计算
  CAN_Set_Current(ptr,speed);//电流输出
}

//6020角度设定
void Assign_To_M6020_angle(Motor_type_def *ptr,int16_t angle)
{
  M6020_angle_cal(ptr,angle);//角度环计算 
//  Motor_speed_cal(ptr,(int16_t)ptr->angle_pid.All_out);//速度环计算
  CAN_Set_Current(ptr,(int16_t)ptr->angle_pid.All_out);//电流输出
}

//3508&2006角度设定
void Assign_To_M3508_angle(Motor_type_def *ptr,int16_t angle)
{
  M3508_angle_cal(ptr,angle);//角度环计算 
  Motor_speed_cal(ptr,(int16_t)ptr->angle_pid.All_out);//速度环计算
  CAN_Set_Current(ptr,(int16_t)ptr->speed_pid.All_out);//电流输出
}

//电机信息初始化
void Motor_message_Init(Motor_type_def *ptr)
{
    if(ptr->mes.KIND==M2006)
    {
      if(ptr->mes.ID<=4)
      {
        ptr->mes.control_StdId=0x200;
      }
      else if(ptr->mes.ID>4)
      {
        ptr->mes.control_StdId=0x1FF;
      }
      ptr->mes.back_StdId=0x200+ptr->mes.ID;
    }
    if(ptr->mes.KIND == M3508)
    {
      if(ptr->mes.ID<=4)
      {
        ptr->mes.control_StdId = 0X200;
      }   
     else if(ptr->mes.ID>4)
      {
        ptr->mes.control_StdId = 0X1FF;
      }
     ptr->mes.back_StdId = 0X200 + ptr->mes.ID;
    }
    
    if(ptr->mes.KIND == M6020)
    {
      if(ptr->mes.ID<=4)
      {
      ptr->mes.control_StdId = 0X1FF;
      }  
      if(ptr->mes.ID>4)
      {
      ptr->mes.control_StdId = 0X2FF;
      }
      ptr->mes.back_StdId = 0X204 + ptr->mes.ID; 
    }
}


//各种电机速度环计算方式相同，仅修改参数即可
float Motor_speed_cal(Motor_type_def *ptr,int16_t speed)
{
    ptr->speed_pid.error = speed - ptr->data.speed_rpm;
    ptr->speed_pid.error_all += ptr->speed_pid.error;
    
    //积分限幅
    if(ptr->mes.KIND == M6020)
    {
    Limit(ptr->speed_pid.error_all,ptr->speed_pid.IntegralLimit,-ptr->speed_pid.IntegralLimit);
    }    
    
    ptr->speed_pid.P_out = ptr->speed_pid.kp * ptr->speed_pid.error;//P_OUT
    ptr->speed_pid.I_out = ptr->speed_pid.ki * ptr->speed_pid.error_all;//I_OUT
    ptr->speed_pid.D_out = ptr->speed_pid.kd * (ptr->speed_pid.error - ptr->speed_pid.error_last);//D_OUT
   
    ptr->speed_pid.error_last = ptr->speed_pid.error;//数据更新
    
    ptr->speed_pid.All_out = ptr->speed_pid.P_out + ptr->speed_pid.I_out + ptr->speed_pid.D_out;
    
    //输出限幅
    
    if(ptr->mes.KIND == M6020)
    {
      Limit(ptr->speed_pid.All_out,25000, -25000);//25000
    }
    else if(ptr->mes.KIND == M3508)
    {
      Limit(ptr->speed_pid.All_out,7000, -7000);//16384
    }
      
    //死区限制
    LimitDeadBand(ptr->speed_pid.All_out,200);
//    LimitDeadBand(ptr->speed_pid.All_out,ptr->speed_pid.DeadBand);

    return ptr->speed_pid.All_out;
}


float M6020_angle_cal(Motor_type_def *ptr,int16_t angle)
{
  
    while(angle>8192)angle = angle - 8192;//圆周处理
  
    ptr->angle_pid.error = angle - ptr->data.angle;//偏差
     
    //过零处理,统一成劣弧
    if(ptr->angle_pid.error>( 8192/2))ptr->angle_pid.error = ptr->angle_pid.error - 8192;
    if(ptr->angle_pid.error<(-8192/2))ptr->angle_pid.error = ptr->angle_pid.error + 8192;    
    
    ptr->angle_pid.error_all += ptr->angle_pid.error;
    
    //积分限幅
  //  Limit(ptr->angle_pid.error_all,ptr->angle_pid.IntegralLimit,-ptr->angle_pid.IntegralLimit);

    ptr->angle_pid.P_out = ptr->angle_pid.kp * ptr->angle_pid.error;//P_OUT
    ptr->angle_pid.I_out = ptr->angle_pid.ki * ptr->angle_pid.error_all;//I_OUT
    ptr->angle_pid.D_out = ptr->angle_pid.kd * (ptr->angle_pid.error - ptr->angle_pid.error_last);//D_OUT
   
    ptr->angle_pid.error_last = ptr->angle_pid.error;//数据更新
    
    ptr->angle_pid.All_out = ptr->angle_pid.P_out + ptr->angle_pid.I_out + ptr->angle_pid.D_out;
    
    //输出限幅
  //  Limit(ptr->angle_pid.All_out,ptr->angle_pid.MaxOutput, -ptr->angle_pid.MaxOutput);
    //死区限制
    LimitDeadBand(ptr->angle_pid.All_out,ptr->angle_pid.DeadBand);

    return ptr->angle_pid.All_out;
}


//3508电机角度环计算  total_angle
float M3508_angle_cal(Motor_type_def *ptr,int16_t angle)
{ 
    ptr->angle_pid.error = angle - ptr->data.total_angle;//偏差
    ptr->angle_pid.error_all += ptr->angle_pid.error;//偏差积分
    
    //积分限幅
  //  Limit(ptr->angle_pid.error_all,ptr->angle_pid.IntegralLimit,-ptr->angle_pid.IntegralLimit);

    ptr->angle_pid.P_out = ptr->angle_pid.kp * ptr->angle_pid.error;//P_OUT
    ptr->angle_pid.I_out = ptr->angle_pid.ki * ptr->angle_pid.error_all;//I_OUT
    ptr->angle_pid.D_out = ptr->angle_pid.kd * (ptr->angle_pid.error - ptr->angle_pid.error_last);//D_OUT
   
    ptr->angle_pid.error_last = ptr->angle_pid.error;//数据更新
    
    ptr->angle_pid.All_out = ptr->angle_pid.P_out + ptr->angle_pid.I_out + ptr->angle_pid.D_out;
    
    //输出限幅
  //  Limit(ptr->angle_pid.All_out,ptr->angle_pid.MaxOutput, -ptr->angle_pid.MaxOutput);
    //死区限制
    LimitDeadBand(ptr->angle_pid.All_out,ptr->angle_pid.DeadBand);

    return ptr->angle_pid.All_out;

}

//获得3508电机total_angle
float err,err_err;
void total_angle_get(Motor_type_def *ptr)
{ 
  float res1=0,res2=0;
  
  static float pos,pos_old;
  
  pos = (rotate_motor.data.angle/8092.0f*360.0f);
  
  err = pos - pos_old ;
   
  if(err>0)
  {
    res1 = err - 360;
    res2 = err;
  }
  else
  {
    res1 = err + 360;
    res2 = err;  
  }
  if(zn_abs(res1)<zn_abs(res2)) //不管正反转，肯定是转的角度小的那个是真的
  {
    err_err = res1;
  }
  else 
  {
    err_err = res2;
  }
   pos_old = pos;
   rotate_motor.data.total_angle += err_err;  
}

/* 直立环PD计算 */
void Vertical_Ring_PD(Motor_type_def *ptr,float angle,float gyro)
{
  
  
  ptr->angle_pid.error=angle-ptr->angle_pid.Mechanical_balance;
  ptr->gyro_pid.error=gyro*100-ptr->gyro_pid.Mechanical_balance;
  
  /* 迭代 */
  ptr->gyro_pid.error_all=ptr->gyro_pid.error-ptr->gyro_pid.error_last;
  ptr->gyro_pid.error_last=ptr->gyro_pid.error;
  
  ptr->bal_vertical_pid.P_out=ptr->bal_vertical_pid.kp*ptr->angle_pid.error;
  ptr->bal_vertical_pid.D_out=ptr->bal_vertical_pid.kd*ptr->gyro_pid.error_all;
  
  ptr->bal_vertical_pid.All_out=ptr->bal_vertical_pid.P_out+ptr->bal_vertical_pid.D_out;
}

/* 速度环PI计算 */
void Speed_Ring_PI(Motor_type_def *ptr,int16_t speed_set,int16_t speed_l,int16_t speed_r)
{
  ptr->bal_speed_pid.error=0-(speed_l+speed_r);
  ptr->bal_speed_pid.error_last*=0.8;
  ptr->bal_speed_pid.error_last=ptr->bal_speed_pid.error*0.2;
  ptr->bal_speed_pid.error_all+=ptr->bal_speed_pid.error_last;
  
  ptr->bal_speed_pid.error_all=ptr->bal_speed_pid.error_all+speed_set;
  if(ptr->bal_speed_pid.error_all>10000)ptr->bal_speed_pid.error_all=10000;
  if(ptr->bal_speed_pid.error_all<-10000)ptr->bal_speed_pid.error_all=-10000;
  
  ptr->bal_speed_pid.P_out=ptr->bal_speed_pid.kp*ptr->bal_speed_pid.error_last;
  ptr->bal_speed_pid.I_out=ptr->bal_speed_pid.ki*ptr->bal_speed_pid.error_all;
}

/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
fp32 PID_calc(bal_pid_type_def *pid,fp32 ref,fp32 set)
{
  if(pid==NULL){return 0.0f;}
  
  pid->error[2] = pid->error[1];
  pid->error[1] = pid->error[0];
  pid->set = set;
  pid->fdb = ref;
  pid->error[0] = set - ref;
  
  if (pid->mode == PID_POSITION)
  {
    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    Limit(pid->Iout, pid->max_iout,-pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    Limit(pid->out, pid->max_out,-pid->max_out);
    }
  else if (pid->mode == PID_DELTA)
  {
    pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
    pid->Iout = pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    pid->out += pid->Pout + pid->Iout + pid->Dout;
    Limit(pid->out, pid->max_out,-pid->max_out);
  }
  
  return pid->out;
}

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
void PID_init(bal_pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}