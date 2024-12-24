#ifndef ZN_INTERACTION_H
#define ZN_INTERACTION_H

#include "zn_chassis_task.h"
#include "zn_gimbal_task.h"//(玄学包含关系)

#define RC_Middle_date 60

typedef enum
{
    EKF ,   //移植的哈工程陀螺仪解算
    AHRS ,   //官方的AHRS解算

} Gyroscope_solving;

typedef struct
{
  Gyroscope_solving gyroscope_solving;   //陀螺仪解算选择
  
} Feature_selection;

typedef struct
{
  float LENGTH_A;                   //前后轴距
  float LENGTH_B;                   //左右轮距
  float WHEEL_PERIMETER;             //麦轮轮径
  float  CHASSIS_DECELE_RATIO;       //电机减数比
  //无偏移
  
  int16_t SlideBlock_R_Middle;
  int16_t SlideBlock_L_Middle;
   
  int16_t YAW_Motor_Middle;//yaw电机中位角度值

  int16_t PITCH_Motor_Middle;//PITCH电机中位角度值  
  int16_t PITCH_Motor_Front;//PITCH电机最大俯角
  int16_t PITCH_Motor_Behind;//PITCH电机最大仰角
  
} Mechanical_Para;//机械参数

typedef struct 
{
  fp32 torque_balance;
  fp32 torque_speed;
  fp32 toque_omega;
  fp32 torque_const;
}BAL_CTEL;

typedef struct
{  
  BAL_CTEL bal_classis_ctrl;
  //底盘
  Chassis_Speed Chassis_speed_set;               //底盘设定速度
  Chassis_Speed Chassis_speed_max;               //底盘速度限幅
  Chassis_Speed Chassis_speed_out;               //斜坡输出
  
//  Chassis_Speed Balance_Chassis_speed_set;               //底盘设定速度
//  Chassis_Speed Balance_Chassis_speed_max;               //底盘速度限幅
//  Chassis_Speed Balance_Chassis_speed_out;               //斜坡输出
//  
  
  double ramp_vx;//斜坡输出系数
  double ramp_vy;
  double ramp_vw;
  
  float gyroscope_speed;//小陀螺速度
  
  int16_t bal_chassis_2006_setspeed[2];
  int16_t bal_chassis_2006_maxspeed[2];
  
  int16_t chassis_slideblock_setangle[2];
  int16_t chassis_slideblock_maxangle[2];
  
  int16_t chassis_3508_setspeed[4];		//四个3508轮子目标转速
  int16_t chassis_3508_maxspeed[4];               //电机转速限幅
  //云台
  float Gimbal_YAW_motor_angle_set;             //云台yaw电机角度设定值
  float Gimbal_YAW_imu_angle_set;               //云台yaw陀螺仪角度设定值  
  float Gimbal_PITCH_motor_angle_set;           //云台pitch电机角度设定值
  float Gimbal_PITCH_imu_angle_set;             //云台pitch陀螺仪角度设定值
  //发射
  int16_t pluck_speed;                        //拨弹盘转速 (连发时)
  int16_t pluck_angle;                       //拨弹盘设定角度（单发时）
  int16_t rub_speed;                          //摩擦轮转速
  int16_t rub_speed_2006;
   
} Movement_Data;//运动数据

//云台控制模式
typedef enum
{
  rc_contorl,//遥控器控制
  mouse_contorl,//鼠标控制
}GimbalContorl_MODE;

typedef struct
{
  eChassisAction actChassis;//底盘状态
  eGimbalAction actGimbal;//云台状态
  GimbalContorl_MODE contorl_MODE;//云台控制模式

  int16_t lid_flag;//弹舱盖状态
  int16_t rub_flag;//摩擦轮状态
  int16_t pluck_flag;//拨弹状态
  int16_t collimation_flag;//自瞄状态
  int16_t super_power_flag;//超级电容状态
  
} Status_Flag;

typedef struct
{
  Mechanical_Para Mechanical;//机械参数
  Movement_Data Movement;//运动数据
  Status_Flag Status;//状态标志
  Feature_selection feature_selection;  //功能模式选择
} ROBOT;

typedef struct
{
  float Diameter_weel;//轮半径
  
  float angle_x;//当前角度
  float Balance_angle_offset;//角度偏差值
  float gyro_x,gyro_w;//当前角速度
  float ramp_vx,ramp_vy,ramp_wz;//xyz轴的斜坡系数
  float target_turn_w,turn_w;
  float target_speed_x,speed_x,speed_y;//x和y的速度
  float target_pose_x,pose_x,pose_y;//底盘x和y的位移
  float move_speed,move_direction;
  float vx_set,vy_set,wz_set,vx_last_set;//遥控器的输出值
  float vx_set_out,vy_set_out,wz_set_out;//遥控器处理最终数值（斜坡函数）
  float bal_speed_set;//最终输出速度
  float torque_balance;//平衡动力相应转矩
  float torque_speed;//前进动力相应转矩
  float toque_omega;//旋转动力相应转矩
  float torque_const;//扭矩常数
  float slideblock_r_nowangle,slideblock_l_nowangle,slideblock_r_setangle,slideblock_l_setangle;//滑块
  float slideblock_kp;//比例系数
  
  uint8_t flag_clear_pose;//里程计停止清零标志(0,清零;1,停止清零)
}chassis_contrl_date;


extern ROBOT Robot1;
extern chassis_contrl_date chassis_contrl;

typedef struct
{
  int16_t once;
  int16_t count;
  int16_t flag; 
} KeyBoard;

typedef enum
{
  ABCDEFG,//占位用
  SWITCH_UP,//上
  SWITCH_DOWN,//下
  SWITCH_MIDDLE//中
}SWIT;

/********************/
void Key_pressed_Q();
void Key_pressed_W();
void Key_pressed_E();
void Key_pressed_R();
void Key_pressed_A();
void Key_pressed_S();
void Key_pressed_D();
void Key_pressed_F();
void Key_pressed_G();
void Key_pressed_Z();
void Key_pressed_X();
void Key_pressed_C();
void Key_pressed_V();
void Key_pressed_B();
void Key_pressed_SHIFT();
void Key_pressed_CTRL();


void ALL_Key_Process();

void ROBOT_Para_Init();

void RC_control_shot();

void Remote_TASK_ALL(SWIT ROBOT_MODE,SWIT SHOT_MODE);
#endif