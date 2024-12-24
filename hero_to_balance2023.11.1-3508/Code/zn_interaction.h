#ifndef ZN_INTERACTION_H
#define ZN_INTERACTION_H

#include "zn_chassis_task.h"
#include "zn_gimbal_task.h"//(��ѧ������ϵ)

#define RC_Middle_date 60

typedef enum
{
    EKF ,   //��ֲ�Ĺ����������ǽ���
    AHRS ,   //�ٷ���AHRS����

} Gyroscope_solving;

typedef struct
{
  Gyroscope_solving gyroscope_solving;   //�����ǽ���ѡ��
  
} Feature_selection;

typedef struct
{
  float LENGTH_A;                   //ǰ�����
  float LENGTH_B;                   //�����־�
  float WHEEL_PERIMETER;             //�����־�
  float  CHASSIS_DECELE_RATIO;       //���������
  //��ƫ��
  
  int16_t SlideBlock_R_Middle;
  int16_t SlideBlock_L_Middle;
   
  int16_t YAW_Motor_Middle;//yaw�����λ�Ƕ�ֵ

  int16_t PITCH_Motor_Middle;//PITCH�����λ�Ƕ�ֵ  
  int16_t PITCH_Motor_Front;//PITCH�����󸩽�
  int16_t PITCH_Motor_Behind;//PITCH����������
  
} Mechanical_Para;//��е����

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
  //����
  Chassis_Speed Chassis_speed_set;               //�����趨�ٶ�
  Chassis_Speed Chassis_speed_max;               //�����ٶ��޷�
  Chassis_Speed Chassis_speed_out;               //б�����
  
//  Chassis_Speed Balance_Chassis_speed_set;               //�����趨�ٶ�
//  Chassis_Speed Balance_Chassis_speed_max;               //�����ٶ��޷�
//  Chassis_Speed Balance_Chassis_speed_out;               //б�����
//  
  
  double ramp_vx;//б�����ϵ��
  double ramp_vy;
  double ramp_vw;
  
  float gyroscope_speed;//С�����ٶ�
  
  int16_t bal_chassis_2006_setspeed[2];
  int16_t bal_chassis_2006_maxspeed[2];
  
  int16_t chassis_slideblock_setangle[2];
  int16_t chassis_slideblock_maxangle[2];
  
  int16_t chassis_3508_setspeed[4];		//�ĸ�3508����Ŀ��ת��
  int16_t chassis_3508_maxspeed[4];               //���ת���޷�
  //��̨
  float Gimbal_YAW_motor_angle_set;             //��̨yaw����Ƕ��趨ֵ
  float Gimbal_YAW_imu_angle_set;               //��̨yaw�����ǽǶ��趨ֵ  
  float Gimbal_PITCH_motor_angle_set;           //��̨pitch����Ƕ��趨ֵ
  float Gimbal_PITCH_imu_angle_set;             //��̨pitch�����ǽǶ��趨ֵ
  //����
  int16_t pluck_speed;                        //������ת�� (����ʱ)
  int16_t pluck_angle;                       //�������趨�Ƕȣ�����ʱ��
  int16_t rub_speed;                          //Ħ����ת��
  int16_t rub_speed_2006;
   
} Movement_Data;//�˶�����

//��̨����ģʽ
typedef enum
{
  rc_contorl,//ң��������
  mouse_contorl,//������
}GimbalContorl_MODE;

typedef struct
{
  eChassisAction actChassis;//����״̬
  eGimbalAction actGimbal;//��̨״̬
  GimbalContorl_MODE contorl_MODE;//��̨����ģʽ

  int16_t lid_flag;//���ո�״̬
  int16_t rub_flag;//Ħ����״̬
  int16_t pluck_flag;//����״̬
  int16_t collimation_flag;//����״̬
  int16_t super_power_flag;//��������״̬
  
} Status_Flag;

typedef struct
{
  Mechanical_Para Mechanical;//��е����
  Movement_Data Movement;//�˶�����
  Status_Flag Status;//״̬��־
  Feature_selection feature_selection;  //����ģʽѡ��
} ROBOT;

typedef struct
{
  float Diameter_weel;//�ְ뾶
  
  float angle_x;//��ǰ�Ƕ�
  float Balance_angle_offset;//�Ƕ�ƫ��ֵ
  float gyro_x,gyro_w;//��ǰ���ٶ�
  float ramp_vx,ramp_vy,ramp_wz;//xyz���б��ϵ��
  float target_turn_w,turn_w;
  float target_speed_x,speed_x,speed_y;//x��y���ٶ�
  float target_pose_x,pose_x,pose_y;//����x��y��λ��
  float move_speed,move_direction;
  float vx_set,vy_set,wz_set,vx_last_set;//ң���������ֵ
  float vx_set_out,vy_set_out,wz_set_out;//ң��������������ֵ��б�º�����
  float bal_speed_set;//��������ٶ�
  float torque_balance;//ƽ�⶯����Ӧת��
  float torque_speed;//ǰ��������Ӧת��
  float toque_omega;//��ת������Ӧת��
  float torque_const;//Ť�س���
  float slideblock_r_nowangle,slideblock_l_nowangle,slideblock_r_setangle,slideblock_l_setangle;//����
  float slideblock_kp;//����ϵ��
  
  uint8_t flag_clear_pose;//��̼�ֹͣ�����־(0,����;1,ֹͣ����)
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
  ABCDEFG,//ռλ��
  SWITCH_UP,//��
  SWITCH_DOWN,//��
  SWITCH_MIDDLE//��
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