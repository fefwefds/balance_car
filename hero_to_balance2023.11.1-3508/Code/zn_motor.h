#ifndef ZN_MOTOR_H
#define ZN_MOTOR_H

#include "main.h"
#include "struct_typedef.h"
#include "zn_pid.h"

//���ٿ���
#define ACCEL_1_X 0.6f //ǰ�����ٶ�(����)     
#define ACCEL_2_X 1.1f //ǰ�����ٶ�(����) 
#define ACCEL_1_Y 0.2f //���Ƽ��ٶ�(����) 
#define ACCEL_2_Y 0.6f //���Ƽ��ٶ�(����) 

#define PID_LQR_k1_KP -30.0f
#define PID_LQR_k1_KI 0.0f
#define PID_LQR_k1_KD 0.0f
#define PID_LQR_k1_MAX_OUT 10000.0f
#define PID_LQR_k1_MAX_IOUT 10000.0f

#define PID_LQR_k2_KP -120.0f//
#define PID_LQR_k2_KI 0.0f
#define PID_LQR_k2_KD 0.0f
#define PID_LQR_k2_MAX_OUT 10000.0f
#define PID_LQR_k2_MAX_IOUT 10000.0f

#define PID_LQR_k3_KP 0.0f//8
#define PID_LQR_k3_KI 0.0f//0.5
#define PID_LQR_k3_KD 0.0f//5
#define PID_LQR_k3_MAX_OUT 0.0f
#define PID_LQR_k3_MAX_IOUT 0.0f

#define PID_LQR_k4_KP 1.0f//-1.2
#define PID_LQR_k4_KI 0.05f//-0.05
#define PID_LQR_k4_KD 0.0f//-0.8
#define PID_LQR_k4_MAX_OUT 400.0f
#define PID_LQR_k4_MAX_IOUT 30.0f

#define PID_LQR_k5_KP -400.0f
#define PID_LQR_k5_KI 0.0f
#define PID_LQR_k5_KD -10000.0f
#define PID_LQR_k5_MAX_OUT 100.0f
#define PID_LQR_k5_MAX_IOUT 0.0f

#define PID_LQR_k6_KP -35.0f
#define PID_LQR_k6_KI 0.0f
#define PID_LQR_k6_KD -15.0f
#define PID_LQR_k6_MAX_OUT 10000.0f
#define PID_LQR_k6_MAX_IOUT 10000.0f

#define PID_Offset_KP 1.0f
#define PID_Offset_KI 0.02f
#define PID_Offset_KD 0.0f
#define PID_Offset_MAX_OUT 2000.0f
#define PID_Offset_MAX_IOUT 50.0f

#define PID_SlideBlock_Speed_KP 1.0f
#define PID_SlideBlock_Speed_KI 0.0f
#define PID_SlideBlock_Speed_KD 0.0f
#define PID_SlideBlock_Speed_MAX_OUT 200.0f
#define PID_SlideBlock_Speed_MAX_IOUT 0.0f

#define PID_POSITION 0
#define PID_DELTA 1

extern float chassis_SlideBlock_PID[3] ;
extern float pid_LQR_k1[3] ;
extern float pid_LQR_k2[3] ;
extern float pid_LQR_k3[3] ;
extern float pid_LQR_k4[3] ;
extern float pid_LQR_k5[3] ;
extern float pid_LQR_k6[3] ;
extern float pid_Offset[3] ;

typedef enum
{
    M2006=2,
    M3508 = 0,
    M6020	
}motor_kind;


typedef enum 
{
    ZN_CAN1 = 0,
    ZN_CAN2
}motor_CAN;

typedef struct
{
  motor_kind KIND;   //�������
  motor_CAN CAN;     //CAN
  u8 ID;             //ID
  u16 control_StdId; //���Ʊ�ʶ
  u16 back_StdId;    //������ʶ
  
} Motor_message;

/*���յ��ĵ���Ĳ����ṹ��*/
typedef struct{
    int16_t       speed_rpm;
    float  	  real_current;
    int16_t  	  given_current;
    uint16_t 	  angle;		 
    uint16_t 	  last_angle;	    
    float       total_angle;
}Motor_measure;

typedef struct
{
  Motor_message mes;           //����趨��Ϣ
  Motor_measure data;          //�����������
  pid_type_def speed_pid;      //�ٶȻ�pid����
  pid_type_def angle_pid;      //�ǶȻ�pid����
  pid_type_def gyro_pid;      //���ٶȻ�pid����
  pid_type_def bal_vertical_pid;      //ֱ����pid����
  pid_type_def bal_speed_pid;      //�ٶȻ�pid����
  pid_type_def bal_turn_pid;      //ת��pid����
} Motor_type_def;

void Motor_All_Para_Init();//������в�����ʼ��
void PID_init(bal_pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);

//void Assign_To_Motor_Balance_speed(Motor_type_def *ptr,int16_t speed,int16_t speed_l,int16_t speed_r,float angle,float gyro);
//void Assign_To_Motor_LQR_speed(Motor_type_def *ptr,float angle,float gyro);
  
void Assign_To_Motor_speed(Motor_type_def *ptr,int16_t speed);//����ٶ��趨
void Assign_To_M6020_angle(Motor_type_def *ptr,int16_t angle);//6020����Ƕ��趨
void Assign_To_M3508_angle(Motor_type_def *ptr,int16_t angle);//3508����Ƕ��趨

void Motor_message_Init(Motor_type_def *ptr);//�����Ϣ��ʼ��

float Motor_speed_cal(Motor_type_def *ptr,int16_t speed);//����ٶȻ�����
float M6020_angle_cal(Motor_type_def *ptr,int16_t angle);//6020����ǶȻ�����
float M3508_angle_cal(Motor_type_def *ptr,int16_t angle);//3508����ǶȻ�����

//void Vertical_Ring_PD(Motor_type_def *ptr,float angle,float gyro);
//void Speed_Ring_PI(Motor_type_def *ptr,int16_t speed_set,int16_t speed_l,int16_t speed_r);
fp32 PID_calc(bal_pid_type_def *pid,fp32 ref,fp32 set);
void PID_init(bal_pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);

void total_angle_get(Motor_type_def *ptr);//���3508���total_angle

extern Motor_type_def yaw_motor;
extern Motor_type_def pitch_motor;

extern Motor_type_def balance_L;
extern Motor_type_def balance_R;
extern Motor_type_def slideblock_R;
extern Motor_type_def slideblock_L;

extern Motor_type_def  rotate_motor;//����
extern Motor_type_def rub_motor_L;//Ħ����
extern Motor_type_def rub_motor_R;
extern Motor_type_def rub_motor_2006;

//extern Motor_type_def  drive_FL;//��ǰ
//extern Motor_type_def  drive_FR;//��ǰ
//extern Motor_type_def  drive_BL;//���
//extern Motor_type_def  drive_BR;//�Һ�

/* ���pid */
extern bal_pid_type_def LQR_k1;
extern bal_pid_type_def LQR_k2;
extern bal_pid_type_def LQR_k3;
extern bal_pid_type_def LQR_k4;
extern bal_pid_type_def LQR_k5;
extern bal_pid_type_def LQR_k6;
extern bal_pid_type_def Offset_PID;
extern bal_pid_type_def SlideBlock_PID[2];


#endif