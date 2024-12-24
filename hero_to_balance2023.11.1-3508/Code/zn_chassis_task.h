#ifndef ZN_CHASSIS_TASK_H
#define ZN_CHASSIS_TASK_H

#include "cmsis_os.h"
#include "main.h"
#include "zn_motor.h"
#include "CAN_receive.h"
#include "can.h"

#define FL_num 0
#define FR_num 1
#define BL_num 2
#define BR_num 3

#define balance_L_num 0
#define balance_R_num 1

#define slideblock_L_num 0
#define slideblock_R_num 1

#define PI 3.1415926f              


#define CHAS_CURRENT_LIMIT    36000//36000    //�ĸ����ӵ��ٶ��ܺ����ֵ���������*4���޹��ʵ���������
#define WARNING_REMAIN_POWER  60


typedef enum
{
    CHASSIS_LOCK          ,  //��������  
    CHASSIS_SEPARATE      , //��̨���̶�������
    CHASSIS_FOLLOW_GIMBAL , //���̸�����̨
    CHASSIS_GYROSCOPE     , //С����ģʽ    

} eChassisAction;


typedef struct
{
    float vx;
    float vy;
    float vw;
    float bal_speed_set;
    float bal_speed_set_last;
    float bal_turn_set;
} Chassis_Speed;


typedef struct
{
    float fTotalCurrentLimit;   //�������䣬ƽ��ģʽ�·����Ǿ��ȵ�
    int16_t  judgDataCorrect;         //����ϵͳ�����Ƿ����
} CurrentLimit;

extern CurrentLimit fCurrentLimit;




/**
  * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void ZN_chassis_task(void const *pvParameters);


void mecanum_calc(Chassis_Speed *speed, int16_t *out_speed);
void Absolute_Cal(Chassis_Speed* absolute_speed, float angle )	;
void balance_mecanum_calc(Chassis_Speed *speed, int16_t *out_speed);


void Chassis_Mode_Change(eChassisAction mode);
int chassis_control_remote_KEY();
void CHASSIS_Single_Loop_Out(void); //���̵�����
void Chassis_Power_Limit(void);
void chassis_balance();
void chassis_nurmal();
void stop_balance();
void chassis_state_update();
float speed_control(float speed_in,float direction);

#endif