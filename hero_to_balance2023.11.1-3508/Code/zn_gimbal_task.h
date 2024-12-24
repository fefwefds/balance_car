#ifndef ZN_GIMBAL_TASK_H
#define ZN_GIMBAL_TASK_H

//云台电机中值
#define  YAW_MOTOR_MIDDLE  2730
#define  PITCH_MOTOR_MIDDLE  3600

//底盘状态
typedef enum
{
    GIMBAL_LOCK   ,  //云台锁定    电机编码器值锁定
    GIMBAL_MOUSE  ,  //键鼠控制    yaw陀螺仪角度控制
    GIMBAL_VISION ,  //视觉自瞄
} eGimbalAction;


void ZN_gimbal_task(void const *pvParameters);

void Gimbal_Mode_Change(eGimbalAction mode);

int gimbal_control_remote();

#endif