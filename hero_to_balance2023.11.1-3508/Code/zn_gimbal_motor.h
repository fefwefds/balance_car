#ifndef ZN_GIMBAL_MOTOR_H
#define ZN_GIMBAL_MOTOR_H

#include "struct_typedef.h"
#include "cmsis_os.h"
#include "main.h"
#include "zn_pid.h"
#include "CAN_receive.h"

extern pid_type_def YAW_imu_angle;
extern pid_type_def YAW_imu_speed;

void YAW_Angle_Set(Motor_type_def *motor,pid_type_def *angle_pid,pid_type_def *speed_pid,float yaw_angle_set);





#endif