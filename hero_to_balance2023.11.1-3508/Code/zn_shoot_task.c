#include "zn_shoot_task.h"
#include "cmsis_os.h"
#include "main.h"

#include "zn_motor.h"
#include "CAN_receive.h"
#include "remote_control.h"
#include "zn_interaction.h"

uint8_t send_data[8];

void ZN_shoot_task(void const *pvParameters)
{
  static portTickType currentTime;
    
  while(1)
  {
   currentTime = xTaskGetTickCount();	//获取当前系统时间
    /*code begin*/
   if(Robot1.Status.rub_flag == 1)
   {
     Robot1.Movement.rub_speed = 4000;
     Robot1.Movement.rub_speed_2006 = 8000;
//     Robot1.Movement.rub_speed = 9000;
//     Robot1.Movement.rub_speed_2006 = 14000;
   }
   else
   {
     Robot1.Movement.rub_speed = 0;
     Robot1.Movement.rub_speed_2006 =0;
   }

   Assign_To_Motor_speed(&rub_motor_L, Robot1.Movement.rub_speed);
   Assign_To_Motor_speed(&rub_motor_R,-Robot1.Movement.rub_speed);
   Assign_To_Motor_speed(&rub_motor_2006,Robot1.Movement.rub_speed_2006);
     
  /*code end*/
  vTaskDelayUntil(&currentTime, 3);//绝对延时 ms
  }
}
