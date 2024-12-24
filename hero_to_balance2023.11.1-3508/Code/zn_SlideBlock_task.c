#include "zn_slideblock_task.h"
#include "INS_task.h"
#include "cmsis_os.h"
#include "zn_motor.h"
#include "zn_interaction.h"
#include "zn_chassis_task.h"

void ZN_slideblock_task(void const *pvParameters)
{	
  static portTickType currentTime;  
  
  Robot1.Mechanical.SlideBlock_R_Middle=2700;//4800 600
  Robot1.Mechanical.SlideBlock_L_Middle=5200;//3100 7300
  
  chassis_contrl.slideblock_kp=50;//5000
  
  vTaskDelayUntil(&currentTime, 1500);//等待电调上电 
  
  while(1)
  {
   currentTime = xTaskGetTickCount();	//获取当前系统时间
    /*code begin*/   
   
   slideblock_normal();

   slideblock_loop();

  /*code end*/
  vTaskDelayUntil(&currentTime, 2);//绝对延时 2ms
  }
}

void slideblock_loop()
{
   Assign_To_M6020_angle(&slideblock_L,Robot1.Movement.chassis_slideblock_setangle[slideblock_L_num]);
   Assign_To_M6020_angle(&slideblock_R,Robot1.Movement.chassis_slideblock_setangle[slideblock_R_num]);  
}

void slideblock_normal()
{
  PID_calc(&SlideBlock_PID[0],0,LQR_k4.error[0]);
  PID_calc(&SlideBlock_PID[1],0,LQR_k4.error[0]);
  
  chassis_contrl.slideblock_l_setangle=Robot1.Mechanical.SlideBlock_L_Middle+chassis_contrl.slideblock_kp*SlideBlock_PID[0].out;
  chassis_contrl.slideblock_r_setangle=Robot1.Mechanical.SlideBlock_R_Middle-chassis_contrl.slideblock_kp*SlideBlock_PID[1].out;
  
  Limit(chassis_contrl.slideblock_r_setangle,4800,600);
  Limit(chassis_contrl.slideblock_l_setangle,7300,3100);
  
  Robot1.Movement.chassis_slideblock_setangle[slideblock_L_num]=chassis_contrl.slideblock_l_setangle;
  Robot1.Movement.chassis_slideblock_setangle[slideblock_R_num]=chassis_contrl.slideblock_r_setangle;
}