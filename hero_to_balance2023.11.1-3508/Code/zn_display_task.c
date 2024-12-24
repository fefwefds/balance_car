#include "zn_display_task.h"
#include "ANO_DT.h"
#include "math.h"
#include "zn_interaction.h"
#include "string.h"
#include "referee_usart_task.h"
#include "referee.h"
#include "INS_task.h"


void ZN_display_task()
{
  static portTickType currentTime;
    
  while(1)
  {
    /*code begin*/  
    
        ANO_DT_Send_Senser( (int)Robot1.Movement.bal_chassis_2006_setspeed[0],//底盘输出功率
                            (int)Robot1.Movement.bal_chassis_2006_setspeed[1],//缓冲功率
                            (int)LQR_k4.out,//剩余血量
                            (int)chassis_contrl.speed_x,
                            (int)chassis_contrl.target_speed_x,
                            INS_angle[1],
                            INS_angle[2],
                            INS_gyro[0],
                            INS_gyro[1],
                            INS_gyro[2]);       
  /*code end*/
  vTaskDelayUntil(&currentTime, 10);//绝对延时 ms
  }
}
