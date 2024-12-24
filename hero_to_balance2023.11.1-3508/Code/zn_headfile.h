#ifndef _HEADFILE_H
#define _HEADFILE_H

#include "main.h"
#include "cmsis_os.h"
#include <math.h>

//系统驱动文件
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_usart.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_can.h"


//CODE
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "led_flow_task.h"
#include "zn_chassis_motor.h"
#include "zn_chassis_task.h"
#include "zn_gimbal_motor.h"
#include "zn_gimbal_task.h"
#include "zn_shoot_task.h"
#include "zn_motor.h"




#endif