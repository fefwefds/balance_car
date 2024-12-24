#ifndef _ZN_JUDGE_H
#define _ZN_JUDGE_H

#include "cmsis_os.h"
#include "crc8_crc16.h"
#include "string.h"
#include "referee.h"

#define    JUDGE_DATA_ERROR      0
#define    FALSE    0
#define    TRUE     1

#define    LEN_HEADER    5        //帧头长
#define    LEN_CMDID     2        //命令码长度
#define    LEN_TAIL      2	  //帧尾CRC16

//起始字节，协议固定为0xA5
#define    JUDGE_FRAME_HEADER         (0xA5)

//自定义帧头
typedef struct
{
    uint8_t  SOF;
    uint16_t DataLength;
    uint8_t  Seq;
    uint8_t  CRC8;
}xFrameHeader;

//5字节帧头，偏移位置
typedef enum
{
	SOF          = 0,//起始位
	DATA_LENGTH  = 1,//帧内数据长度，根据这个来获取数据长度
	SEQ          = 3,//包序号
	CRC8         = 4 //CRC8
	
}FrameHeaderOffset;

//命令码ID，用来判断接收的是什么数据
//typedef enum
//{ 
//    ID_game_state       		= 0x0001,//比赛状态数据
//    ID_game_result 	   		= 0x0002,//比赛结果数据
//    ID_game_robot_survivors       	= 0x0003,//机器人血量数据
//    ID_event_data  			= 0x0101,//场地事件数据
//    ID_supply_projectile_action   	= 0x0102,//补给站动作标识
//    ID_supply_projectile_booking 	= 0x0103,//请求补给站补弹数据
//    ID_game_robot_state    		= 0x0201,//机器人状态数据
//    ID_power_heat_data    		= 0x0202,//实时功率热量数据
//    ID_game_robot_pos        	        = 0x0203,//机器人位置数据
//    ID_buff_musk			= 0x0204,//机器人增益数据
//    ID_aerial_robot_energy		= 0x0205,//空中机器人能量状态数据
//    ID_robot_hurt			= 0x0206,//伤害状态数据
//    ID_shoot_data			= 0x0207,//实时射击数据
//} CmdID;

extern xFrameHeader  FrameHeader;

int Judge_Read_Data(uint8_t *ReadFromUsart);
int JUDGE_sGetDataState(void);
uint16_t JUDGE_fGetRemainEnergy(void);
uint16_t JUDGE_usGetRemoteHeat17(void);
uint16_t JUDGE_usGetHeatLimit(void);
float JUDGE_usGetSpeedHeat17(void);

#endif