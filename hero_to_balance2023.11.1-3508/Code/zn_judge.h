#ifndef _ZN_JUDGE_H
#define _ZN_JUDGE_H

#include "cmsis_os.h"
#include "crc8_crc16.h"
#include "string.h"
#include "referee.h"

#define    JUDGE_DATA_ERROR      0
#define    FALSE    0
#define    TRUE     1

#define    LEN_HEADER    5        //֡ͷ��
#define    LEN_CMDID     2        //�����볤��
#define    LEN_TAIL      2	  //֡βCRC16

//��ʼ�ֽڣ�Э��̶�Ϊ0xA5
#define    JUDGE_FRAME_HEADER         (0xA5)

//�Զ���֡ͷ
typedef struct
{
    uint8_t  SOF;
    uint16_t DataLength;
    uint8_t  Seq;
    uint8_t  CRC8;
}xFrameHeader;

//5�ֽ�֡ͷ��ƫ��λ��
typedef enum
{
	SOF          = 0,//��ʼλ
	DATA_LENGTH  = 1,//֡�����ݳ��ȣ������������ȡ���ݳ���
	SEQ          = 3,//�����
	CRC8         = 4 //CRC8
	
}FrameHeaderOffset;

//������ID�������жϽ��յ���ʲô����
//typedef enum
//{ 
//    ID_game_state       		= 0x0001,//����״̬����
//    ID_game_result 	   		= 0x0002,//�����������
//    ID_game_robot_survivors       	= 0x0003,//������Ѫ������
//    ID_event_data  			= 0x0101,//�����¼�����
//    ID_supply_projectile_action   	= 0x0102,//����վ������ʶ
//    ID_supply_projectile_booking 	= 0x0103,//���󲹸�վ��������
//    ID_game_robot_state    		= 0x0201,//������״̬����
//    ID_power_heat_data    		= 0x0202,//ʵʱ������������
//    ID_game_robot_pos        	        = 0x0203,//������λ������
//    ID_buff_musk			= 0x0204,//��������������
//    ID_aerial_robot_energy		= 0x0205,//���л���������״̬����
//    ID_robot_hurt			= 0x0206,//�˺�״̬����
//    ID_shoot_data			= 0x0207,//ʵʱ�������
//} CmdID;

extern xFrameHeader  FrameHeader;

int Judge_Read_Data(uint8_t *ReadFromUsart);
int JUDGE_sGetDataState(void);
uint16_t JUDGE_fGetRemainEnergy(void);
uint16_t JUDGE_usGetRemoteHeat17(void);
uint16_t JUDGE_usGetHeatLimit(void);
float JUDGE_usGetSpeedHeat17(void);

#endif