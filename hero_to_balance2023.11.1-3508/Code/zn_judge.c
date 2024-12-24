#include "zn_judge.h"

int Judge_Data_TF = FALSE;          //���������Ƿ���ã�������������
xFrameHeader        FrameHeader;    //����֡ͷ��Ϣ
uint8_t retval_tf = FALSE;          //������ȷ����־��ÿ�ε��ö�ȡ����ϵͳ���ݺ�������Ĭ��Ϊ����
/**
  * @brief  ��ȡ�������ݣ��ж��ж�ȡ��֤�ٶ�
  * @param  ��������
  * @retval �Ƿ�������ж�������
  * @attention  �ڴ��ж�֡ͷ��CRCУ�飬������д�����ݣ����ظ��ж�֡ͷ
  */
int Judge_Read_Data(uint8_t *ReadFromUsart)
{
  uint16_t judge_length;//ͳ��һ֡���ݳ���

  //�����ݰ��������κδ���
  if(ReadFromUsart == NULL)
    {
	return -1;
    }
  memcpy(&FrameHeader, ReadFromUsart, LEN_HEADER);
  //�ж�֡ͷ�����Ƿ�Ϊ0xA5
  if(ReadFromUsart[ SOF ] == JUDGE_FRAME_HEADER)
    {
     //֡ͷCRCУ��
     if (verify_CRC8_check_sum( ReadFromUsart, LEN_HEADER ) == TRUE)
      {
        //ͳ��һ֡���ݳ��ȣ�����CR16У��
        judge_length = ReadFromUsart[ DATA_LENGTH ] + LEN_HEADER + LEN_CMDID + LEN_TAIL;      
        //֡βCRC16У��
	if(verify_CRC16_check_sum(ReadFromUsart,judge_length) == TRUE)
	{
	  retval_tf = TRUE;
        }     
      }
          
    }
  if(retval_tf == TRUE)
    {
      Judge_Data_TF = TRUE;//����������
    }
  else		//ֻҪCRC16У�鲻ͨ����ΪFALSE
    {
      Judge_Data_TF = FALSE;//����������
    }
  return retval_tf;//����������������
}

/**
  * @brief  �����Ƿ����
  * @param  void
  * @retval  TRUE����   FALSE������
  * @attention  �ڲ��ж�ȡ������ʵʱ�ı䷵��ֵ
  */
int JUDGE_sGetDataState(void)
{
    return Judge_Data_TF;
}

/**
  * @brief  ��ȡʣ�ཹ������
  * @param  void
  * @retval ʣ�໺�役�����������60��
  * @attention  
  */
uint16_t JUDGE_fGetRemainEnergy(void)
{
    return ((int)power_heat_data_t.chassis_power_buffer);//���幦��
}

/**
  * @brief  ��ȡǹ������
  * @param  void
  * @retval 17mm
  * @attention  ʵʱ����
  */
uint16_t JUDGE_usGetRemoteHeat17(void)
{
    return power_heat_data_t.shooter_id1_17mm_cooling_heat;//17mmǹ������
}

/**
  * @brief  ��ȡǹ������
  * @param  void
  * @retval ��ǰ�ȼ�17mm��������
  * @attention  
  */
uint16_t JUDGE_usGetHeatLimit(void)
{
	return robot_state.shooter_heat0_cooling_limit;
}

/////**
//  * @brief  ��ȡ����
//  * @param  void
//  * @retval 17mm
//  * @attention  ʵʱ����
//  */
//float JUDGE_usGetSpeedHeat17(void)
//{
//    return shoot_data_t.bullet_speed;
//}