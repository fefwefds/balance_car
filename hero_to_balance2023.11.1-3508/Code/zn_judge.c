#include "zn_judge.h"

int Judge_Data_TF = FALSE;          //裁判数据是否可用，辅助函数调用
xFrameHeader        FrameHeader;    //发送帧头信息
uint8_t retval_tf = FALSE;          //数据正确与否标志，每次调用读取裁判系统数据函数都先默认为错误
/**
  * @brief  读取裁判数据，中断中读取保证速度
  * @param  缓存数据
  * @retval 是否对正误判断做处理
  * @attention  在此判断帧头和CRC校验，无误再写入数据，不重复判断帧头
  */
int Judge_Read_Data(uint8_t *ReadFromUsart)
{
  uint16_t judge_length;//统机一帧数据长度

  //无数据包，则不作任何处理
  if(ReadFromUsart == NULL)
    {
	return -1;
    }
  memcpy(&FrameHeader, ReadFromUsart, LEN_HEADER);
  //判断帧头数据是否为0xA5
  if(ReadFromUsart[ SOF ] == JUDGE_FRAME_HEADER)
    {
     //帧头CRC校验
     if (verify_CRC8_check_sum( ReadFromUsart, LEN_HEADER ) == TRUE)
      {
        //统计一帧数据长度，用于CR16校验
        judge_length = ReadFromUsart[ DATA_LENGTH ] + LEN_HEADER + LEN_CMDID + LEN_TAIL;      
        //帧尾CRC16校验
	if(verify_CRC16_check_sum(ReadFromUsart,judge_length) == TRUE)
	{
	  retval_tf = TRUE;
        }     
      }
          
    }
  if(retval_tf == TRUE)
    {
      Judge_Data_TF = TRUE;//辅助函数用
    }
  else		//只要CRC16校验不通过就为FALSE
    {
      Judge_Data_TF = FALSE;//辅助函数用
    }
  return retval_tf;//对数据正误做处理
}

/**
  * @brief  数据是否可用
  * @param  void
  * @retval  TRUE可用   FALSE不可用
  * @attention  在裁判读取函数中实时改变返回值
  */
int JUDGE_sGetDataState(void)
{
    return Judge_Data_TF;
}

/**
  * @brief  读取剩余焦耳能量
  * @param  void
  * @retval 剩余缓冲焦耳能量（最大60）
  * @attention  
  */
uint16_t JUDGE_fGetRemainEnergy(void)
{
    return ((int)power_heat_data_t.chassis_power_buffer);//缓冲功率
}

/**
  * @brief  读取枪口热量
  * @param  void
  * @retval 17mm
  * @attention  实时热量
  */
uint16_t JUDGE_usGetRemoteHeat17(void)
{
    return power_heat_data_t.shooter_id1_17mm_cooling_heat;//17mm枪口热量
}

/**
  * @brief  读取枪口热量
  * @param  void
  * @retval 当前等级17mm热量上限
  * @attention  
  */
uint16_t JUDGE_usGetHeatLimit(void)
{
	return robot_state.shooter_heat0_cooling_limit;
}

/////**
//  * @brief  读取射速
//  * @param  void
//  * @retval 17mm
//  * @attention  实时热量
//  */
//float JUDGE_usGetSpeedHeat17(void)
//{
//    return shoot_data_t.bullet_speed;
//}