#include "stdlib.h"
#include "string.h"
#include "zn_lib.h"
#include "math.h"
#include "main.h"

#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif

uint8_t GlobalDebugMode = 7;


/*********************************************
*功能：斜坡函数,使目标输出值缓慢等于期望值
*参数： float finala     期望最终输出
        float now        当前输出
        float ramp      变化速度(越大越快)
*返回值：now        当前输出
*日期：2023.8.19
*修改日期：
*********************************************/
float RAMP_float( float finala, float now, float ramp )
{
  
   float buffer = 0;
    
   buffer = finala - now;
	
    if (buffer > 0)
    {
      if (buffer > ramp)
      {  
       now += ramp;
      }   
      else
      {
        now += buffer;
      }
    }
    else
     {
       if (buffer < -ramp)
       {
          now += -ramp;
       }
       else
       {
          now += buffer;
       }
     }
		
    return now;
}


/*********************************************
*功能：快速开方
*参数：float x   要开方的数
*返回值：  0   数小于0
           y    开方后的数值
*日期：2023.10.6
*修改日期：
*********************************************/
float Sqrt(float x)
{
    float y;
    float delta;
    float maxError;

    if (x <= 0)
    {
        return 0;
    }

    // initial guess
    y = x / 2;

    // refine
    maxError = x * 0.001f;

    do
    {
        delta = (y * y) - x;
        y -= delta / (2 * y);
    } while (delta > maxError || delta < -maxError);

    return y;
}


/*********************************************
*功能：快速求平方根倒数
*参数：float num   要求的数
*返回值：  y    经过平方根倒数后的数
*日期：2023.10.6
*修改日期：
*********************************************/
/*
float invSqrt(float num)
{
    float halfnum = 0.5f * num;
    float y = num;
    long i = *(long *)&y;
    i = 0x5f375a86- (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}*/

/*********************************************
*功能：斜坡函数初始化
*参数： ramp_function_source_t *ramp_source_type  斜坡函数结构体
        float frame_period                        间隔的时间 单位s
        float max                                 最大值
        float min                                 最小值
*返回值： 
*日期：2023.10.6
*修改日期：
*********************************************/
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
}

/*********************************************
*功能：斜坡函数计算，根据输入的值进行叠加，输入单位为 /s 即一秒后增加输入的值
*参数： ramp_function_source_t *ramp_source_type  斜坡函数结构体
        float input   输入值
*返回值： ramp_source_type->out  经过斜坡函数处理后的值
*日期：2023.10.6
*修改日期：
*********************************************/
float ramp_calc(ramp_function_source_t *ramp_source_type, float input)
{
    ramp_source_type->input = input;
    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
    return ramp_source_type->out;
}



/*********************************************
*功能：绝对值限制
*参数： float num   要限制的变量
        float Limit  限制区间
*返回值： num  经过绝对函数作用后的数
*日期：2023.10.6
*修改日期：
*********************************************/
float abs_limit(float num, float Limit)
{
    if (num > Limit)
    {
        num = Limit;
    }
    else if (num < -Limit)
    {
        num = -Limit;
    }
    return num;
}



/*********************************************
*功能：判断符号位
*参数： float value   要判断的数
*返回值： 1.0f    正数
          -1.0f    负数
*日期：2023.10.6
*修改日期：
*********************************************/
float sign(float value)
{
    if (value >= 0.0f)
    {
        return 1.0f;
    }
    else
    {
        return -1.0f;
    }
}



/*********************************************
*功能：浮点死区
*参数： float value   要判断的数
        float minValue  浮点死区的最小值
        float maxValue   浮点死区的最大值
*返回值： 0.0f 在死区外
          Value   在死区内
*日期：2023.10.6
*修改日期：
*********************************************/
float float_deadband(float Value, float minValue, float maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}



/*********************************************
*功能：int26死区
*参数： int16_t Value   要判断的数
        int16_t minValue int26死区的最小值
        int16_t maxValue int26死区的最大值
*返回值： 0.0f 在死区外
          Value   在死区内
*日期：2023.10.6
*修改日期：
*********************************************/
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }
    return Value;
}



/*********************************************
*功能：限幅函数  浮点型
*参数： float Value    要限幅的数
        float minValue  上限
        float maxValue  下限
*返回值： minValue  小于下限等于下限
          VmaxValue  大于上限等于上限
          Value       在限幅区间内
*日期：2023.10.6
*修改日期：
*********************************************/
float float_constrain(float Value, float minValue, float maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}



/*********************************************
*功能：限幅函数  整数
*参数： int16_t Value    要限幅的数
        int16_t minValue  上限
        int16_t maxValue  下限
*返回值： minValue  小于下限等于下限
          VmaxValue  大于上限等于上限
          Value       在限幅区间内
*日期：2023.10.6
*修改日期：
*********************************************/
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}



/*********************************************
*功能：循环限幅函数 
*参数： float Input   输入
        float minValue  上限
        float maxValue  下限
*返回值： Value    经过循环限幅作用后的数值
*日期：2023.10.6
*修改日期：
*********************************************/
float loop_float_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        float len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        float len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}



/*********************************************
*功能：角度格式化为  -180~180
*参数： float Ang  要角度格式化的数
*返回值： Value    角度格式化后的值
*日期：2023.10.6
*修改日期：
*********************************************/
float theta_format(float Ang)
{
    return loop_float_constrain(Ang, -180.0f, 180.0f);
}




/*********************************************
*功能：浮点数的四舍五入
*参数： float raw  要四舍五入的浮点数
*返回值： integer    经过四舍五入后的浮点数
*日期：2023.10.6
*修改日期：
*********************************************/
int float_rounding(float raw)
{
    static int integer;
    static float decimal;
    integer = (int)raw;
    decimal = raw - integer;
    if (decimal > 0.5f)
        integer++;
    return integer;
}



/*********************************************
*功能：最小二乘法初始化   函数拟合
*参数： Ordinary_Least_Squares_t *OLS  最小二乘法结构体
         uint16_t order               样本数
*返回值：
*日期：2023.10.6
*修改日期：
*********************************************/
void OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order)
{
    OLS->Order = order;
    OLS->Count = 0;
    OLS->x = (float *)user_malloc(sizeof(float) * order);
    OLS->y = (float *)user_malloc(sizeof(float) * order);
    OLS->k = 0;
    OLS->b = 0;
    memset((void *)OLS->x, 0, sizeof(float) * order);
    memset((void *)OLS->y, 0, sizeof(float) * order);
    memset((void *)OLS->t, 0, sizeof(float) * 4);
}



/*********************************************
*功能：最小二乘法拟合
*参数： Ordinary_Least_Squares_t *OLS  最小二乘法结构体
        float deltax                    信号新样本距上一个样本时间间隔
        float y                          信号值
*返回值：
*日期：2023.10.6
*修改日期：
*********************************************/
void OLS_Update(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }
    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);
    OLS->b = (OLS->t[0] * OLS->t[3] - OLS->t[1] * OLS->t[2]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= OLS->Order;
}



/*********************************************
*功能：最小二乘法提取信号微分
*参数： Ordinary_Least_Squares_t *OLS  最小二乘法结构体
        float deltax                    信号新样本距上一个样本时间间隔
        float y                          信号值
*返回值： OLS->k      斜率k
*日期：2023.10.6
*修改日期：
*********************************************/
float OLS_Derivative(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }

    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= OLS->Order;

    return OLS->k;
}


/*********************************************
*功能：获取最小二乘法提取信号微分
*参数： Ordinary_Least_Squares_t *OLS  最小二乘法结构体
*返回值： OLS->k      斜率k
*日期：2023.10.6
*修改日期：
*********************************************/
float Get_OLS_Derivative(Ordinary_Least_Squares_t *OLS)
{
    return OLS->k;
}



/*********************************************
*功能：最小二乘法平滑信号
*参数： Ordinary_Least_Squares_t *OLS  最小二乘法结构体
        float deltax                    信号新样本距上一个样本时间间隔
        float y                          信号
*返回值： OLS->k * OLS->x[OLS->Order - 1] + OLS->b   返回平滑输出
*日期：2023.10.6
*修改日期：
*********************************************/
float OLS_Smooth(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }

    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);
    OLS->b = (OLS->t[0] * OLS->t[3] - OLS->t[1] * OLS->t[2]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= OLS->Order;

    return OLS->k * OLS->x[OLS->Order - 1] + OLS->b;
}


/*********************************************
*功能：最小二乘法平滑信号
*参数： Ordinary_Least_Squares_t *OLS  最小二乘法结构体
*返回值： OLS->k * OLS->x[OLS->Order - 1] + OLS->b   返回平滑输出
*日期：2023.10.6
*修改日期：
*********************************************/
float Get_OLS_Smooth(Ordinary_Least_Squares_t *OLS)
{
    return OLS->k * OLS->x[OLS->Order - 1] + OLS->b;
}
