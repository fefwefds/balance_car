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
*���ܣ�б�º���,ʹĿ�����ֵ������������ֵ
*������ float finala     �����������
        float now        ��ǰ���
        float ramp      �仯�ٶ�(Խ��Խ��)
*����ֵ��now        ��ǰ���
*���ڣ�2023.8.19
*�޸����ڣ�
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
*���ܣ����ٿ���
*������float x   Ҫ��������
*����ֵ��  0   ��С��0
           y    ���������ֵ
*���ڣ�2023.10.6
*�޸����ڣ�
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
*���ܣ�������ƽ��������
*������float num   Ҫ�����
*����ֵ��  y    ����ƽ�������������
*���ڣ�2023.10.6
*�޸����ڣ�
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
*���ܣ�б�º�����ʼ��
*������ ramp_function_source_t *ramp_source_type  б�º����ṹ��
        float frame_period                        �����ʱ�� ��λs
        float max                                 ���ֵ
        float min                                 ��Сֵ
*����ֵ�� 
*���ڣ�2023.10.6
*�޸����ڣ�
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
*���ܣ�б�º������㣬���������ֵ���е��ӣ����뵥λΪ /s ��һ������������ֵ
*������ ramp_function_source_t *ramp_source_type  б�º����ṹ��
        float input   ����ֵ
*����ֵ�� ramp_source_type->out  ����б�º���������ֵ
*���ڣ�2023.10.6
*�޸����ڣ�
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
*���ܣ�����ֵ����
*������ float num   Ҫ���Ƶı���
        float Limit  ��������
*����ֵ�� num  �������Ժ������ú����
*���ڣ�2023.10.6
*�޸����ڣ�
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
*���ܣ��жϷ���λ
*������ float value   Ҫ�жϵ���
*����ֵ�� 1.0f    ����
          -1.0f    ����
*���ڣ�2023.10.6
*�޸����ڣ�
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
*���ܣ���������
*������ float value   Ҫ�жϵ���
        float minValue  ������������Сֵ
        float maxValue   �������������ֵ
*����ֵ�� 0.0f ��������
          Value   ��������
*���ڣ�2023.10.6
*�޸����ڣ�
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
*���ܣ�int26����
*������ int16_t Value   Ҫ�жϵ���
        int16_t minValue int26��������Сֵ
        int16_t maxValue int26���������ֵ
*����ֵ�� 0.0f ��������
          Value   ��������
*���ڣ�2023.10.6
*�޸����ڣ�
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
*���ܣ��޷�����  ������
*������ float Value    Ҫ�޷�����
        float minValue  ����
        float maxValue  ����
*����ֵ�� minValue  С�����޵�������
          VmaxValue  �������޵�������
          Value       ���޷�������
*���ڣ�2023.10.6
*�޸����ڣ�
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
*���ܣ��޷�����  ����
*������ int16_t Value    Ҫ�޷�����
        int16_t minValue  ����
        int16_t maxValue  ����
*����ֵ�� minValue  С�����޵�������
          VmaxValue  �������޵�������
          Value       ���޷�������
*���ڣ�2023.10.6
*�޸����ڣ�
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
*���ܣ�ѭ���޷����� 
*������ float Input   ����
        float minValue  ����
        float maxValue  ����
*����ֵ�� Value    ����ѭ���޷����ú����ֵ
*���ڣ�2023.10.6
*�޸����ڣ�
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
*���ܣ��Ƕȸ�ʽ��Ϊ  -180~180
*������ float Ang  Ҫ�Ƕȸ�ʽ������
*����ֵ�� Value    �Ƕȸ�ʽ�����ֵ
*���ڣ�2023.10.6
*�޸����ڣ�
*********************************************/
float theta_format(float Ang)
{
    return loop_float_constrain(Ang, -180.0f, 180.0f);
}




/*********************************************
*���ܣ�����������������
*������ float raw  Ҫ��������ĸ�����
*����ֵ�� integer    �������������ĸ�����
*���ڣ�2023.10.6
*�޸����ڣ�
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
*���ܣ���С���˷���ʼ��   �������
*������ Ordinary_Least_Squares_t *OLS  ��С���˷��ṹ��
         uint16_t order               ������
*����ֵ��
*���ڣ�2023.10.6
*�޸����ڣ�
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
*���ܣ���С���˷����
*������ Ordinary_Least_Squares_t *OLS  ��С���˷��ṹ��
        float deltax                    �ź�����������һ������ʱ����
        float y                          �ź�ֵ
*����ֵ��
*���ڣ�2023.10.6
*�޸����ڣ�
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
*���ܣ���С���˷���ȡ�ź�΢��
*������ Ordinary_Least_Squares_t *OLS  ��С���˷��ṹ��
        float deltax                    �ź�����������һ������ʱ����
        float y                          �ź�ֵ
*����ֵ�� OLS->k      б��k
*���ڣ�2023.10.6
*�޸����ڣ�
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
*���ܣ���ȡ��С���˷���ȡ�ź�΢��
*������ Ordinary_Least_Squares_t *OLS  ��С���˷��ṹ��
*����ֵ�� OLS->k      б��k
*���ڣ�2023.10.6
*�޸����ڣ�
*********************************************/
float Get_OLS_Derivative(Ordinary_Least_Squares_t *OLS)
{
    return OLS->k;
}



/*********************************************
*���ܣ���С���˷�ƽ���ź�
*������ Ordinary_Least_Squares_t *OLS  ��С���˷��ṹ��
        float deltax                    �ź�����������һ������ʱ����
        float y                          �ź�
*����ֵ�� OLS->k * OLS->x[OLS->Order - 1] + OLS->b   ����ƽ�����
*���ڣ�2023.10.6
*�޸����ڣ�
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
*���ܣ���С���˷�ƽ���ź�
*������ Ordinary_Least_Squares_t *OLS  ��С���˷��ṹ��
*����ֵ�� OLS->k * OLS->x[OLS->Order - 1] + OLS->b   ����ƽ�����
*���ڣ�2023.10.6
*�޸����ڣ�
*********************************************/
float Get_OLS_Smooth(Ordinary_Least_Squares_t *OLS)
{
    return OLS->k * OLS->x[OLS->Order - 1] + OLS->b;
}
