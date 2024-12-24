#ifndef ZN_PID_H
#define ZN_PID_H

#include "main.h"
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "bsp_dwt.h"
#include "struct_typedef.h"
#include "zn_lib.h"
#include "math.h"

////�޷�
//#define Limit(input, max,min)   \
//    {                          \
//        if (input > max)       \
//        {                      \
//            input = max;       \
//        }                      \
//        else if (input < min) \
//        {                      \
//            input = min;      \
//        }                      \
//    }
//
////����
//#define LimitDeadBand(input, min)              \
//        if (input <= min && input >= -min)       \
//        {                                       \
//            input = 0;                          \
//        } 
//
//#define zn_abs(x) ((x) > 0 ? (x) : (-x))


/*����PID��ز����ṹ��*/
typedef struct _PID_TypeDef
{
	float  kp;                       //����
	float  ki;                       //����
	float  kd;                       //΢��     
  
  	float  MaxOutput;		        //����޷�
	float  IntegralLimit;		//�����޷�
	float  DeadBand;			//����	
  
	float  target;		      //�ٶȻ�Ŀ��ֵ
        float   measure;		      //�ٶȲ���ֵ
        
        float   error;		      //���
	float   error_last;      	      //�ϴ����
        float   error_all;               //ƫ�����
 
        float  P_out;                          //POUT
	float  I_out;                          //IOUT
	float  D_out;                          //DOUT
        float  All_out;                    //�����
        
        float Mechanical_balance;                //ƽ��Ƕ���ֵ
	
}pid_type_def; 

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
  uint8_t mode;
  
  //PID������
  fp32 Kp;
  fp32 Ki;
  fp32 Kd;
  
  fp32 max_out;   //������
  fp32 max_iout;  //���������
  
  fp32 set;
  fp32 fdb;
  
  fp32 out;
  fp32 Pout;
  fp32 Iout;
  fp32 Dout;
  fp32 Dbuf[3];   //΢���� 0���� 1��һ�� 2���ϴ�
  fp32 error[3];  //����� 0���� 1��һ�� 2���ϴ�
  
}bal_pid_type_def;

void pid_paragram_init(pid_type_def *ptr,float kp,float ki,float kd);

float RAMP_float( float finala, float now, float ramp );

//��������ֲ�����̵Ŀ����㷨

#ifndef abs
#define abs(x) ((x > 0) ? x : -x)
#endif


#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

/******************************** FUZZY PID **********************************/
#define NB -3
#define NM -2
#define NS -1
#define ZE 0
#define PS 1
#define PM 2
#define PB 3

typedef __packed struct
{
    float KpFuzzy;
    float KiFuzzy;
    float KdFuzzy;

    float (*FuzzyRuleKp)[7];
    float (*FuzzyRuleKi)[7];
    float (*FuzzyRuleKd)[7];

    float KpRatio;
    float KiRatio;
    float KdRatio;

    float eStep;
    float ecStep;

    float e;
    float ec;
    float eLast;

    uint32_t DWT_CNT;
    float dt;
} FuzzyRule_t;

void Fuzzy_Rule_Init(FuzzyRule_t *fuzzyRule, float (*fuzzyRuleKp)[7], float (*fuzzyRuleKi)[7], float (*fuzzyRuleKd)[7],
                     float kpRatio, float kiRatio, float kdRatio,
                     float eStep, float ecStep);
void Fuzzy_Rule_Implementation(FuzzyRule_t *fuzzyRule, float measure, float ref);

/******************************* PID CONTROL *********************************/
typedef enum pid_Improvement_e
{
    NONE = 0X00,                        //0000 0000
    Integral_Limit = 0x01,              //0000 0001
    Derivative_On_Measurement = 0x02,   //0000 0010
    Trapezoid_Intergral = 0x04,         //0000 0100
    Proportional_On_Measurement = 0x08, //0000 1000
    OutputFilter = 0x10,                //0001 0000
    ChangingIntegrationRate = 0x20,     //0010 0000
    DerivativeFilter = 0x40,            //0100 0000
    ErrorHandle = 0x80,                 //1000 0000
} PID_Improvement_e;

typedef enum errorType_e
{
    PID_ERROR_NONE = 0x00U,
    Motor_Blocked = 0x01U
} ErrorType_e;

typedef __packed struct
{
    uint64_t ERRORCount;
    ErrorType_e ERRORType;
} PID_ErrorHandler_t;

typedef __packed struct pid_t
{
    float Ref;
    float Kp;
    float Ki;
    float Kd;

    float Measure;
    float Last_Measure;
    float Err;
    float Last_Err;
    float Last_ITerm;

    float Pout;
    float Iout;
    float Dout;
    float ITerm;

    float Output;
    float Last_Output;
    float Last_Dout;

    float MaxOut;
    float IntegralLimit;
    float DeadBand;
    float ControlPeriod;
    float CoefA;         //For Changing Integral
    float CoefB;         //ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC; // RC = 1/omegac
    float Derivative_LPF_RC;

    uint16_t OLS_Order;
    Ordinary_Least_Squares_t OLS;

    uint32_t DWT_CNT;
    float dt;

    FuzzyRule_t *FuzzyRule;

    uint8_t Improve;

    PID_ErrorHandler_t ERRORHandler;

    void (*User_Func1_f)(struct pid_t *pid);
    void (*User_Func2_f)(struct pid_t *pid);
} PID_t;

void PID_Init(
    PID_t *pid,
    float max_out,
    float intergral_limit,
    float deadband,

    float kp,
    float ki,
    float kd,

    float A,
    float B,

    float output_lpf_rc,
    float derivative_lpf_rc,

    uint16_t ols_order,

    uint8_t improve);
float PID_Calculate(PID_t *pid, float measure, float ref);

/*************************** FEEDFORWARD CONTROL *****************************/
typedef __packed struct
{
    float c[3]; // G(s) = 1/(c2s^2 + c1s + c0)

    float Ref;
    float Last_Ref;

    float DeadBand;

    uint32_t DWT_CNT;
    float dt;

    float LPF_RC; // RC = 1/omegac

    float Ref_dot;
    float Ref_ddot;
    float Last_Ref_dot;

    uint16_t Ref_dot_OLS_Order;
    Ordinary_Least_Squares_t Ref_dot_OLS;
    uint16_t Ref_ddot_OLS_Order;
    Ordinary_Least_Squares_t Ref_ddot_OLS;

    float Output;
    float MaxOut;

} Feedforward_t;

void Feedforward_Init(
    Feedforward_t *ffc,
    float max_out,
    float *c,
    float lpf_rc,
    uint16_t ref_dot_ols_order,
    uint16_t ref_ddot_ols_order);

float Feedforward_Calculate(Feedforward_t *ffc, float ref);

/************************* LINEAR DISTURBANCE OBSERVER *************************/
typedef __packed struct
{
    float c[3]; // G(s) = 1/(c2s^2 + c1s + c0)

    float Measure;
    float Last_Measure;

    float u; // system input

    float DeadBand;

    uint32_t DWT_CNT;
    float dt;

    float LPF_RC; // RC = 1/omegac

    float Measure_dot;
    float Measure_ddot;
    float Last_Measure_dot;

    uint16_t Measure_dot_OLS_Order;
    Ordinary_Least_Squares_t Measure_dot_OLS;
    uint16_t Measure_ddot_OLS_Order;
    Ordinary_Least_Squares_t Measure_ddot_OLS;

    float Disturbance;
    float Output;
    float Last_Disturbance;
    float Max_Disturbance;
} LDOB_t;

void LDOB_Init(
    LDOB_t *ldob,
    float max_d,
    float deadband,
    float *c,
    float lpf_rc,
    uint16_t measure_dot_ols_order,
    uint16_t measure_ddot_ols_order);

float LDOB_Calculate(LDOB_t *ldob, float measure, float u);

/*************************** Tracking Differentiator ***************************/
typedef __packed struct
{
    float Input;

    float h0;
    float r;

    float x;
    float dx;
    float ddx;

    float last_dx;
    float last_ddx;

    uint32_t DWT_CNT;
    float dt;
} TD_t;

void TD_Init(TD_t *td, float r, float h0);
float TD_Calculate(TD_t *td, float input);


#endif