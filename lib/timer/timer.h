#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"
//---------------------------------------------------------------------
//
//--------------------------------------------------------------------
// 宏定义：基于 ARR = 5666
#define PWM_ARR             5666.0f
#define PWM_ARR_INT         5666
#define ENCODER_MASK       0x3FFF

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define SQRT3 0.577350269f

#define ENCODER_RESOLUTION 16384
#define POLE_PAIRS         7 //10
//---------------------------------------------
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float output_limit;
    float last_error;
} PID_Controller;
//-------------------------------------------
#define CH_COUNT  3       // 记录 3 个通道（如：Iq, Id, Speed）
#define SAMPLE_NUM 300    // 记录 500 组数据
#define FRAME_SIZE (CH_COUNT + 1) // 加上 1 个结束符
typedef union {
    float f;
    uint32_t u;
} VofaData_t;
//--------------------------------------------
void TIM1_PWM_Init(u16 arr);
void SVPWM_Output_Standard(float Valpha, float Vbeta);
float PID_Calc_Current(PID_Controller* pid, float target, float current);
float PID_Calc_Speed(PID_Controller* pid, float target, float current);
float PID_Calc_Pos(PID_Controller* pid, float target, float current);
void Motor_OpenLoop_Tick(void);


#endif

