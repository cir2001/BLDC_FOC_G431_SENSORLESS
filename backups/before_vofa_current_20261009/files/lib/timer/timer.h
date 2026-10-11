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
#define POLE_PAIRS         7 // 14 极 = 7 极对
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
void Motor_Stop(void);
// 短时电压开环：上电待机，故障锁定至复位。
typedef enum {
    MOTOR_TEST_IDLE = 0,
    MOTOR_TEST_PREFLIGHT,
    MOTOR_TEST_ALIGN,
    MOTOR_TEST_RAMP,
    MOTOR_TEST_RUN,
    MOTOR_TEST_DONE,
    MOTOR_TEST_STOPPED,
    MOTOR_TEST_ADC_FAULT,
    MOTOR_TEST_ADC_RANGE,
    MOTOR_TEST_OVERCURRENT
} MotorTestStatus;
extern volatile MotorTestStatus motor_test_status;
void Motor_Test_SetReady(uint8_t ready);
void Motor_Test_RequestStart(void);
void Motor_Test_Service(void); // 仅主循环调用，启动准备和日志均不在控制中断执行。

void SVPWM_Output_Standard(float Valpha, float Vbeta);
float PID_Calc_Current(PID_Controller* pid, float target, float current);
float PID_Calc_Speed(PID_Controller* pid, float target, float current);
float PID_Calc_Pos(PID_Controller* pid, float target, float current);
void Motor_OpenLoop_Tick(void);


#endif

