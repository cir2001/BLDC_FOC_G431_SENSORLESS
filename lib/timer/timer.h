#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

#define PWM_ARR             5666.0f
#define PWM_ARR_INT         5666
#define ENCODER_MASK       0x3FFF

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define SQRT3 0.577350269f

#define ENCODER_RESOLUTION 16384
#define POLE_PAIRS         7 // 14 极 = 7 极对

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float output_limit;
    float last_error;
} PID_Controller;

#define CH_COUNT  5       // Iu, Iv, speed_ratio, theta_open, theta_smo
#define SAMPLE_NUM 300    // 记录 300 组数据
#define FRAME_SIZE (CH_COUNT + 1) // 加上 1 个结束符
typedef union {
    float f;
    uint32_t u;
} VofaData_t;

void TIM1_PWM_Init(u16 arr);
void Motor_Stop(void);

typedef enum {
    MOTOR_TEST_IDLE = 0,
    MOTOR_TEST_PREFLIGHT,
    MOTOR_TEST_PREZERO,
    MOTOR_TEST_ALIGN,
    MOTOR_TEST_RAMP,
    MOTOR_TEST_RUN,
    MOTOR_TEST_HANDOFF,   
    MOTOR_TEST_CLOSED,    
    MOTOR_TEST_POSTZERO,
    MOTOR_TEST_DONE,
    MOTOR_TEST_STOPPED,
    MOTOR_TEST_ADC_FAULT,
    MOTOR_TEST_ADC_RANGE,
    MOTOR_TEST_OVERCURRENT,
    MOTOR_TEST_OVERSPEED  // 独立估算速度越界保护 (>1.25x 持续 0.1s)
} MotorTestStatus;

extern volatile MotorTestStatus motor_test_status;
void Motor_Test_SetReady(uint8_t ready);
void Motor_Test_RequestStart(void);
void Motor_Test_RequestComparison(unsigned rpm, unsigned with_p);
void Motor_Test_RequestOpen(void);  
void Motor_Test_RequestAngle(void); 
void Motor_Test_RequestStaticAlign(void); 
void Motor_Test_RequestStop(void); 
extern volatile uint8_t vofa_stream_active;
void Motor_Vofa_RequestText(void); 
void Motor_Test_Service(void); 

void SVPWM_Output_Standard(float Valpha, float Vbeta);
float PID_Calc_Current(PID_Controller* pid, float target, float current);
float PID_Calc_Speed(PID_Controller* pid, float target, float current);
float PID_Calc_Pos(PID_Controller* pid, float target, float current);
void Motor_OpenLoop_Tick(void);

#endif