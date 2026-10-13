#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

#define PWM_ARR             5666.0f
#define PWM_ARR_INT         5666
#define ENCODER_MASK        0x3FFF

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define SQRT3 0.577350269f

#define ENCODER_RESOLUTION 16384
#define POLE_PAIRS          7

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float output_limit;
    float last_error;
} PID_Controller;

// 执行器对外遥测状态结构体 (供 FDCAN 1kHz 异步打包)
typedef struct {
    float actual_rpm;       // 当前实测机械转速 (RPM)
    float speed_ratio;      // 当前归一化转速比 (基于 500 RPM)
    float target_rpm;       // 当前目标转速 (RPM)
    float i_u;              // U 相相电流 (A)
    float i_d;              // d 轴直轴电流 (A)
    float i_q;              // q 轴交轴电流 (A)
    float base_vd;          // 当前端电压基座调制比
    float v_trim;           // 速度外环输出微调量
    uint8_t status;         // 当前运行状态码
    uint8_t fault_code;     // 故障码
} Actuator_Telemetry_t;

// VOFA+ 协议: 5个物理数据通道 + 1个 JustFloat 帧尾 (0x7F800000U)
// CH0: Iu | CH1: actual_rpm | CH2: target_rpm | CH3: v_trim | CH4: Iq_rot
#define CH_COUNT   5
#define SAMPLE_NUM 300     
#define FRAME_SIZE (CH_COUNT + 1)
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
    MOTOR_TEST_OVERSPEED
} MotorTestStatus;

extern volatile MotorTestStatus motor_test_status;
void Motor_Test_SetReady(uint8_t ready);
void Motor_Test_RequestStart(void);
void Motor_Test_RequestStop(void);
void Motor_Set_Target_RPM(float rpm);
float Motor_Get_Command_Target_RPM(void);
void Motor_Get_Telemetry(Actuator_Telemetry_t* telem);

// 兼容旧指令接口声明
void Motor_Test_RequestOpen(void);
void Motor_Test_RequestAngle(void);
void Motor_Test_RequestComparison(unsigned rpm, unsigned with_p);
void Motor_Test_RequestStaticAlign(void);

extern volatile uint8_t vofa_stream_active;
void Motor_Vofa_RequestText(void); 
void Motor_Test_Service(void); 

void SVPWM_Output_Standard(float Valpha, float Vbeta);
float PID_Calc_Current(PID_Controller* pid, float target, float current);

#endif