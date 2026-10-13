//*****************************************************************
//  Project Name: BLDC_FOC_G431_SENSORLESS
//  File Name: main.c
//  芯片：STM32G431CUb6
//*****************************************************************
#include <stm32g4xx.h>
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "timer.h"
#include "led.h"
#include <stdio.h>
#include "button.h"
#include "opamp.h"
#include "control.h"
#include "adc_foc.h"
#include <stdlib.h>
#include <math.h>
#include "fdcan.h"

void Reset_PID_Controllers(void);

PID_Controller pid_id;
PID_Controller pid_iq;
PID_Controller pid_speed;
PID_Controller pid_pos;

extern float target_speed;
extern float actual_speed_filt;
extern float target_iq;
extern float target_id;  
extern float target_pos;

extern volatile float Vd;
extern volatile float Vq;

extern VofaData_t DataLog[2][SAMPLE_NUM][FRAME_SIZE];

volatile uint8_t run_foc_flag = 0;
volatile uint32_t led_tick = 0;

volatile float debug_id = 0;
volatile float debug_iq = 0;
volatile float debug_Vq = 0;
volatile float debug_Vd = 0;

volatile float debug_iu = 0;
volatile float debug_iv = 0;
volatile float debug_iw = 0;

extern float elec_angle;

int main(void) {
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));

    SystemClock_Config(); 
    delay_init(170); 

    // 开启 DWT 硬件周期计数器
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    uart2_init(921600); 
    delay_ms(100);

    LED_Init(); 
    CORDIC_Init();
    delay_ms(100);

    OPAMP_Init_Registers(); 
    delay_ms(100);
    ADC_Init_Registers();
    delay_ms(100);
    printf("Analog Init Done.\r\n");
    delay_ms(10);

    // 初始化 FDCAN1 (PA11=RX, PB9=TX, PC11=CAN_SHDN 拉低使能)
    FDCAN1_Init();
    printf("FDCAN1 Hardware Ready (PA11/PB9/PC11, Classic CAN 1Mbps, Node=ESC1).\r\n");
    delay_ms(10);

    TIM1_PWM_Init(5666); 
    TIM1->DIER &= ~TIM_DIER_UIE; 
    printf("TIM1 Hardware Ready.\r\n");
    delay_ms(10);

    Motor_Stop();
    uint8_t calib_ok = Calibrate_Current_Offset();
    uint8_t zero_collected = 0U;
    if (calib_ok != 0U) {
        zero_collected = Check_Current_Zero();
    }
    printf("[Zero test] collected=%u (not a noise pass/fail).\r\n", (unsigned int)zero_collected);
    delay_ms(20);

    Motor_Stop();
    printf("[Sample test] calib=%u, MOE=%u, UIE=%u\r\n",
        (unsigned int)calib_ok,
        (unsigned int)((TIM1->BDTR & TIM_BDTR_MOE) != 0U),
        (unsigned int)((TIM1->DIER & TIM_DIER_UIE) != 0U));
    delay_ms(20);

    // --- PID 参数初始化 ---
    target_pos = 0.0f;
    pid_pos.kp = 450.0f;
    pid_pos.ki = 0.0f;
    pid_pos.output_limit = 50.0f;

    target_speed = 50.0f;      
    pid_speed.kp = 0.08f;      
    pid_speed.ki = 0.2f;  
    pid_speed.output_limit = 50.0f;

    // --- 电流环参数初始化 ---
    pid_id.kp = 0.008f;   
    pid_id.ki = 0.8f;   
    pid_id.output_limit = 0.010f; 
    pid_id.integral = 0.0f;
    
    pid_iq.kp = 0.008f;  
    pid_iq.ki = 0.8f; 
    pid_iq.output_limit = 0.010f; 
    pid_iq.integral = 0.0f;

    Motor_Stop();

    // 阶段 1：强制 Ready=0，上锁保护
    #if (CAN_PHASE1_PASSIVE_TEST == 1U)
    Motor_Test_SetReady(0U);
    printf("[Phase-1 Active] PASSIVE TEST MODE: Hardware MOE locked to 0! Listening 0x201, Sending 0x301.\r\n");
    #else
    Motor_Test_SetReady(calib_ok && zero_collected);
    printf("[Actuator Ready] CAN-FD Online & UART Active. M1=Start, M0=Stop.\r\n");
    #endif
    delay_ms(10);

    uint32_t last_led_cycles = DWT->CYCCNT;

    while (1) 
    {
        uint32_t now_cyc = DWT->CYCCNT;

        // 1. 优先以当前精确 cycle 执行 FDCAN 轮询
        FDCAN1_Poll(now_cyc);

        // 2. 执行电机诊断与串口数据服务
        Motor_Test_Service();

        // 3. 500ms LED 状态心跳指示 (500ms = 85,000,000 cycles)
        if ((uint32_t)(now_cyc - last_led_cycles) >= (500U * 170000U)) {
            last_led_cycles = now_cyc;
            LED0_TOGGLE();
        }
    }
}