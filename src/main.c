//*****************************************************************
//  Project Name: BLDC_FOC_G431_SENSORLESS  B-G431B-ESC1开发板无传感器FOC电机控制项目
//	File Name: main.c
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

    // --- main.c: 电流环参数初始化 ---
    pid_id.kp = 0.008f;   
    pid_id.ki = 0.8f;    
    pid_id.output_limit = 0.08f; // 放宽至 0.08f，确保允许输出 0.058f
    pid_id.integral = 0.0f;
    
    pid_iq.kp = 0.008f;   
    pid_iq.ki = 0.8f; 
    pid_iq.output_limit = 0.08f; // 放宽至 0.08f
    pid_iq.integral = 0.0f;

    Motor_Stop();
    Motor_Test_SetReady(calib_ok && zero_collected);
    printf("[OpenLoop] ready=%u; O1=open+shadow H1=angle-only M1=current-loop M0=stop; fault requires reset.\r\n",
           (unsigned int)(calib_ok && zero_collected));
    delay_ms(10);
    printf("[VOFA] JustFloat 921600: CH0=Iu CH1=Iv CH2=speed_ratio CH3=theta_open CH4=theta_smo.\r\n");
    delay_ms(10);
    printf("[StaticAlign] A1=fixed angle, Id=0.3A PI, ramp0.3s+hold1s, auto OFF; M0=abort.\r\n");
    delay_ms(10);

    while (1) 
    {
        Motor_Test_Service();
        delay_ms(10);

        led_tick++;
        if (led_tick >= 50) 
        {
            led_tick = 0;
            LED0_TOGGLE();
        }
    }
}