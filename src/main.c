//*****************************************************************
//  Project Name: BLDC_FOC_G431_SENSORLESS  B-G431B-ESC1开发板无传感器FOC电机控制项目
//	File Name: main.c
//  芯片：STM32G431CUb6
//  version: V0.0
//	Author: Zhou Jian
//	At:	Xi'an China
//*****************************************************************
//	功能：
//		1. BLDC 电机 FOC 控制框架搭建
//		2. TIM1 产生中心对齐 PWM 波形，驱动三相桥式驱动器
//		3. ADC 采样三相电流 与总线电压  
//*****************************************************************	
//=================================================================		
//		date		comment
//	2026-09-28   	初始化
//  
//*****************************************************************
//----------------LED 定义 ---------------------------------------
//--- 开发板自带LED ---
//	LED0	<------->		PB   
//---------------- USART2接口  --------------------------------------- 
//	UART2   115200，1start，8bit，1stop，no parity  虚拟串口向上位机发送
//		USART2_TX			PB3
//		USART2_RX			PB4
//---- TIM1 PWM 输出定义 ----------------------------------------
//		TIM1_CH1		<------->		PA8		(U相PWM)
//		TIM1_CH1N		<------->		PC13	(U相PWM-)           
//		TIM1_CH2		<------->		PA9	    (V相PWM)
//		TIM1_CH2N		<------->		PA12	(V相PWM-)
//		TIM1_CH3		<------->		PA10	(W相PWM)
//		TIM1_CH3N		<------->		PB15	(W相PWM-)
//---- ADC 输入定义 ------------------------------------------------
//		ADC1_IN5		<------->		PA1		(U相    电流采样)Curr_fdbk1_OPAmp+
//		ADC1_IN6		<------->		PA7		(V相    电流采样)Curr_fdbk2_OPAmp+   
//		ADC1_IN7		<------->		PB0		(W相    电流采样)Curr_fdbk3_OPAmp+
//		ADC1_IN10		<------->		PA0		(总线电压采样)
//=========================================================
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
//------------------------------------------

//-----------------------------------------
void Reset_PID_Controllers(void);
void TIM1_Force_Update_Test(u16 arr);
void Check_DMA_Complete(void);
//-----------------------------------------
//-----------------------------------------
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

extern volatile uint8_t  write_bank;    // 当前正在写入哪个缓冲 (0 或 1)
extern volatile uint32_t write_ptr;     // 写入指针
extern volatile uint8_t  bank_ready; // 哪一个缓冲准备好了发送 (0, 1 或 0xFF表示无)
extern volatile uint8_t  is_dma_busy;   // DMA 发送状态

//-----------------------------------------
// --- 新增：全局控制标志位 ---
volatile uint8_t run_foc_flag = 0; // 0: 停止；本步骤不允许进入闭环

volatile uint32_t led_tick = 0;

// 用于主循环打印的调试变量
volatile float debug_id = 0;
volatile float debug_iq = 0;
volatile float debug_Vq = 0;
volatile float debug_Vd = 0;

volatile float debug_iu = 0;
volatile float debug_iv = 0;
volatile float debug_iw = 0;

extern float elec_angle;

//================================================================================
int main(void) {
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));
    // 系统时钟初始化 (170MHz)

    SystemClock_Config(); 
    // 延时函数初始化
    delay_init(170); 

    // USART2 初始化，波特率 921600
    uart2_init(921600); 
    delay_ms(100);

    LED_Init(); // 初始化LED
    // Button_Init(); // 初始化按键
    // 3. CORDIC 硬件加速器初始化 (必须在算法调用前)
    CORDIC_Init();
    delay_ms(100);

    delay_ms(100);

    OPAMP_Init_Registers(); 
    delay_ms(100);
    ADC_Init_Registers();
    delay_ms(100);
    printf("Analog Init Done.\r\n");
    delay_ms(10);

    TIM1_PWM_Init(5666); 
    TIM1->DIER &= ~TIM_DIER_UIE; // 确保 FOC 中断是关着的，防止它干扰校准
    printf("TIM1 Hardware Ready.\r\n");
    delay_ms(10);

    // 第一步：解除编码器依赖，只初始化外设，不开启功率输出。
    // 原 Calibrate_Current_Offset() 内部会开启 MOE，下一步检查采样时再处理。
    Motor_Stop();

    //--- PID 参数初始化 ---
    //--- 位置环 (最外环) ---
    target_pos = 0.0f; // 让电机转到 1 圈的位置
    pid_pos.kp = 450.0f;
    pid_pos.ki = 0.0f;
    pid_pos.output_limit = 50.0f; // 限制最大速度环输出 (rad/s)

    // --- 速度环 (外环) ---
    target_speed = 50.0f;      
    pid_speed.kp = 0.08f;      // 速度环 Kp 通常较小
    pid_speed.ki = 0.2f; 
    pid_speed.output_limit = 50.0f; // 限制最大电流

    // --- 电流环 (内环) ---
    pid_id.kp = 5.0f;   pid_id.ki = 1.0f; 
    pid_id.output_limit = 1500.0f; // 对应 SVPWM 最大电压 
    
    pid_iq.kp = 5.0f;   pid_iq.ki = 1.0f; 
    pid_iq.output_limit = 1500.0f; // 对应 SVPWM 最大电压

    // 暂无可用转子角度，不启动 FOC 中断，也不设置 MOE。
    Motor_Stop();
    printf("[Step1] Encoder removed. PWM output OFF; FOC start disabled.\r\n");
    delay_ms(10);

// -------------------------- 主循环 --------------------------
    while (1) 
    {
        // --- 1. 启动发送逻辑 ---
        // 检查是否有 bank 准备好，且 DMA 此时没活干
        // if (bank_ready != 0xFF && is_dma_busy == 0) {
        //     is_dma_busy = 1; 
        //     uint8_t current_send_bank = bank_ready;
        //     bank_ready = 0xFF; // 释放标志，允许 Timer 准备下一个

        //     DMA1_Channel1->CCR &= ~DMA_CCR_EN;
        //     DMA1->IFCR = DMA_IFCR_CGIF1; // 清除之前的残留标志
            
        //     DMA1_Channel1->CMAR = (uint32_t)DataLog[current_send_bank];
        //     // 关键：字节数 = 采样数 * 通道数 * 4
        //     DMA1_Channel1->CNDTR = SAMPLE_NUM * FRAME_SIZE * 4; 
            
        //     DMA1_Channel1->CCR |= DMA_CCR_EN;
        //     // 确保串口的 DMAT 位是开启的
        //     USART2->CR3 |= USART_CR3_DMAT; 
        // }
//===================================================================
        // --- 2. 检查完成逻辑 ---
        // 在主循环查 DMA 状态，彻底替代中断
        // 只要 DMA 的 TCIF1 标志位置 1，说明搬运完成
        if (is_dma_busy && (DMA1->ISR & DMA_ISR_TCIF1)) {
            DMA1->IFCR = DMA_IFCR_CTCIF1; // 清除完成标志
            // 这里千万不要去动 USART2->CR3，保持 DMAT 开启即可
            is_dma_busy = 0; // 释放锁，允许下一波发送
        }
    //-----------------------------------------------------
        led_tick++;
        if (led_tick >= 100000) // 每500ms翻转一次LED0
        {
            led_tick = 0;
            // LED0_TOGGLE();      // 翻转LED0
        }
     }
}



