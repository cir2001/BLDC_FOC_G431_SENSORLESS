#include "sys.h"
#include "usart.h"
#include <stdio.h>  
#include <errno.h>
#include "led.h"    
#include <string.h>
#include "timer.h"
//-------------------------------------------------------------------------------
//设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000)
#define FLASH_SAVE_ADDR  0X08010000 	
u16 FalshSave[32];
//--------------------------------------------------------
uint8_t UART2_DMA_TX_BUF[UART2_DMA_TX_BUF_SIZE];
uint8_t UART2_RX_BUF[UART2_RX_BUF_SIZE]; // 接收缓冲区

// 引用外部控制变量
extern float move_speed_deg_per_s;
extern uint8_t run_foc_flag;
extern PID_Controller pid_pos; // 假设你的位置环 PID 结构体名为 pid_pos
// --- 往复测试变量 ---
extern uint8_t test_mode_en;    // 测试模式使能
extern uint8_t test_state;      // 0: 停止, 1: 正转中, 2: 反转中
extern float test_start_pos;    // 起始位置
extern float actual_pos_rad;            // 实际累加位置 (rad, 多圈)
extern float target_pos;       // 目标位置 (rad)
//----------------------------------------------------------
//
//----------------------------------------------------------
void uart2_init(uint32_t bound) {
    // 1. 时钟使能 (G4 必须开启 DMAMUX)
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
    RCC->AHB1ENR  |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;

    // 2. GPIO 配置 PB3=TX, PB4=RX (直接操作寄存器最稳)
    GPIOB->MODER   &= ~(GPIO_MODER_MODE3 | GPIO_MODER_MODE4);
    GPIOB->MODER   |= (2U << GPIO_MODER_MODE3_Pos) | (2U << GPIO_MODER_MODE4_Pos); // AF模式

    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED3 | GPIO_OSPEEDR_OSPEED4);
    GPIOB->OSPEEDR |= (3U << GPIO_OSPEEDR_OSPEED3_Pos) | (3U << GPIO_OSPEEDR_OSPEED4_Pos); // 高速模式
    
    // PB3 -> AF7, PB4 -> AF7 (AFRL 寄存器)
    GPIOB->AFR[0] &= ~(0xF << (3 * 4) | 0xF << (4 * 4));
    GPIOB->AFR[0] |=  (7 << (3 * 4) | 7 << (4 * 4));

    // 3. USART2 配置
    // 注意：请务必确认系统频率！如果是 170MHz，用 170000000，如果是 16MHz，用 16000000
    USART2->BRR = 170000000 / bound; 

    USART2->CR1 = 0;  // 先清
    USART2->CR1 = USART_CR1_TE 
                | USART_CR1_RE
                | USART_CR1_IDLEIE // 开启空闲中断
                | USART_CR1_UE;   // UE 最后使能

    USART2->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR; // 开启 DMA 发送和接收请求

    // 4. DMAMUX 配置 (USART2_TX ID=27)
    DMAMUX1_Channel0->CCR = 27U; 
    DMAMUX1_Channel1->CCR = 26U; // DMA1_CH2 用于 RX

    // 5. DMA1_Channel1 配置
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1_Channel1->CPAR = (uint32_t)&USART2->TDR;
    
    DMA1_Channel1->CCR = DMA_CCR_DIR        // 1 = 内存 → 外设
                        | DMA_CCR_MINC;      // 内存地址递增


    // 6. 【新增】DMA1_Channel2 (RX) 配置
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    DMA1_Channel2->CPAR = (uint32_t)&USART2->RDR;
    DMA1_Channel2->CMAR = (uint32_t)UART2_RX_BUF;
    DMA1_Channel2->CNDTR = UART2_RX_BUF_SIZE;
    // 模式：循环模式(CIRC), 内存递增(MINC), 中等优先级
    DMA1_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | (1U << DMA_CCR_PL_Pos); 
    DMA1_Channel2->CCR |= DMA_CCR_EN;

    // 7. NVIC 配置
    NVIC_SetPriority(USART2_IRQn, 1); 
    NVIC_EnableIRQ(USART2_IRQn);
    // NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));  // 抢占优先级 2
    // NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

//----------------------------------------------------------
// USART2 中断服务程序
//----------------------------------------------------------
void USART2_IRQHandler(void)
{
    // 1. 检查是否为空闲中断 (IDLE)
    if(USART2->ISR & USART_ISR_IDLE)
    {
        USART2->ICR |= USART_ICR_IDLECF; // 清除空闲中断标志

        // 1. 停止 DMA 以读取准确的 CNDTR
        DMA1_Channel2->CCR &= ~DMA_CCR_EN;

        // 2. 计算长度
        uint16_t rx_len = UART2_RX_BUF_SIZE - DMA1_Channel2->CNDTR;

        if(rx_len > 0)
        {
            // 3. 强制在数据末尾加结束符，确保 atof 停止
            UART2_RX_BUF[rx_len] = '\0'; 
            
            // 调试用：先看看收到的到底是什么
            // printf("Raw Rx: %s\r\n", UART2_RX_BUF); 

            uart2_parse_command((char*)UART2_RX_BUF);
        }

        // 4. 【关键】重置 DMA 计数器，确保下次数据从 buf[0] 开始
        DMA1_Channel2->CNDTR = UART2_RX_BUF_SIZE;
        DMA1_Channel2->CCR |= DMA_CCR_EN;
    }

    // 清除错误标志（如 ORE），防止串口死掉
    if (USART2->ISR & (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE)) {
        USART2->ICR |= (USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF);
    }
}
//-----------------------------------------------------------
//
//----------------------------------------------------------
void uart2_dma_send(uint16_t len)
{
    // 等待之前的 DMA 发送完成（如果上次没发完）
    while(DMA1_Channel1->CNDTR != 0); 
    
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;          // 临时关闭通道
    DMA1_Channel1->CMAR = (uint32_t)UART2_DMA_TX_BUF; // 设置内存缓冲区地址
    DMA1_Channel1->CNDTR = len;                 // 设置发送长度
    
    DMA1->IFCR |= DMA_IFCR_CTCIF1;              // 清除通道1完成标志
    DMA1_Channel1->CCR |= DMA_CCR_EN;           // 重新开启 DMA
}
//--------------------------------------------------------------------------
// @brief  串口1发送一个字节
//---------------------------------------------------------------------------
void uart2_send_byte(u8 data)
{
    while (!(USART2->ISR & USART_ISR_TXE)); 
    USART2->TDR = data;
}
// --------------------------------------------------------------------------
// @brief  重定向 printf 到串口 2
// --------------------------------------------------------------------------
int _write(int file, char *ptr, int len)
{
    // 1. [关键] 检查并强制清除串口硬件错误标志
    // 如果不清除 ORE(过载)、NE(噪声)、FE(帧错误)，串口会彻底罢工
    if (USART2->ISR & (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE)) {
        USART2->ICR |= (USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF);
    }

    // 2. [关键] 增加超时保护，防止 DMA 计数器停滞导致死循环
    uint32_t timeout = 50000; 
    while(DMA1_Channel1->CNDTR != 0 && timeout--); 

    if(timeout == 0) {
        // 如果 DMA 卡住了，强行关闭通道并重置，放弃本次打印
        DMA1_Channel1->CCR &= ~DMA_CCR_EN;
        DMA1_Channel1->CNDTR = 0; 
        return 0; 
    }

    // 3. 正常 DMA 拷贝与发送
    int send_len = (len < 256) ? len : 256;
    memcpy(UART2_DMA_TX_BUF, ptr, send_len);
    
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1_Channel1->CMAR = (uint32_t)UART2_DMA_TX_BUF;
    DMA1_Channel1->CNDTR = send_len;
    DMA1->IFCR |= DMA_IFCR_CTCIF1;
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    return len;
}

/**
 * @brief 指令解析器
 * 格式示例：S60 (速度60), P1.5 (Kp设为1.5), M1 (启动), M0 (停止)
 */
void uart2_parse_command(char *buf) {
    char cmd = buf[0];

    float val = parse_float_manual(&buf[1]); // 使用手工版

    switch (cmd) {
        case 'S':
            move_speed_deg_per_s = val;
            printf(">> Target Speed: %.2f deg/s\r\n", val);
            break;
        case 'P':
            pid_pos.kp = val;
            printf(">> Pos_Kp: %.3f\r\n", val);
            break;
        case 'M':
            run_foc_flag = (val > 0.5f) ? 1 : 0;
            printf(">> Motor %s\r\n", run_foc_flag ? "ON" : "OFF");
            break;
        case 'T': // Test Mode: T1 开启, T0 关闭
            if (val > 0.5f) {
                test_mode_en = 1;
                test_state = 1;           // 从正转开始
                test_start_pos = actual_pos_rad; // 以当前位置为基准点
                target_pos = actual_pos_rad;
                printf(">> Test Mode START: 5 turns FWD/BWD\r\n");
            } else {
                test_mode_en = 0;
                test_state = 0;
                printf(">> Test Mode STOP\r\n");
            }
            break;
        default:
            // 这里可以打印出具体的字符 ASCII 码，看是否有隐藏字符
            printf("?? Unknown Cmd: %c (0x%02X)\r\n", cmd, cmd);
            break;
    }
}

// 放在 uart.c 中，替代 atof
float parse_float_manual(char* s) {
    float res = 0.0f, fact = 1.0f;
    // 跳过非数字字符，直到找到数字或负号
    while (*s && (*s < '0' || *s > '9') && *s != '-') s++;
    
    if (*s == '-') { s++; fact = -1.0f; }
    
    // 解析整数部分
    for (int point_seen = 0; *s; s++) {
        if (*s == '.') { point_seen = 1; continue; }
        int d = *s - '0';
        if (d >= 0 && d <= 9) {
            if (point_seen) fact /= 10.0f;
            res = res * 10.0f + (float)d;
        } else break; // 遇到空格或换行直接退出
    }
    return res * fact;
}


