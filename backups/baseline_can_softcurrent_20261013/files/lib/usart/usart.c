#include "sys.h"
#include "usart.h"
#include <stdio.h>  
#include <errno.h>
#include "led.h"    
#include <string.h>
#include "timer.h"
#include "fdcan.h"

#define FLASH_SAVE_ADDR  0X08010000 	
u16 FalshSave[32];

uint8_t UART2_DMA_TX_BUF[UART2_DMA_TX_BUF_SIZE];
uint8_t UART2_RX_BUF[UART2_RX_BUF_SIZE]; 

extern float move_speed_deg_per_s;
extern volatile uint8_t run_foc_flag;
extern PID_Controller pid_pos; 

extern uint8_t test_mode_en;    
extern uint8_t test_state;      
extern float test_start_pos;    
extern float actual_pos_rad;    
extern float target_pos;       

// 函数前向声明
float parse_float_manual(char* s);
void uart2_parse_command(char *buf);

void uart2_init(uint32_t bound) {
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
    RCC->AHB1ENR  |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;

    GPIOB->MODER   &= ~(GPIO_MODER_MODE3 | GPIO_MODER_MODE4);
    GPIOB->MODER   |= (2U << GPIO_MODER_MODE3_Pos) | (2U << GPIO_MODER_MODE4_Pos); 

    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED3 | GPIO_OSPEEDR_OSPEED4);
    GPIOB->OSPEEDR |= (3U << GPIO_OSPEEDR_OSPEED3_Pos) | (3U << GPIO_OSPEEDR_OSPEED4_Pos); 
    
    GPIOB->AFR[0] &= ~(0xF << (3 * 4) | 0xF << (4 * 4));
    GPIOB->AFR[0] |=  (7 << (3 * 4) | 7 << (4 * 4));

    USART2->BRR = 170000000 / bound; 

    USART2->CR1 = 0;  
    USART2->CR1 = USART_CR1_TE 
                | USART_CR1_RE
                | USART_CR1_IDLEIE 
                | USART_CR1_UE;   

    USART2->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR; 

    DMAMUX1_Channel0->CCR = 27U; 
    DMAMUX1_Channel1->CCR = 26U; 

    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1_Channel1->CPAR = (uint32_t)&USART2->TDR;
    DMA1_Channel1->CCR = DMA_CCR_DIR | DMA_CCR_MINC;      

    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    DMA1_Channel2->CPAR = (uint32_t)&USART2->RDR;
    DMA1_Channel2->CMAR = (uint32_t)UART2_RX_BUF;
    DMA1_Channel2->CNDTR = UART2_RX_BUF_SIZE;
    DMA1_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | (1U << DMA_CCR_PL_Pos); 
    DMA1_Channel2->CCR |= DMA_CCR_EN;

    NVIC_SetPriority(USART2_IRQn, 1); 
    NVIC_EnableIRQ(USART2_IRQn);
}

void USART2_IRQHandler(void)
{
    if(USART2->ISR & USART_ISR_IDLE)
    {
        // 1. 先停止 DMA 锁存准确计数
        DMA1_Channel2->CCR &= ~DMA_CCR_EN;
        USART2->ICR |= USART_ICR_IDLECF; // 清除空闲中断标志

        uint16_t rx_len = UART2_RX_BUF_SIZE - DMA1_Channel2->CNDTR;

        if (rx_len > 0U && rx_len < UART2_RX_BUF_SIZE)
        {
            UART2_RX_BUF[rx_len] = '\0'; 
            uart2_parse_command((char*)UART2_RX_BUF);
        }

        // 重置 DMA 接收计数器
        DMA1_Channel2->CNDTR = UART2_RX_BUF_SIZE;
        DMA1_Channel2->CCR |= DMA_CCR_EN;
    }

    if (USART2->ISR & (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE)) {
        USART2->ICR |= (USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF);
    }
}

void uart2_dma_send(uint16_t len)
{
    while(DMA1_Channel1->CNDTR != 0); 
    
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;          
    DMA1_Channel1->CMAR = (uint32_t)UART2_DMA_TX_BUF; 
    DMA1_Channel1->CNDTR = len;                 
    
    DMA1->IFCR |= DMA_IFCR_CTCIF1;              
    DMA1_Channel1->CCR |= DMA_CCR_EN;           
}

void uart2_send_byte(u8 data)
{
    while (!(USART2->ISR & USART_ISR_TXE)); 
    USART2->TDR = data;
}

int _write(int file, char *ptr, int len)
{
    if (vofa_stream_active) return len; 

    if (USART2->ISR & (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE)) {
        USART2->ICR |= (USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF);
    }

    uint32_t timeout = 50000; 
    while(DMA1_Channel1->CNDTR != 0 && timeout--); 

    if(timeout == 0) {
        DMA1_Channel1->CCR &= ~DMA_CCR_EN;
        DMA1_Channel1->CNDTR = 0; 
        return 0; 
    }

    int send_len = (len < 256) ? len : 256;
    memcpy(UART2_DMA_TX_BUF, ptr, send_len);
    
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1_Channel1->CMAR = (uint32_t)UART2_DMA_TX_BUF;
    DMA1_Channel1->CNDTR = send_len;
    DMA1->IFCR |= DMA_IFCR_CTCIF1;
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    return len;
}

void uart2_parse_command(char *buf) {
    if (buf == NULL || buf[0] == '\0') return;

    char cmd = buf[0];
    float val = parse_float_manual(&buf[1]); 

    switch (cmd) {
        case 'm':
        case 'M':
            #if (CAN_PHASE1_PASSIVE_TEST == 1U)
            if (val > 0.5f) {
                printf("[Cmd Ignored] In CAN Phase-1 Passive Mode: Hardware MOE locked to 0!\r\n");
            } else {
                Motor_Test_RequestStop();
                printf("[Cmd] Motor Stopped\r\n");
            }
            #else
            if (val > 200.0f) {
                Motor_Set_Target_RPM(val);
                printf("[Cmd] Target RPM updated -> %.1f RPM\r\n", (double)val);
            } else if (val > 0.5f) {
                Motor_Set_Target_RPM(500.0f);
                Motor_Test_RequestStart();
                printf("[Cmd] Motor Started (Base 500 RPM)\r\n");
            } else {
                Motor_Test_RequestStop();
                printf("[Cmd] Motor Stopped\r\n");
            }
            #endif
            break;
        case 'o':
        case 'O':
            if (val > 0.5f && val < 1.5f) Motor_Test_RequestOpen();
            break;

        case 'h':
        case 'H':
            if (val > 0.5f && val < 1.5f) Motor_Test_RequestAngle();
            break;

        case 'a':
        case 'A':
            if (val > 0.5f && val < 1.5f) Motor_Test_RequestStaticAlign();
            break;

        case 'c':
        case 'C':
            if (val < 0.5f) FDCAN1_RequestDiag();
            break;

        case 'd':
        case 'D':
            if (val < 0.5f) Motor_Vofa_RequestText();
            break;

        // 【关键修复】开放 s / S 指令，用于多转速阶跃测试
        case 's':
        case 'S':
            if (val >= 300.0f && val <= 800.0f) {
                Motor_Set_Target_RPM(val);
                printf("[Cmd] Target RPM -> %.1f RPM (Ratio=%.3f)\r\n", (double)val, (double)(val / 500.0f));
            } else {
                printf("[Cmd] Target RPM out of range (300 ~ 800 RPM): %.1f\r\n", (double)val);
            }
            break;

        // 遥测快照查询
        case 't':
        case 'T': {
            Actuator_Telemetry_t telem;
            Motor_Get_Telemetry(&telem);
            float err_pct = (telem.target_rpm > 1.0f) ? 
                            ((telem.actual_rpm - telem.target_rpm) / telem.target_rpm * 100.0f) : 0.0f;
            printf("[Telemetry] Target=%.1f RPM, Actual=%.1f RPM (Err=%+.2f%%), Iq=%.3fA, Base_Vd=%.4f, Trim=%.5f, Stat=%u\r\n",
                   (double)telem.target_rpm, (double)telem.actual_rpm, (double)err_pct,
                   (double)telem.i_q, (double)telem.base_vd, (double)telem.v_trim, (unsigned int)telem.status);
            break;
        }

        default:
            printf("?? Unknown Cmd: %c (0x%02X)\r\n", cmd, cmd);
            break;
    }
}

float parse_float_manual(char* s) {
    float res = 0.0f, fact = 1.0f;
    while (*s && (*s < '0' || *s > '9') && *s != '-') s++;
    
    if (*s == '-') { s++; fact = -1.0f; }
    
    for (int point_seen = 0; *s; s++) {
        if (*s == '.') { point_seen = 1; continue; }
        int d = *s - '0';
        if (d >= 0 && d <= 9) {
            if (point_seen) fact /= 10.0f;
            res = res * 10.0f + (float)d;
        } else break; 
    }
    return res * fact;
}