
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <string.h>
typedef uint16_t u16;
typedef struct {uint32_t SR, CR1, CR2, DIER, BDTR, ARR, PSC, RCR, CNT, CCR1, CCR2, CCR3, CCR4, EGR;} TimerRegs;
typedef struct {uint32_t ISR, CR, JDR1, JDR2;} AdcRegs;
static TimerRegs timer_regs;
static AdcRegs adc1_regs, adc2_regs;
#define TIM1 (&timer_regs)
#define ADC1 (&adc1_regs)
#define ADC2 (&adc2_regs)
#define TIM_SR_UIF 1U
#define TIM_DIER_UIE 1U
#define TIM_BDTR_MOE (1U<<15)
#define TIM_CR1_CEN 1U
#define TIM_CR1_DIR (1U<<4)
#define TIM_CR1_CMS (3U<<5)
#define TIM_CR1_CMS_Pos 5U
#define TIM_EGR_UG 1U
#define ADC_ISR_JEOS (1U<<6)
#define ADC_ISR_JEOC (1U<<5)
#define ADC_CR_ADEN 1U
#define ADC_CR_JADSTART (1U<<3)
#define TIM1_UP_TIM16_IRQn 0
#define LED0_TOGGLE() ((void)0)
static uint32_t __get_PRIMASK(void) {return 0;}
static void __disable_irq(void) {}
static void __set_PRIMASK(uint32_t p) {(void)p;}
static void NVIC_ClearPendingIRQ(int n) {(void)n;}
static void CORDIC_SinCos(float a, float *s, float *c) {*s=sinf(a); *c=cosf(a);}

typedef struct {uint32_t CCR,CNDTR,CMAR;} DmaChannel;
typedef struct {uint32_t ISR,IFCR;} DmaRegs;
typedef struct {uint32_t ISR;} UartRegs;
static DmaChannel dma_ch; static DmaRegs dma; static UartRegs uart;
#define DMA1_Channel1 (&dma_ch)
#define DMA1 (&dma)
#define USART2 (&uart)
#define DMA_CCR_EN 1U
#define DMA_ISR_TCIF1 2U
#define DMA_ISR_TEIF1 8U
#define DMA_IFCR_CGIF1 15U
#define USART_ISR_TC 64U
static void __DMB(void) {}

typedef struct {uint32_t CTRL, CYCCNT;} MockDwt;
typedef struct {uint32_t DEMCR;} MockCoreDebug;
static MockDwt mock_dwt;
static MockCoreDebug mock_core_debug;
#define DWT (&mock_dwt)
#define CoreDebug (&mock_core_debug)
#define CoreDebug_DEMCR_TRCENA_Msk (1U<<24)
#define DWT_CTRL_CYCCNTENA_Msk 1U
static void delay_ms(unsigned ms) {(void)ms;}
