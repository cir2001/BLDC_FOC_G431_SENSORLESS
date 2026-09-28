#include "delay.h"
#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////  
//
////////////////////////////////////////////////////////////////////////////////// 
 
static u8  fac_us=0;//us延时倍乘数			   
static u16 fac_ms=0;//ms延时倍乘数,在ucos下,代表每个节拍的ms数

/**
 * @brief  初始化延迟函数
 * @note   SYSTICK的时钟固定为AHB时钟的1/8
 * 170MHz / 8 = 21.25MHz (每微秒 21.25 个脉冲)
 */
void delay_init(u8 SYSCLK)
{
    // 1. 选择外部时钟源 (HCLK/8)
    // SysTick->CTRL 的位 2 为 0 表示外部时钟，1 表示内核时钟
    SysTick->CTRL &= ~(1 << 2); 
    
    // 2. 计算 1us 对应的计数值
    // 170 / 8 = 21.25 -> 这里存 21 (会有极小误差)
    fac_us = SYSCLK / 8;
    
    // 3. 计算 1ms 对应的计数值
    fac_ms = (u16)fac_us * 1000;
}								    

/**
 * @brief  延时 nus 微秒
 * @param  nus: 要延时的微秒数
 * @note   注意: nus * fac_us 不得超过 2^24 (SysTick最大值)
 * 170MHz下，最大延时约 788,915us
 */
void delay_us(u32 nus)
{		
    u32 temp;	    	 
    if (nus == 0) return;
    
    SysTick->LOAD = nus * fac_us; // 时间加载	  		 
    SysTick->VAL = 0x00;          // 清空当前计数值
    SysTick->CTRL |= 0x01;        // 使能计数器开始倒数 	 
    
    do {
        temp = SysTick->CTRL;
    } while ((temp & 0x01) && !(temp & (1 << 16))); // 等待 COUNTFLAG 置 1
    
    SysTick->CTRL &= ~0x01;       // 关闭计数器
    SysTick->VAL = 0X00;          // 清空计数器	 
}

/**
 * @brief  内部调用函数，执行毫秒级单次延时
 * @note   为了处理 SysTick 24位寄存器溢出问题，限定单次延时上限
 */
static void delay_xms(u16 nms)
{	 		  	  
    u32 temp;		   
    SysTick->LOAD = (u32)nms * fac_ms;
    SysTick->VAL = 0x00;           
    SysTick->CTRL |= 0x01;          
    
    do {
        temp = SysTick->CTRL;
    } while ((temp & 0x01) && !(temp & (1 << 16)));
    
    SysTick->CTRL &= ~0x01;       
    SysTick->VAL = 0X00;          	  	    
} 

/**
 * @brief  延时 nms 毫秒
 * @param  nms: 0~65535
 */
void delay_ms(u16 nms)
{	 	 
    // G431 在 170MHz 下，SysTick 单次最多延时约 788ms
    // 我们取 500ms 作为一个循环安全阶梯
    u8 repeat = nms / 500;
    u16 remain = nms % 500;
    
    while (repeat) {
        delay_xms(500);
        repeat--;
    }
    
    if (remain) {
        delay_xms(remain);
    }
}

