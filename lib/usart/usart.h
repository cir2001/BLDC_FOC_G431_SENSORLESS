#ifndef __USART_H
#define __USART_H 
#include "sys.h"	
//---------------------------------------------------------
//
//---------------------------------------------------------
#define USART_TRANS_LEN     	200  	//定义最大接收字节数 200
#define UART2_DMA_TX_BUF_SIZE   256
#define UART2_RX_BUF_SIZE       256

void uart1_init(u32 bound);
void uart1_send_byte(u8 data);

void uart2_init(u32 bound);
void uart2_send_byte(u8 data);

void uart2_parse_command(char *buf);

float parse_float_manual(char* s);


#endif	   

