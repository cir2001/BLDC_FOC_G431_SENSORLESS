#ifndef __CONTROL_H
#define __CONTROL_H

#include <stm32g4xx.h>
//---------------------------------------------------------
//
//---------------------------------------------------------
void CORDIC_Init(void);
void CORDIC_SinCos(float angle, float *s, float *c);

#endif 



