#!/usr/bin/env python3
"""Exercise actual timer.c with ideal trig/register mocks, not a motor model."""
from pathlib import Path
import subprocess
import tempfile
root = Path(__file__).resolve().parents[2]
here = Path(__file__).resolve().parent
s = (root/'lib/timer/timer.c').read_text()
a=s.index('void TIM1_PWM_Init(u16 arr)'); b=s.index('void SVPWM_Output_Standard(', a)
s=s[:a]+s[b:]
s='\n'.join(x for x in s.splitlines() if not x.startswith('#include'))
h=(root/'lib/timer/timer.h').read_text().replace('#include "sys.h"','')
helpers=(here/'zero_helpers.c').read_text().split('static void fresh(')[0]
cases=r'''
static void tick(void) {
 ADC1->ISR=ADC2->ISR=ADC_ISR_JEOS;
 ADC1->JDR1=ADC2->JDR1=ADC2->JDR2=2500;
 TIM1->SR=TIM_SR_UIF; TIM1_UP_TIM16_IRQHandler();
}
static void setup(unsigned mode) {
 motor_test_status=MOTOR_TEST_IDLE; run_foc_flag=0; motor_fault_latched=0;
 motor_start_requested=0; motor_test_ready=1;
 memset((void*)vofa_bank_state,0,sizeof(vofa_bank_state)); vofa_tx_bank=0xff;
 DMA1_Channel1->CNDTR=0;
 TIM1->ARR=PWM_ARR_INT; TIM1->PSC=0; TIM1->RCR=1;
 ADC1->CR=ADC2->CR=ADC_CR_ADEN|ADC_CR_JADSTART;
 Motor_Test_RequestMode(mode); assert(Motor_OpenLoop_Start());
 pid_id=(PID_Controller){.kp=.008f,.ki=.8f,.output_limit=.05f}; pid_iq=pid_id;
 motor_test_status=MOTOR_TEST_RUN;
 open_loop_ticks=OPEN_LOOP_ALIGN_TICKS+OPEN_LOOP_RAMP_TICKS;
 handoff_run_ticks=HANDOFF_WAIT_TICKS;
}
static void locked_tick(void) {
 float we=OPEN_LOOP_TARGET_RPM*(6.2831853f/60.0f)*POLE_PAIRS;
 smo.pll_speed=we;
 smo.angle_est=open_loop_angle+we*control_dt-3.14159265f;
 if(smo.angle_est<0) smo.angle_est+=6.2831853f;
 smo.e_alpha_filt=.05f; smo.e_beta_filt=0;
 tick();
}
int main(void) {
 // Voltage reconstruction from quantized PWM must match SMO scaling.
 for(int i=0;i<100;i++) {
  float a=i*.062831853f, va=.045f*cosf(a), vb=.045f*sinf(a);
  SVPWM_Output_Standard(va,vb);
  float da=(float)TIM1->CCR1/PWM_ARR, db=(float)TIM1->CCR2/PWM_ARR, dc=(float)TIM1->CCR3/PWM_ARR;
  assert(fabsf((da-(da+db+dc)/3)*V_BUS-va*SVPWM_VOLTAGE_SCALE*V_BUS)<.003f);
  assert(fabsf((db-dc)*.57735027f*V_BUS-vb*SVPWM_VOLTAGE_SCALE*V_BUS)<.003f);
 }
 setup(TEST_OPEN);
 for(unsigned i=0;i<LOCK_CONFIRM_TICKS+5;i++) locked_tick();
 assert(motor_test_status==MOTOR_TEST_RUN && !current_loop_started);
 setup(TEST_ANGLE);
 for(unsigned i=0;i<LOCK_CONFIRM_TICKS-1;i++) locked_tick();
 assert(motor_test_status==MOTOR_TEST_RUN);
 locked_tick(); assert(motor_test_status==MOTOR_TEST_HANDOFF);
 handoff_trans_ticks=HANDOFF_RAMP_TICKS-1; tick();
 assert(motor_test_status==MOTOR_TEST_CLOSED && !current_loop_started);
 tick(); assert(!current_loop_started && Vd==OPEN_LOOP_MAX_MOD && Vq==0);
 setup(TEST_FULL);
 motor_test_status=MOTOR_TEST_HANDOFF; handoff_trans_ticks=HANDOFF_RAMP_TICKS-1;
 handoff_delta_angle=.2f; tick();
 assert(motor_test_status==MOTOR_TEST_CLOSED && !current_loop_started);
 float vd=Vd,vq=Vq; tick();
 assert(current_loop_started && fabsf(Vd-vd)<1e-6f && fabsf(Vq-vq)<1e-6f);
 for(unsigned i=0;i<CURRENT_BLEND_TICKS;i++) tick();
 assert(fabsf(target_id)<1e-6f && fabsf(target_iq-.25f)<1e-6f);
 Motor_Test_RequestStop(); assert(stop_stage==MOTOR_TEST_CLOSED);
 assert(motor_test_status==MOTOR_TEST_POSTZERO && !(TIM1->BDTR&TIM_BDTR_MOE));
 for(unsigned i=0;i<ZERO_OBSERVE_TICKS;i++) tick();
 assert(motor_test_status==MOTOR_TEST_STOPPED && !(TIM1->DIER&TIM_DIER_UIE));
 setup(TEST_OPEN); ADC1->ISR=0; ADC2->ISR=ADC_ISR_JEOS;
 TIM1->SR=TIM_SR_UIF; TIM1_UP_TIM16_IRQHandler();
 assert(motor_test_status==MOTOR_TEST_ADC_FAULT && motor_fault_latched);
 puts("PASS: PWM voltage scaling, open/angle/full isolation, lock dwell, PI boundary continuity, current ramp, normal stop and ADC fault.");
}
'''
with tempfile.TemporaryDirectory(prefix='foc-handoff-') as tmp:
 c=Path(tmp)/'test.c'; exe=Path(tmp)/'test'
 c.write_text((here/'zero_mock.h').read_text()+h+helpers+s+cases)
 subprocess.run(['gcc','-std=c11','-O2','-Wno-pointer-to-int-cast','-fsanitize=undefined','-fno-sanitize-recover=all',str(c),'-lm','-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
