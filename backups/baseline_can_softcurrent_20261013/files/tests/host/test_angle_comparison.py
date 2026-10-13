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

static void setup(unsigned rpm, unsigned p) {
 motor_test_status=MOTOR_TEST_IDLE; run_foc_flag=0; motor_fault_latched=0;
 motor_test_ready=1; motor_start_requested=0;
 memset((void*)vofa_bank_state,0,sizeof(vofa_bank_state)); vofa_tx_bank=0xff;
 DMA1_Channel1->CNDTR=0; TIM1->ARR=PWM_ARR_INT; TIM1->PSC=0; TIM1->RCR=1;
 ADC1->CR=ADC2->CR=ADC_CR_ADEN|ADC_CR_JADSTART;
 Motor_Test_RequestComparison(rpm,p); assert(Motor_OpenLoop_Start());
}
static void tick(void) {
 ADC1->ISR=ADC2->ISR=ADC_ISR_JEOS;
 ADC1->JDR1=ADC2->JDR1=ADC2->JDR2=2500;
 TIM1->SR=TIM_SR_UIF; TIM1_UP_TIM16_IRQHandler();
}
int main(void) {
 run_diag=(RunDiagnostics){0}; open_loop_angle=3;
 Run_Diag_Check(.01f,6.2731853f,.02f,sinf(.01f),cosf(.01f));
 assert(run_diag.angle_bad==0);
 run_diag=(RunDiagnostics){0};
 Run_Diag_Check(6.2731853f,.01f,-.02f,sinf(6.2731853f),cosf(6.2731853f));
 assert(run_diag.angle_bad==0);
 Run_Diag_Check(1,0,.02f,sinf(1),cosf(1)); assert(run_diag.angle_bad==1);
 setup(450,0); assert(test_rpm==450 && test_p_gain==0);
 setup(550,0); assert(test_rpm==550 && test_p_gain==0);
 setup(500,1); assert(test_rpm==500 && test_p_gain==.002f);
 setup(500,0); assert(test_rpm==500 && test_p_gain==0);
 motor_test_status=MOTOR_TEST_CLOSED;
 open_loop_ticks=OPEN_LOOP_ALIGN_TICKS+OPEN_LOOP_RAMP_TICKS;
 closed_initialized=1; target_id=3; target_iq=-2;
 // Actual observer angle increments, independent from open-loop reference.
 elec_angle=0; smo.angle_est=.02f; observer_angle_step=.02f; handoff_delta_angle=0;
 tick(); assert(run_diag.angle_bad==0 && Vd==OPEN_LOOP_MAX_MOD && Vq==0);
 closed_stats.samples=15000;
 for(unsigned i=0;i<30000;i++) tick();
 assert(win_stats.complete && win_stats.samples==30000);
 assert(win_stats.sat_count_d==0 && win_stats.sat_count_q==0);
 closed_stats.samples=74999; tick();
 assert(motor_test_status==MOTOR_TEST_POSTZERO && !(TIM1->BDTR&TIM_BDTR_MOE));
 for(unsigned i=0;i<45000;i++) tick();
 assert(motor_test_status==MOTOR_TEST_STOPPED && !(TIM1->DIER&TIM_DIER_UIE));
 puts("PASS: both angle wraps, real jump detection, observer reference, test selection, pure feedforward, complete window and timed stop.");
}
'''
with tempfile.TemporaryDirectory(prefix='foc-handoff-') as tmp:
 c=Path(tmp)/'test.c'; exe=Path(tmp)/'test'
 c.write_text((here/'zero_mock.h').read_text()+h+helpers+s+cases)
 subprocess.run(['gcc','-std=c11','-O2','-Wno-pointer-to-int-cast','-fsanitize=undefined','-fno-sanitize-recover=all',str(c),'-lm','-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
