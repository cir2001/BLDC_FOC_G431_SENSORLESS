#!/usr/bin/env python3
"""Check current timer ISR/SVPWM with ideal sin/cos; no hardware timing model."""
from pathlib import Path
import subprocess, tempfile
here=Path(__file__).resolve().parent
ns={}
runner=(here/'test_zero_stats.py').read_text()
exec(runner[:runner.index('with tempfile.TemporaryDirectory')], {'__file__':str(here/'test_zero_stats.py')}, ns)
unit=ns['unit'][:-len((here/'zero_stats_cases.c').read_text())]
unit = "static unsigned injected_cycles;\n" + unit.replace(
    "Motor_Control_Update(); //", "Motor_Control_Update(); DWT->CYCCNT += injected_cycles; //")
unit+=r'''

int main(void) {
 reset_all();prezero();
 assert(run_diag.samples[MOTOR_TEST_PREFLIGHT]==3);
 for(unsigned i=0;i<45000;i++)tick();
 assert(run_diag.samples[MOTOR_TEST_PREZERO]==45000);
 for(unsigned i=0;i<OPEN_LOOP_ALIGN_TICKS+1U;i++){fresh(2510,2500,2500,1,1);pump();}
 for(unsigned i=0;i<OPEN_LOOP_RAMP_TICKS-1U;i++){fresh(2520,2500,2500,1,1);pump();}
 for(unsigned i=0;i<100;i++){fresh(2530,2500,2500,1,1);pump();}
 assert(fabsf(run_diag.peak[MOTOR_TEST_ALIGN]-10/34.05f)<1e-5f);
 assert(fabsf(run_diag.peak[MOTOR_TEST_RAMP]-20/34.05f)<1e-5f);
 assert(fabsf(run_diag.peak[MOTOR_TEST_RUN]-30/34.05f)<1e-5f);
 assert(run_diag.angle_bad==0 && run_diag.trig_bad==0);
 Motor_Test_RequestStop();tick();assert(run_diag.samples[MOTOR_TEST_POSTZERO]==1);
 // Timestamp wrap and missing-ADC early return must still be recorded.
 run_diag.have_entry=1;run_diag.last_entry=UINT32_MAX-100;run_diag.min_period=UINT32_MAX;
 DWT->CYCCNT=100;unsigned calls=run_diag.calls;
 fresh(2500,2500,2500,0,1);
 assert(run_diag.calls==calls+1 && run_diag.min_period==201 && run_diag.clock_ok);
 assert(motor_test_status==MOTOR_TEST_ADC_FAULT);
 run_diag=(RunDiagnostics){0};
 open_loop_angle=0.001f;Run_Diag_Check(6.2821853f,0.002f,sinf(0.001f),cosf(0.001f));
 assert(!run_diag.angle_bad && !run_diag.trig_bad);
 open_loop_angle=0.5f;Run_Diag_Check(0.001f,0.001f,0,2);
 assert(run_diag.angle_bad==1 && run_diag.trig_bad==1);
 union {uint32_t u;float f;} nan={.u=0x7fc00000};
 Run_Diag_Check(0,0,nan.f,1);assert(run_diag.trig_bad==2);
 reset_all();start();assert(run_diag.calls==0 && run_diag.angle_checks==0);
 assert(run_diag.samples[MOTOR_TEST_RUN]==0 && run_diag.min_period==UINT32_MAX);
 injected_cycles=2U*PWM_ARR_INT+1U;
 DWT->CYCCNT=UINT32_MAX-50U;tick();
 assert(run_diag.max_cycles==injected_cycles && run_diag.over_budget==1);
 puts("PASS: stage peaks, nominal checks, angle wrap/fault, invalid trig, timestamp wrap, fault exit accounting and restart reset.");
}
'''
with tempfile.TemporaryDirectory(prefix='foc-output-') as tmp:
 c=Path(tmp)/'test.c'; exe=Path(tmp)/'test';c.write_text(unit)
 subprocess.run(['gcc','-std=c11','-Ofast','-Wno-pointer-to-int-cast','-fsanitize=undefined','-fno-sanitize-recover=all',str(c),'-lm','-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
