#!/usr/bin/env python3
"""Check current timer ISR/SVPWM with ideal sin/cos; no hardware timing model."""
from pathlib import Path
import subprocess, tempfile
here=Path(__file__).resolve().parent
ns={}
runner=(here/'test_zero_stats.py').read_text()
exec(runner[:runner.index('with tempfile.TemporaryDirectory')], {'__file__':str(here/'test_zero_stats.py')}, ns)
unit=ns['unit'][:-len((here/'zero_stats_cases.c').read_text())]
uart=(here.parents[1]/'lib/usart/usart.c').read_bytes().decode('latin1')
a=uart.index('void uart2_parse_command(char *buf)')
b=uart.index('float parse_float_manual(char* s)', a)
parser=uart[a:b]
unit+='\n#include <stdlib.h>\nstatic float parse_float_manual(char *s) { return strtof(s, 0); }\n'+parser
unit+=r'''

static void static_start(void) {
 Motor_Test_SetReady(1);Motor_Test_RequestStaticAlign();assert(Motor_OpenLoop_Start());
 assert(static_align_active);for(unsigned i=0;i<3;i++)tick();
 for(unsigned i=0;i<45000;i++){assert(!(TIM1->BDTR&TIM_BDTR_MOE));tick();}
 assert(motor_test_status==MOTOR_TEST_ALIGN);
}
int main(void) {
 reset_all();Motor_Test_SetReady(1);
 uart2_parse_command("a1\n");assert(motor_start_requested==2);
 uart2_parse_command("m0\n");assert(motor_start_requested==0);
 uart2_parse_command("A1\n");assert(motor_start_requested==2);
 uart2_parse_command("M0\n");assert(motor_start_requested==0);
 uart2_parse_command("m1\n");assert(motor_start_requested==1);
 uart2_parse_command("m0\n");assert(motor_start_requested==0);
 reset_all();static_start();
 for(unsigned i=0;i<19500;i++) {
  fresh(2466,2517,2517,1,1);pump();
  assert(open_loop_angle==0 && elec_angle==0 && Vq==0);
  assert(Vd>=0 && Vd<=STATIC_ALIGN_MOD);
  assert(TIM1->BDTR&TIM_BDTR_MOE);
  assert(motor_test_status==MOTOR_TEST_ALIGN);
  if(i==11999)assert(static_align_stats.count==0);
  if(i==12000)assert(static_align_stats.count==0);
  if(i==12001)assert(static_align_stats.count==1);
  if(i==4500){Motor_Test_RequestStart();Motor_Test_RequestStaticAlign();assert(!motor_start_requested);}
 }
 assert(static_align_stats.count==7499);
 fresh(2466,2517,2517,1,1);pump();
 assert(static_align_stats.count==7500 && static_align_stats.sum[0]==2466U*7500);
 assert(!(TIM1->BDTR&TIM_BDTR_MOE));assert(motor_test_status==MOTOR_TEST_POSTZERO);
 assert(Vd==0 && Vq==0 && !run_foc_flag);
 Motor_Vofa_RequestText();for(unsigned i=0;i<45000;i++)tick();
 pump();uart.ISR=USART_ISR_TC;pump();off();assert(!vofa_stream_active);
 start();assert(!static_align_active && static_align_stats.count==0);Motor_Test_RequestStop();off();
 // Abort before output enables.
 reset_all();Motor_Test_SetReady(1);Motor_Test_RequestStaticAlign();assert(Motor_OpenLoop_Start());
 for(unsigned i=0;i<3;i++)tick();Motor_Test_RequestStop();off();
 for(unsigned i=0;i<45010;i++)tick();off();assert(static_align_stats.count==0);
 // Abort during hold leaves an incomplete report and never restarts.
 reset_all();static_start();for(unsigned i=0;i<13000;i++)tick();
 unsigned n=static_align_stats.count;assert(n==999);
 Motor_Test_RequestStop();Motor_Test_RequestStop();for(unsigned i=0;i<45000;i++)tick();
 off();assert(static_align_stats.count==n);Static_Align_Report();
 // Fault paths bypass normal postzero sampling and latch against restart.
 reset_all();static_start();fresh(2570,2500,2500,1,1);off();
 assert(motor_test_status==MOTOR_TEST_OVERCURRENT);
 Motor_Test_RequestStaticAlign();assert(!motor_start_requested);
 reset_all();static_start();fresh(2500,2500,2500,0,1);off();
 assert(motor_test_status==MOTOR_TEST_ADC_FAULT);
 puts("PASS: A1 fixed angle/ramp/deadline, exact 7500 samples, auto-off/postzero/D0, M0 cancellation/partial stats, faults and M1 isolation.");
}
'''
with tempfile.TemporaryDirectory(prefix='foc-output-') as tmp:
 c=Path(tmp)/'test.c'; exe=Path(tmp)/'test';c.write_text(unit)
 subprocess.run(['gcc','-std=c11','-Ofast','-Wno-pointer-to-int-cast','-fsanitize=undefined','-fno-sanitize-recover=all',str(c),'-lm','-o',str(exe)],check=True)
 output=subprocess.run([str(exe)],check=True,capture_output=True,text=True).stdout
 assert "mean U=0.9985 V=-0.4993 W=-0.4993 SUM=0.0000" in output, output
 assert "mean Id=0.9985 Iq=0.0000" in output, output
 assert "INCOMPLETE window" in output
 print(output)
