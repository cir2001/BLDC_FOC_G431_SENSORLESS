#!/usr/bin/env python3
"""Check current timer ISR/SVPWM with ideal sin/cos; no hardware timing model."""
from pathlib import Path
import subprocess, tempfile
here=Path(__file__).resolve().parent
ns={}
runner=(here/'test_zero_stats.py').read_text()
exec(runner[:runner.index('with tempfile.TemporaryDirectory')], {'__file__':str(here/'test_zero_stats.py')}, ns)
unit=ns['unit'][:-len((here/'zero_stats_cases.c').read_text())]
unit+=r'''
int main(void) {
 reset_all();prezero();
 for(unsigned i=0;i<45000;i++)tick();
 float last=0; unsigned prev[3]={TIM1->CCR1,TIM1->CCR2,TIM1->CCR3};
 unsigned lo=5666, hi=0, maxstep=0, wraps=0;
 double travelled=0;
 for(unsigned i=0;i<OPEN_LOOP_ALIGN_TICKS+OPEN_LOOP_RAMP_TICKS+150000;i++) {
  tick();float angle=open_loop_angle;float delta=angle-last;
  if(delta<0){delta+=6.2831853f;wraps++;}
  assert(delta>=0 && delta<OPEN_LOOP_TARGET_RPM*(6.2831853f/60.0f)*POLE_PAIRS*control_dt+1e-6f);
  if(i>=OPEN_LOOP_ALIGN_TICKS+OPEN_LOOP_RAMP_TICKS)travelled+=delta;
  assert(fabsf(Valpha*Valpha+Vbeta*Vbeta-Vd*Vd)<1e-8f);
  unsigned c[3]={TIM1->CCR1,TIM1->CCR2,TIM1->CCR3};
  for(unsigned j=0;j<3;j++) {
   unsigned step=c[j]>prev[j]?c[j]-prev[j]:prev[j]-c[j];
   if(i>0 && step>maxstep)maxstep=step;
   if(c[j]<lo)lo=c[j];if(c[j]>hi)hi=c[j];prev[j]=c[j];
  }
  last=angle;
 }
 assert(maxstep<= (unsigned)ceilf(0.433f*PWM_ARR*1.732051f*OPEN_LOOP_MAX_MOD*OPEN_LOOP_TARGET_RPM*(6.2831853f/60.0f)*POLE_PAIRS*control_dt)+1U);assert(lo>2700 && hi<2960);
 double rpm=travelled/(150000*control_dt)*60/(6.283185307179586*POLE_PAIRS);
 assert(fabs(rpm-OPEN_LOOP_TARGET_RPM)<0.02);
 printf("PASS: ideal trig, ramp/run/wrap continuous; CCR range %u..%u, max step %u, wraps %u, field rpm %.6f\n",lo,hi,maxstep,wraps,rpm);
}
'''
with tempfile.TemporaryDirectory(prefix='foc-output-') as tmp:
 c=Path(tmp)/'test.c'; exe=Path(tmp)/'test';c.write_text(unit)
 subprocess.run(['gcc','-std=c11','-Ofast','-Wno-pointer-to-int-cast','-fsanitize=undefined','-fno-sanitize-recover=all',str(c),'-lm','-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
