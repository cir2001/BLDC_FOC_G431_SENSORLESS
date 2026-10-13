
PID_Controller pid_id, pid_iq, pid_speed, pid_pos;
volatile uint8_t run_foc_flag;
float offset_u=2500, offset_v=2500, offset_w=2500;
volatile float debug_id, debug_iq, debug_Vq, debug_Vd, debug_iu, debug_iv, debug_iw;

static void fresh(int u,int v,int w,int f1,int f2) {
 ADC1->JDR1=u; ADC2->JDR1=v; ADC2->JDR2=w;
 ADC1->ISR=f1 ? ADC_ISR_JEOS : 0; ADC2->ISR=f2 ? ADC_ISR_JEOS : 0;
 TIM1->SR=TIM_SR_UIF; TIM1_UP_TIM16_IRQHandler();
}
static void reset_test(void) {
 memset(&timer_regs,0,sizeof(timer_regs)); memset(&adc1_regs,0,sizeof(adc1_regs)); memset(&adc2_regs,0,sizeof(adc2_regs));
 run_foc_flag=0; motor_fault_latched=0; motor_start_requested=0; motor_test_ready=0;
 motor_test_status=MOTOR_TEST_IDLE; offset_u=offset_v=offset_w=2500;
 TIM1->ARR=5666; TIM1->RCR=1; ADC1->CR=ADC2->CR=ADC_CR_ADEN|ADC_CR_JADSTART;
}
static void start(void) {Motor_Test_SetReady(1); Motor_Test_RequestStart(); assert(Motor_OpenLoop_Start()); assert(!(TIM1->BDTR&TIM_BDTR_MOE));}
static void power_on(void) {start(); for(int i=0;i<3;i++) fresh(2500,2500,2500,1,1); assert(TIM1->BDTR&TIM_BDTR_MOE);}
static void off(void) {assert(!(TIM1->BDTR&TIM_BDTR_MOE)); assert(!(TIM1->DIER&TIM_DIER_UIE)); assert(!run_foc_flag);}

static unsigned stages_seen;
static void reset_all(void) {
 reset_test(); memset(&dma_ch,0,sizeof(dma_ch)); memset(&dma,0,sizeof(dma));
 memset((void*)vofa_bank_state,0,sizeof(vofa_bank_state)); memset((void*)vofa_bank_used,0,sizeof(vofa_bank_used));
 vofa_tx_bank=0xFF;vofa_stream_active=0;vofa_text_requested=0;vofa_downsample=0;vofa_dropped=0;stages_seen=0;
}
static void pump(void) {
 Motor_Vofa_Service(); dma.ISR=0;
 if(vofa_tx_bank!=0xFF) {
  unsigned bank=vofa_tx_bank;
  assert(dma_ch.CNDTR==vofa_bank_used[bank]*24);
  for(unsigned i=0;i<vofa_bank_used[bank];i++) {
   assert(DataLog[bank][i][5].u==0x7F800000U);
   assert(fabsf(DataLog[bank][i][3].f-(DataLog[bank][i][0].f+DataLog[bank][i][1].f+DataLog[bank][i][2].f))<1e-5f);
   unsigned stage=(unsigned)DataLog[bank][i][4].f;assert(stage<=4);stages_seen|=1U<<stage;
  }
  dma_ch.CNDTR=0;dma.ISR=DMA_ISR_TCIF1;
 }
}
static void tick(void) {fresh(2500,2500,2500,1,1);pump();}
static void prezero(void) {start();for(int i=0;i<3;i++)tick(); assert(motor_test_status==MOTOR_TEST_PREZERO);}
static int stage_regression(void) {
 reset_all();prezero();
 for(unsigned i=0;i<ZERO_OBSERVE_TICKS-1;i++){tick();assert(!(TIM1->BDTR&TIM_BDTR_MOE));}
 tick();assert(TIM1->BDTR&TIM_BDTR_MOE); assert(motor_test_status==MOTOR_TEST_ALIGN);
 for(unsigned i=0;i<OPEN_LOOP_ALIGN_TICKS+OPEN_LOOP_RAMP_TICKS+10000;i++)tick();
 assert(motor_test_status==MOTOR_TEST_RUN);
 Motor_Test_RequestStop();assert(!(TIM1->BDTR&TIM_BDTR_MOE));assert(!run_foc_flag);
 assert(TIM1->DIER&TIM_DIER_UIE);assert(motor_test_status==MOTOR_TEST_POSTZERO);
 Motor_Test_RequestStart();assert(!motor_start_requested);
 Motor_Vofa_RequestText(); assert(vofa_text_requested);
 for(unsigned i=0;i<ZERO_OBSERVE_TICKS-1;i++) {
  tick();assert(!(TIM1->BDTR&TIM_BDTR_MOE));assert(vofa_stream_active);
  if(i==100) {unsigned saved=zero_observe_ticks;Motor_Test_RequestStop();assert(zero_observe_ticks==saved);}
 }
 tick();assert(motor_test_status==MOTOR_TEST_STOPPED);off();pump();pump();
 uart.ISR=USART_ISR_TC;pump();assert(!vofa_stream_active);assert(stages_seen==31U);assert(vofa_dropped==0);
 // M0 during startup cancels every future output enable.
 reset_all();prezero();for(int i=0;i<100;i++)tick();Motor_Test_RequestStop();off();
 for(int i=0;i<45010;i++)fresh(2500,2500,2500,1,1);off();assert(!motor_start_requested);
 // Missing samples during prezero are fail-closed.
 reset_all();prezero();fresh(2500,2500,2500,0,1);off();assert(motor_fault_latched);
 // Postzero overcurrent latches the fault but continues sampling with MOE off.
 reset_all();prezero();for(unsigned i=0;i<ZERO_OBSERVE_TICKS;i++)tick();Motor_Test_RequestStop();
 fresh(2570,2570,2570,1,1);
 assert(!(TIM1->BDTR&TIM_BDTR_MOE));assert(TIM1->DIER&TIM_DIER_UIE);
 assert(motor_fault_latched && postzero_overlimit==1 && motor_test_status==MOTOR_TEST_POSTZERO);
 Motor_Test_RequestStart();assert(!motor_start_requested);
 for(unsigned i=1;i<ZERO_OBSERVE_TICKS;i++)tick();
 off();assert(motor_test_status==MOTOR_TEST_OVERCURRENT);assert(postzero_samples==45000);
 reset_all();prezero();for(unsigned i=0;i<ZERO_OBSERVE_TICKS;i++)tick();
 fresh(2570,2570,2570,1,1);off();assert(motor_test_status==MOTOR_TEST_OVERCURRENT);
 puts("PASS: 3s pre/post, immediate M0, startup cancellation, repeated M0, blocked restart, deferred D0, five stages/24-byte frames, no drops and fault shutdown.");
 return 0;
}
