static void feed(unsigned n, int u, int v, int w) {
 for(unsigned i=0;i<n;i++) {fresh(u,v,w,1,1);pump();}
}
static void expect_stats(const ZeroWindowStats *s, unsigned n, unsigned u, unsigned v, unsigned w) {
 assert(s->count==n);
 assert(s->sum[0]==n*u && s->sum[1]==n*v && s->sum[2]==n*w);
 assert(s->sum[3]==n*(u+v+w));
}
int main(void) {
 stage_regression();
 reset_all();prezero();offset_u=2500.25f;offset_v=2499.5f;offset_w=2500.75f;
 feed(30000,2510,2510,2510);expect_stats(&zero_before,0,0,0,0);
 feed(1,2501,2499,2500);expect_stats(&zero_before,1,2501,2499,2500);
 feed(14998,2501,2499,2500);assert(zero_before.count==14999);assert(!(TIM1->BDTR&TIM_BDTR_MOE));
 feed(1,2501,2499,2500);expect_stats(&zero_before,15000,2501,2499,2500);
 assert(motor_test_status==MOTOR_TEST_ALIGN);
 Motor_Test_RequestStop();Motor_Vofa_RequestText();
 feed(30000,2520,2520,2520);expect_stats(&zero_after,0,0,0,0);
 feed(1,2503,2498,2504);expect_stats(&zero_after,1,2503,2498,2504);
 feed(14998,2503,2498,2504);assert(vofa_stream_active);assert(zero_after.count==14999);
 feed(1,2503,2498,2504);expect_stats(&zero_after,15000,2503,2498,2504);
 assert(postzero_samples==45000);off();pump();uart.ISR=USART_ISR_TC;pump();
 assert(!vofa_stream_active);
 puts("REPORT_CONSTANT");Zero_Stats_Report();
 // Next actual start resets both windows.
 start();assert(zero_before.count==0 && zero_after.count==0);
 // Correlated channels: SUM p-p is not the sum of phase p-p values.
 zero_before=(ZeroWindowStats){0};zero_after=(ZeroWindowStats){0};
 for(unsigned i=0;i<15000;i++) {
  Zero_Stats_Add(&zero_before, i%2?2502:2498, i%2?2498:2502,2500);
  Zero_Stats_Add(&zero_after, i%2?2503:2499, i%2?2497:2501,2500);
 }
 assert(zero_before.max[3]-zero_before.min[3]==0);
 puts("REPORT_CORRELATED");Zero_Stats_Report();
 ZeroWindowStats extreme={0};
 for(unsigned i=0;i<15000;i++)Zero_Stats_Add(&extreme,4095,4095,4095);
 assert(extreme.sum[3]==184275000U);
 // ADC missing partway through the final second excludes the invalid sample.
 reset_all();prezero();feed(45000,2500,2500,2500);Motor_Test_RequestStop();
 feed(30123,2500,2500,2500);fresh(2500,2500,2500,0,1);
 assert(zero_after.count==123 && postzero_samples==30123);off();
 puts("REPORT_PARTIAL");Zero_Stats_Report();
 reset_all();prezero();feed(100,2500,2500,2500);Motor_Test_RequestStop();off();
 puts("REPORT_EMPTY");Zero_Stats_Report();
 puts("PASS: exact window boundaries, sums, restart reset, correlated extrema, accumulator range, partial and empty windows.");
 return 0;
}
