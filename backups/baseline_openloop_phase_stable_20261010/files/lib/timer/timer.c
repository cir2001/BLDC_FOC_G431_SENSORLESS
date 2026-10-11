#include "timer.h"
#include "led.h"
#include <math.h>
#include "control.h"
#include "adc_foc.h"
#include <stdio.h>
#include "usart.h"
#include "delay.h"

//=============================================================================
// 1. 系统参数与宏定义
//=============================================================================
#define CURRENT_COUNTS_PER_AMP      (-34.05f)
#define INV_CURRENT_COUNTS_PER_AMP  (-0.029368575f)

// 旋转测试参数 (保持当前已验证稳定的 500 RPM 刚度参数)
#define OPEN_LOOP_ALIGN_MOD         (0.025f)  // 对齐期电压上限 (~0.8A)
#define OPEN_LOOP_MAX_MOD           (0.058f)  // 稳态旋转电压上限 (维持稳定牵引)
#define OPEN_LOOP_TARGET_RPM        (500.0f)  // 同步电磁转速 (RPM)
#define OPEN_LOOP_TRIP_A            (4.5f)    // 软件保护电流阈值 (A)
#define OPEN_LOOP_ALIGN_TICKS       (4500U)   // 对齐时间 0.3s
#define OPEN_LOOP_RAMP_TICKS        (45000U)  // 加速时间 3.0s

// 平滑切换参数
#define HANDOFF_WAIT_TICKS          (30000U)  // 稳态强拖 2.0s
#define HANDOFF_RAMP_TICKS          (15000U)  // 切换融合时间 1.0s

// SMO 与 PLL 滤波参数 (高精度快速锁相配置)
#define SMO_GAIN                    (0.40f)   // 滑模反电势增益
#define SMO_E_BOUNDARY              (0.08f)   // 连续化边界
#define SMO_LPF_FC                  (250.0f)  // 反电势滤波截止频率 250Hz
#define PLL_KP                      (120.0f)  // PLL 比例增益
#define PLL_KI                      (2500.0f) // PLL 积分增益

// 观察窗口参数
#define ZERO_OBSERVE_TICKS          (45000U)  // 前/后采样总时长 ~3.0s
#define ZERO_STATS_TICKS            (15000U)  // 统计窗口时长 (最后1秒)

// 稳态相位差统计窗口 (RUN 阶段经过 1.0s 收敛后，统计 2.0s)
#define PHASE_STATS_WARMUP_TICKS    (15000U)  // 稳态前 1.0s 预热丢弃
#define PHASE_STATS_WINDOW_TICKS    (30000U)  // 统计窗口 2.0s (30,000 samples)

// 静态对齐测试参数 (保留诊断支持)
#define STATIC_ALIGN_TARGET_ANGLE   (0.0f)    
#define STATIC_ALIGN_TARGET_ID      (0.3f)    
#define STATIC_ALIGN_HOLD_TICKS     (15000U)  
#define STATIC_ALIGN_STATS_TICKS    (7500U)   

// 电机电气参数 (2207 1950KV, Rs=0.035Ω, Ls=15μH)
#define V_BUS                       (12.0f)   // 母线直流电压 (V)
#define SMO_RS                      (0.035f)
#define SMO_LS                      (0.000015f)

//=============================================================================
// 2. 状态机、故障现场与统计结构体
//=============================================================================
volatile MotorTestStatus motor_test_status = MOTOR_TEST_IDLE;
static volatile uint8_t  motor_test_ready = 0U;
static volatile uint8_t  motor_start_requested = 0U;
static volatile uint8_t  motor_fault_latched = 0U;

// 测试运行模式: 1=M1(完整闭环), 2=A1(静态对齐), 3=O1(纯开环+影子), 4=H1(仅角度闭环电压维持)
static volatile uint8_t  test_running_mode = 0U; 

static uint32_t open_loop_ticks = 0U;
static uint32_t handoff_run_ticks = 0U;
static uint32_t handoff_trans_ticks = 0U;
static uint32_t zero_observe_ticks = 0U;
static uint32_t preflight_ticks = 0U;
static uint32_t preflight_good = 0U;
static float    control_dt = 0.0f;

static float handoff_delta_angle = 0.0f; 
static volatile float test_peak_a = 0.0f;

static volatile uint32_t postzero_samples = 0U;
static volatile uint32_t postzero_overlimit = 0U;
static volatile uint32_t postzero_adc1_isr = 0U, postzero_adc2_isr = 0U;

// 相位差平稳性在线统计结构体
typedef struct {
    uint32_t samples;
    float    sum;
    float    min_diff;
    float    max_diff;
    uint8_t  complete;
} PhaseDiffStats;

static PhaseDiffStats phase_stats;

typedef enum {
    ADC_FAULT_NONE = 0,
    ADC_FAULT_START_CONFIG,      
    ADC_FAULT_PREFLIGHT_TIMEOUT, 
    ADC_FAULT_RUN_ADC1_MISSING,  
    ADC_FAULT_RUN_ADC2_MISSING,  
    ADC_FAULT_RUN_BOTH_MISSING   
} AdcFaultSubReason;

typedef struct {
    uint32_t adc1_isr, adc2_isr;
    uint32_t adc1_cr,  adc2_cr;
    uint32_t tim1_sr,  tim1_cr1;
    uint32_t check_cnt, clear_cnt;
    uint8_t  stage;
    uint8_t  sub_reason;
} FaultSnapshot;

static volatile FaultSnapshot fault_snap;

typedef struct {
    uint32_t count;
    uint32_t sum[4];
    uint32_t min[4];
    uint32_t max[4];
} ZeroWindowStats;

static ZeroWindowStats zero_before, zero_after;
static uint8_t         static_align_active = 0U;
static uint32_t        static_align_ticks = 0U;
static ZeroWindowStats static_align_stats;

typedef struct {
    float    peak[10];
    uint32_t samples[10];
    uint32_t calls, max_cycles, over_budget, last_entry, min_period, max_period;
    uint32_t angle_checks, angle_bad, trig_bad;
    float    max_angle_error, max_norm_error, max_trig_step;
    float    previous_s, previous_c;
    uint8_t  have_entry, have_trig, clock_ok;
} RunDiagnostics;

static RunDiagnostics run_diag;

// 滑模观测器 (SMO)
typedef struct {
    float i_alpha_est;
    float i_beta_est;
    float e_alpha_filt;
    float e_beta_filt;
    float angle_est;       
    float pll_angle;       
    float pll_speed;       
} SMO_Observer;

static SMO_Observer smo;

//=============================================================================
// 3. FOC 控制与调试变量
//=============================================================================
float open_loop_angle = 0.0f;
float elec_angle = 0.0f;
volatile float Vd = 0.0f, Vq = 0.0f;
volatile float Valpha = 0.0f, Vbeta = 0.0f;
float target_id = 0.0f, target_iq = 0.0f;
float target_speed = 0.0f; 
float target_pos = 0.0f;   

extern PID_Controller pid_id, pid_iq;
extern float offset_u, offset_v, offset_w;
extern volatile uint8_t run_foc_flag;

// VOFA 缓冲
VofaData_t DataLog[2][SAMPLE_NUM][FRAME_SIZE];
enum { VOFA_FREE, VOFA_FILLING, VOFA_READY, VOFA_SENDING };
static volatile uint8_t  vofa_bank_state[2] = {VOFA_FREE, VOFA_FREE};
static volatile uint16_t vofa_bank_used[2] = {0U, 0U};
static uint8_t  vofa_tx_bank = 0xFFU;
static uint32_t vofa_bank_sequence[2];
static uint32_t vofa_next_sequence = 0U;
static uint8_t  vofa_downsample = 0U;
static volatile uint32_t vofa_dropped = 0U;
volatile uint8_t vofa_stream_active = 0U;
static volatile uint8_t  vofa_text_requested = 0U;

// 内部函数声明
static void Motor_Control_Update(void);
static void Motor_Test_Trip(MotorTestStatus reason);
static void Zero_Stats_Add(ZeroWindowStats *stats, uint32_t u, uint32_t v, uint32_t w);
static void Zero_Stats_Report(void);
static void Run_Diag_Report(void);
static void Run_Diag_Check(float current_angle, float previous_angle, float expected_step, float s, float c);
static void Static_Align_Report(void);
static void Motor_Vofa_Record(float ch0, float ch1, float ch2, float ch3, float ch4);
static void Motor_Vofa_Service(void);
static void SMO_Update(float valpha, float vbeta, float ialpha, float ibeta, float dt);

//=============================================================================
// 4. 定时器中断与核心控制
//=============================================================================
void TIM1_UP_TIM16_IRQHandler(void)
{
    if (!(TIM1->SR & TIM_SR_UIF)) return;
    
    uint32_t entry = DWT->CYCCNT;
    if (run_diag.have_entry) {
        uint32_t period = entry - run_diag.last_entry;
        if (period < run_diag.min_period) run_diag.min_period = period;
        if (period > run_diag.max_period) run_diag.max_period = period;
        if (period != 0U) run_diag.clock_ok = 1U;
    }
    run_diag.last_entry = entry;
    run_diag.have_entry = 1U;

    Motor_Control_Update();

    uint32_t elapsed = DWT->CYCCNT - entry;
    if (elapsed > run_diag.max_cycles) run_diag.max_cycles = elapsed;
    if (elapsed >= 2U * PWM_ARR_INT) ++run_diag.over_budget;
    ++run_diag.calls;
}

static void Motor_Control_Update(void)
{
    TIM1->SR &= ~TIM_SR_UIF;

    if (run_foc_flag != 1U && motor_test_status != MOTOR_TEST_POSTZERO) {
        Motor_Stop();
        return;
    }

    // --- A. 注入组采样检查与硬件现场记录 ---
    uint8_t preflight = (motor_test_status == MOTOR_TEST_PREFLIGHT);
    if (preflight) ++preflight_ticks;

    uint32_t check_cnt = TIM1->CNT;
    uint32_t isr1 = ADC1->ISR;
    uint32_t isr2 = ADC2->ISR;

    if (motor_test_status == MOTOR_TEST_POSTZERO) {
        if ((isr1 & ADC_ISR_JEOS) == 0U || (isr2 & ADC_ISR_JEOS) == 0U) {
            ADC1->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
            ADC2->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
            return;
        }
    } else {
        if ((isr1 & ADC_ISR_JEOS) == 0U || (isr2 & ADC_ISR_JEOS) == 0U) {
            if (preflight && preflight_ticks < 8U) {
                preflight_good = 0U;
                ADC1->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
                ADC2->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
                return;
            }

            fault_snap.adc1_isr   = isr1;
            fault_snap.adc2_isr   = isr2;
            fault_snap.adc1_cr    = ADC1->CR;
            fault_snap.adc2_cr    = ADC2->CR;
            fault_snap.tim1_sr    = TIM1->SR;
            fault_snap.tim1_cr1   = TIM1->CR1;
            fault_snap.check_cnt  = check_cnt;
            fault_snap.clear_cnt  = TIM1->CNT;
            fault_snap.stage      = (uint8_t)motor_test_status;

            if (preflight) {
                fault_snap.sub_reason = ADC_FAULT_PREFLIGHT_TIMEOUT;
            } else if ((isr1 & ADC_ISR_JEOS) == 0U && (isr2 & ADC_ISR_JEOS) == 0U) {
                fault_snap.sub_reason = ADC_FAULT_RUN_BOTH_MISSING;
            } else if ((isr1 & ADC_ISR_JEOS) == 0U) {
                fault_snap.sub_reason = ADC_FAULT_RUN_ADC1_MISSING;
            } else {
                fault_snap.sub_reason = ADC_FAULT_RUN_ADC2_MISSING;
            }

            postzero_adc1_isr = isr1;
            postzero_adc2_isr = isr2;
            Motor_Test_Trip(MOTOR_TEST_ADC_FAULT);
            return;
        }
    }

    uint32_t sample_u = ADC1->JDR1 & 0x0FFFU;
    uint32_t sample_v = ADC2->JDR1 & 0x0FFFU;
    uint32_t sample_w = ADC2->JDR2 & 0x0FFFU;

    ADC1->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC; 
    ADC2->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
    fault_snap.clear_cnt = TIM1->CNT;

    if (sample_u <= 16U || sample_u >= 4079U ||
        sample_v <= 16U || sample_v >= 4079U ||
        sample_w <= 16U || sample_w >= 4079U) {
        Motor_Test_Trip(MOTOR_TEST_ADC_RANGE);
        return;
    }

    // --- B. 电流解算 (安培) ---
    float raw_u = (float)sample_u - offset_u;
    float raw_v = (float)sample_v - offset_v;
    float raw_w = (float)sample_w - offset_w;

    float iu_uncomp = raw_u * INV_CURRENT_COUNTS_PER_AMP;
    float iv_uncomp = raw_v * INV_CURRENT_COUNTS_PER_AMP;
    float iw_uncomp = raw_w * INV_CURRENT_COUNTS_PER_AMP;

    float peak = fabsf(iu_uncomp);
    float abs_v = fabsf(iv_uncomp);
    float abs_w = fabsf(iw_uncomp);
    if (abs_v > peak) peak = abs_v;
    if (abs_w > peak) peak = abs_w;
    if (peak > test_peak_a) test_peak_a = peak;

    unsigned stage = (unsigned)motor_test_status;
    if (stage < 10U) {
        ++run_diag.samples[stage];
        if (peak > run_diag.peak[stage]) run_diag.peak[stage] = peak;
    }

    if (motor_test_status == MOTOR_TEST_POSTZERO) {
        TIM1->BDTR &= ~TIM_BDTR_MOE;
        TIM1->CCR1 = PWM_ARR_INT / 2U;
        TIM1->CCR2 = PWM_ARR_INT / 2U;
        TIM1->CCR3 = PWM_ARR_INT / 2U;
        Valpha = 0.0f;
        Vbeta  = 0.0f;
        ++postzero_samples;
        if (peak >= OPEN_LOOP_TRIP_A) {
            ++postzero_overlimit;
        }
        if (zero_observe_ticks >= ZERO_OBSERVE_TICKS - ZERO_STATS_TICKS) {
            Zero_Stats_Add(&zero_after, sample_u, sample_v, sample_w);
        }
        if (++zero_observe_ticks >= ZERO_OBSERVE_TICKS) {
            Motor_Stop();
            motor_test_status = MOTOR_TEST_STOPPED;
        }
        return; 
    } else if (peak >= OPEN_LOOP_TRIP_A) {
        Motor_Test_Trip(MOTOR_TEST_OVERCURRENT);
        return;
    }

    // --- C. 阶段切换与预采样 ---
    if (preflight) {
        if (++preflight_good >= 3U) {
            zero_observe_ticks = 0U;
            motor_test_status = MOTOR_TEST_PREZERO;
        } else if (preflight_ticks >= 8U) {
            Motor_Test_Trip(MOTOR_TEST_ADC_FAULT);
        }
        return;
    }

    if (motor_test_status == MOTOR_TEST_PREZERO) {
        if (zero_observe_ticks >= ZERO_OBSERVE_TICKS - ZERO_STATS_TICKS) {
            Zero_Stats_Add(&zero_before, sample_u, sample_v, sample_w);
        }
        if (++zero_observe_ticks >= ZERO_OBSERVE_TICKS) {
            open_loop_ticks = 0U;
            motor_test_status = MOTOR_TEST_ALIGN;
            TIM1->BDTR |= TIM_BDTR_MOE;
        }
        return;
    }

    // --- D. 零序补偿与 Clarke 变换 ---
    float com = (raw_u + raw_v + raw_w) * 0.33333333f;
    float iu = (raw_u - com) * INV_CURRENT_COUNTS_PER_AMP;
    float iv = (raw_v - com) * INV_CURRENT_COUNTS_PER_AMP;
    float iw = (raw_w - com) * INV_CURRENT_COUNTS_PER_AMP;

    float i_alpha = iu;
    float i_beta  = (iv - iw) * 0.57735027f;

    // --- E. 角度与指令生成 ---
    float previous_angle = elec_angle;
    float expected_step = 0.0f;

    if (static_align_active) {
        const uint32_t total = OPEN_LOOP_ALIGN_TICKS + STATIC_ALIGN_HOLD_TICKS;
        if (static_align_ticks > total - STATIC_ALIGN_STATS_TICKS) {
            Zero_Stats_Add(&static_align_stats, sample_u, sample_v, sample_w);
        }
        if (static_align_ticks >= total) {
            Motor_Test_RequestStop();
            return;
        }
        ++static_align_ticks;
        open_loop_angle = STATIC_ALIGN_TARGET_ANGLE;

        float id_ramp = (static_align_ticks < OPEN_LOOP_ALIGN_TICKS) ?
                        ((float)static_align_ticks / (float)OPEN_LOOP_ALIGN_TICKS) : 1.0f;
        target_id = STATIC_ALIGN_TARGET_ID * id_ramp;
        target_iq = 0.0f;
        elec_angle = open_loop_angle;
    } else {
        // 旋转模式
        if (open_loop_ticks < OPEN_LOOP_ALIGN_TICKS + OPEN_LOOP_RAMP_TICKS) {
            ++open_loop_ticks;
        }

        if (open_loop_ticks <= OPEN_LOOP_ALIGN_TICKS) {
            open_loop_angle = 0.0f;
            Vd = OPEN_LOOP_ALIGN_MOD * (float)open_loop_ticks / (float)OPEN_LOOP_ALIGN_TICKS;
            Vq = 0.0f;
            elec_angle = open_loop_angle;
        } 
        else if (open_loop_ticks < OPEN_LOOP_ALIGN_TICKS + OPEN_LOOP_RAMP_TICKS) {
            uint32_t ramp_ticks = open_loop_ticks - OPEN_LOOP_ALIGN_TICKS;
            float ramp = (float)ramp_ticks / (float)OPEN_LOOP_RAMP_TICKS;
            motor_test_status = MOTOR_TEST_RAMP;
            
            float omega_e = OPEN_LOOP_TARGET_RPM * (6.2831853f / 60.0f) * POLE_PAIRS;
            expected_step = omega_e * ramp * control_dt;
            open_loop_angle += expected_step;
            if (open_loop_angle >= 6.2831853f) open_loop_angle -= 6.2831853f;

            Vd = OPEN_LOOP_ALIGN_MOD + (OPEN_LOOP_MAX_MOD - OPEN_LOOP_ALIGN_MOD) * ramp;
            Vq = 0.0f;
            elec_angle = open_loop_angle;
        }
        else {
            float omega_e = OPEN_LOOP_TARGET_RPM * (6.2831853f / 60.0f) * POLE_PAIRS;
            expected_step = omega_e * control_dt;
            open_loop_angle += expected_step;
            if (open_loop_angle >= 6.2831853f) open_loop_angle -= 6.2831853f;

            if (motor_test_status == MOTOR_TEST_RAMP) {
                motor_test_status = MOTOR_TEST_RUN;
                handoff_run_ticks = 0U;
            }

            // 模式分支逻辑
            if (test_running_mode == 3U) {
                // ==================== O1: 纯开环稳态 + 影子在线相位差统计 ====================
                elec_angle = open_loop_angle;
                Vd = OPEN_LOOP_MAX_MOD;
                Vq = 0.0f;

                ++handoff_run_ticks;
                // 在进入 RUN 阶段 1.0s (15000 拍) 之后启动统计，连续采集 30000 拍 (2.0s)
                if (handoff_run_ticks > PHASE_STATS_WARMUP_TICKS &&
                    phase_stats.samples < PHASE_STATS_WINDOW_TICKS) {
                    
                    float diff = open_loop_angle - smo.angle_est;
                    while (diff >  3.14159265f) diff -= 6.2831853f;
                    while (diff < -3.14159265f) diff += 6.2831853f;

                    phase_stats.sum += diff;
                    if (phase_stats.samples == 0U || diff < phase_stats.min_diff) phase_stats.min_diff = diff;
                    if (phase_stats.samples == 0U || diff > phase_stats.max_diff) phase_stats.max_diff = diff;

                    ++phase_stats.samples;
                    if (phase_stats.samples == PHASE_STATS_WINDOW_TICKS) {
                        phase_stats.complete = 1U;
                    }
                }
            }
            else {
                // ==================== M1(mode=1) 或 H1(mode=4) ====================
                if (motor_test_status == MOTOR_TEST_RUN) {
                    elec_angle = open_loop_angle;
                    Vd = OPEN_LOOP_MAX_MOD;
                    Vq = 0.0f;

                    if (++handoff_run_ticks >= HANDOFF_WAIT_TICKS) {
                        float target_omega_e = OPEN_LOOP_TARGET_RPM * (6.2831853f / 60.0f) * POLE_PAIRS;
                        if (smo.pll_speed > 0.0f && fabsf(smo.pll_speed - target_omega_e) < (target_omega_e * 0.15f)) {
                            motor_test_status = MOTOR_TEST_HANDOFF;
                            handoff_trans_ticks = 0U;

                            float init_delta = open_loop_angle - smo.angle_est;
                            while (init_delta >  3.14159265f) init_delta -= 6.2831853f;
                            while (init_delta < -3.14159265f) init_delta += 6.2831853f;
                            handoff_delta_angle = init_delta;
                        }
                    }
                }
                else if (motor_test_status == MOTOR_TEST_HANDOFF) {
                    float blend = (float)handoff_trans_ticks / (float)HANDOFF_RAMP_TICKS;
                    if (blend > 1.0f) blend = 1.0f;

                    float blended_angle = smo.angle_est + (1.0f - blend) * handoff_delta_angle;
                    while (blended_angle >= 6.2831853f) blended_angle -= 6.2831853f;
                    while (blended_angle < 0.0f)        blended_angle += 6.2831853f;
                    elec_angle = blended_angle;

                    expected_step = OPEN_LOOP_TARGET_RPM * (6.2831853f / 60.0f) * POLE_PAIRS * control_dt;

                    Vd = OPEN_LOOP_MAX_MOD * (1.0f - 0.70f * blend);
                    Vq = OPEN_LOOP_MAX_MOD * (0.80f * blend);

                    if (++handoff_trans_ticks >= HANDOFF_RAMP_TICKS) {
                        motor_test_status = MOTOR_TEST_CLOSED;
                    }
                } 
                else if (motor_test_status == MOTOR_TEST_CLOSED) {
                    elec_angle = smo.angle_est;
                    expected_step = smo.pll_speed * control_dt;

                    if (test_running_mode == 4U) {
                        // H1: 仅角度接管，维持充足电压保持高速
                        Vd = 0.012f;
                        Vq = 0.046f;
                    } else {
                        // M1: 电流双闭环
                        target_id = 0.0f;
                        target_iq = 0.25f; 
                    }
                }
            }
        }
    }

    // --- F. 正余弦与 Park 变换 (Cortex-M4F 原生指令) ---
    float st, ct;
    CORDIC_SinCos(elec_angle, &st, &ct);
    Run_Diag_Check(elec_angle, previous_angle, expected_step, st, ct);

    float i_d =  i_alpha * ct + i_beta * st;
    float i_q = -i_alpha * st + i_beta * ct;

    // --- G. 控制执行 ---
    if (static_align_active) {
        Vd = PID_Calc_Current(&pid_id, target_id, i_d);
        Vq = PID_Calc_Current(&pid_iq, target_iq, i_q);
    }
    else if (motor_test_status == MOTOR_TEST_CLOSED && test_running_mode == 1U) {
        Vd = PID_Calc_Current(&pid_id, target_id, i_d);
        Vq = PID_Calc_Current(&pid_iq, target_iq, i_q);
    }

    // 逆 Park 变换
    Valpha = Vd * ct - Vq * st;
    Vbeta  = Vd * st + Vq * ct;

    // --- H. 滑模观测器 (SMO) 影子运行 ---
    float v_actual_scale = V_BUS * 0.4330127f;
    float v_alpha_volts  = Valpha * v_actual_scale;
    float v_beta_volts   = Vbeta  * v_actual_scale;
    SMO_Update(v_alpha_volts, v_beta_volts, i_alpha, i_beta, control_dt);

    // --- I. SVPWM 调制输出 ---
    SVPWM_Output_Standard(Valpha, Vbeta);

    // --- J. VOFA 通信记录 ---
    if (!preflight) {
        float target_we = OPEN_LOOP_TARGET_RPM * (6.2831853f / 60.0f) * POLE_PAIRS;
        float speed_ratio = (target_we > 0.1f) ? (smo.pll_speed / target_we) : 0.0f;
        Motor_Vofa_Record(iu_uncomp, iv_uncomp, speed_ratio, open_loop_angle, smo.angle_est);
    }
}

//=============================================================================
// 5. 滑模观测器 (SMO) 影子实现
//=============================================================================
static void SMO_Update(float valpha, float vbeta, float ialpha, float ibeta, float dt)
{
    float err_alpha = smo.i_alpha_est - ialpha;
    float err_beta  = smo.i_beta_est  - ibeta;

    float z_alpha = (fabsf(err_alpha) <= SMO_E_BOUNDARY) ?
                    (err_alpha / SMO_E_BOUNDARY * SMO_GAIN) :
                    ((err_alpha > 0.0f) ? SMO_GAIN : -SMO_GAIN);

    float z_beta  = (fabsf(err_beta) <= SMO_E_BOUNDARY) ?
                    (err_beta / SMO_E_BOUNDARY * SMO_GAIN) :
                    ((err_beta > 0.0f) ? SMO_GAIN : -SMO_GAIN);

    float inv_ls = 1.0f / SMO_LS;
    smo.i_alpha_est += ((-SMO_RS * smo.i_alpha_est + valpha - z_alpha) * inv_ls) * dt;
    smo.i_beta_est  += ((-SMO_RS * smo.i_beta_est  + vbeta  - z_beta)  * inv_ls) * dt;

    float wc = 6.2831853f * SMO_LPF_FC;
    float lpf_alpha = wc * dt;
    if (lpf_alpha > 1.0f) lpf_alpha = 1.0f;
    smo.e_alpha_filt += lpf_alpha * (z_alpha - smo.e_alpha_filt);
    smo.e_beta_filt  += lpf_alpha * (z_beta  - smo.e_beta_filt);

    // PLL 鉴相
    float s_pll, c_pll;
    CORDIC_SinCos(smo.pll_angle, &s_pll, &c_pll);
    float pll_error = -smo.e_alpha_filt * c_pll - smo.e_beta_filt * s_pll;

    smo.pll_speed += (PLL_KI * pll_error) * dt;
    if (smo.pll_speed < 10.0f) smo.pll_speed = 10.0f;

    float omega_pll = smo.pll_speed + (PLL_KP * pll_error);
    if (omega_pll < 0.0f) omega_pll = 0.0f;

    smo.pll_angle += omega_pll * dt;
    while (smo.pll_angle >= 6.2831853f) smo.pll_angle -= 6.2831853f;
    while (smo.pll_angle < 0.0f)        smo.pll_angle += 6.2831853f;

    float phase_comp = omega_pll / wc;
    float final_angle = smo.pll_angle + phase_comp;
    while (final_angle >= 6.2831853f) final_angle -= 6.2831853f;
    while (final_angle < 0.0f)        final_angle += 6.2831853f;

    smo.angle_est = final_angle;
}

//=============================================================================
// 6. 定时器与 PWM 硬件初始化
//=============================================================================
void TIM1_PWM_Init(u16 arr)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    GPIO_Set(GPIOA, PIN8|PIN9|PIN10|PIN11|PIN12, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_170M, GPIO_PUPD_PU); 
    GPIO_Set(GPIOB, PIN9|PIN15, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_170M, GPIO_PUPD_PU); 
    GPIO_Set(GPIOC, PIN13, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_170M, GPIO_PUPD_PU); 

    GPIO_AF_Set(GPIOA, 8, 6); 
    GPIO_AF_Set(GPIOA, 9, 6); 
    GPIO_AF_Set(GPIOA, 10, 6);
    GPIO_AF_Set(GPIOC, 13, 4); 
    GPIO_AF_Set(GPIOA, 12, 6); 
    GPIO_AF_Set(GPIOB, 15, 4);
    GPIO_AF_Set(GPIOA, 11, 11);

    TIM1->ARR = arr;
    TIM1->PSC = 0;
    TIM1->RCR = 1;

    TIM1->CR1 &= ~TIM_CR1_CMS;
    TIM1->CR1 |= (1U << TIM_CR1_CMS_Pos);

    TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M);
    TIM1->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM1->CCMR1 |= (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 &= ~TIM_CCMR2_OC3M;
    TIM1->CCMR2 |= (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;

    TIM1->CCMR2 &= ~(TIM_CCMR2_OC4M | (1U << 24));
    TIM1->CCMR2 |= (7U << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;

    TIM1->CCR4 = arr - 450; 

    TIM1->CR2 &= ~TIM_CR2_MMS2;
    TIM1->CR2 |= (7U << TIM_CR2_MMS2_Pos);

    TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE | 
                   TIM_CCER_CC2E | TIM_CCER_CC2NE | 
                   TIM_CCER_CC3E | TIM_CCER_CC3NE | 
                   TIM_CCER_CC4E);

    TIM1->BDTR &= ~TIM_BDTR_DTG;
    TIM1->BDTR |= 100;
    TIM1->BDTR &= ~TIM_BDTR_MOE;

    TIM1->EGR |= TIM_EGR_UG;
    TIM1->CR1 |= TIM_CR1_CEN;
    
    NVIC_SetPriority(TIM1_UP_TIM16_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
}

//=============================================================================
// 7. SVPWM 算法
//=============================================================================
void SVPWM_Output_Standard(float va, float vb)
{
    float v_sq = va * va + vb * vb;
    if (v_sq > 1.3333333f) { 
        float inv_mod = 1.1547005f / sqrtf(v_sq);
        va *= inv_mod;
        vb *= inv_mod;
    }

    float u_u = va;
    float u_v = -0.5f * va + 0.8660254f * vb;
    float u_w = -0.5f * va - 0.8660254f * vb;

    float max_v = u_u, min_v = u_u;
    if (u_v > max_v) { max_v = u_v; }
    if (u_v < min_v) { min_v = u_v; }
    if (u_w > max_v) { max_v = u_w; }
    if (u_w < min_v) { min_v = u_w; }

    float v_offset = (max_v + min_v) * 0.5f;

    float ta = (u_u - v_offset + 1.1547f) * 0.4330127f; 
    float tb = (u_v - v_offset + 1.1547f) * 0.4330127f;
    float tc = (u_w - v_offset + 1.1547f) * 0.4330127f;

    uint16_t max_ccr = PWM_ARR - 200; 
    uint16_t ccr_u = (uint16_t)(ta * PWM_ARR);
    uint16_t ccr_v = (uint16_t)(tb * PWM_ARR);
    uint16_t ccr_w = (uint16_t)(tc * PWM_ARR);

    if (ccr_u > max_ccr) ccr_u = max_ccr;
    if (ccr_v > max_ccr) ccr_v = max_ccr;
    if (ccr_w > max_ccr) ccr_w = max_ccr;

    TIM1->CCR1 = ccr_u;
    TIM1->CCR2 = ccr_v;
    TIM1->CCR3 = ccr_w;
    TIM1->CCR4 = (uint16_t)(PWM_ARR - 450);
}

//=============================================================================
// 8. 电流环 PID 计算器
//=============================================================================
float PID_Calc_Current(PID_Controller* pid, float target, float current)
{
    float error = target - current;
    const float dt = 6.666667e-5f; 

    float p_term = pid->kp * error;
    float next_integral = pid->integral + (error * pid->ki * dt);

    if (next_integral > pid->output_limit)  next_integral = pid->output_limit;
    if (next_integral < -pid->output_limit) next_integral = -pid->output_limit;
    pid->integral = next_integral;

    float out = p_term + pid->integral;
    if (out > pid->output_limit)  out = pid->output_limit;
    if (out < -pid->output_limit) out = -pid->output_limit;

    return out;
}

void PID_Reset(PID_Controller* pid) 
{
    pid->integral = 0.0f;
}

//=============================================================================
// 9. 电机启停与控制状态机接口实现
//=============================================================================
void Motor_Stop(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->DIER &= ~TIM_DIER_UIE;
    run_foc_flag = 0U;
    motor_start_requested = 0U;
    if (motor_test_status >= MOTOR_TEST_PREFLIGHT &&
        motor_test_status <= MOTOR_TEST_POSTZERO) {
        motor_test_status = MOTOR_TEST_STOPPED;
    }
    target_id = 0.0f;
    target_iq = 0.0f;
    Vd = 0.0f;
    Vq = 0.0f;
    Valpha = 0.0f;
    Vbeta = 0.0f;
}

static void Motor_Test_Trip(MotorTestStatus reason)
{
    Motor_Stop();
    motor_fault_latched = 1U;
    motor_test_status = reason;
}

void Motor_Test_SetReady(uint8_t ready)
{
    motor_test_ready = (ready != 0U);
}

static void Motor_Test_RequestMode(uint8_t mode)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (motor_test_ready && !motor_fault_latched && run_foc_flag == 0U &&
        motor_test_status != MOTOR_TEST_POSTZERO) {
        motor_start_requested = mode; 
    }
    __set_PRIMASK(primask);
}

void Motor_Test_RequestStart(void)       { Motor_Test_RequestMode(1U); } // M1: 双闭环
void Motor_Test_RequestStaticAlign(void) { Motor_Test_RequestMode(2U); } // A1: 静态对齐
void Motor_Test_RequestOpen(void)        { Motor_Test_RequestMode(3U); } // O1: 纯开环+影子
void Motor_Test_RequestAngle(void)       { Motor_Test_RequestMode(4U); } // H1: 仅角度闭环电压维持

void Motor_Test_RequestStop(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    MotorTestStatus previous = motor_test_status;
    
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCR1 = PWM_ARR_INT / 2U;
    TIM1->CCR2 = PWM_ARR_INT / 2U;
    TIM1->CCR3 = PWM_ARR_INT / 2U;
    Valpha = 0.0f;
    Vbeta  = 0.0f;
    motor_start_requested = 0U;

    if (!motor_fault_latched && vofa_stream_active &&
        previous >= MOTOR_TEST_ALIGN && previous <= MOTOR_TEST_CLOSED) {
        run_foc_flag = 0U;
        target_id = target_iq = 0.0f;
        Vd = Vq = 0.0f;
        zero_observe_ticks = 0U;
        postzero_samples = postzero_overlimit = 0U;
        postzero_adc1_isr = postzero_adc2_isr = 0U;
        motor_test_status = MOTOR_TEST_POSTZERO;
        TIM1->DIER |= TIM_DIER_UIE;
    } else {
        Motor_Stop();
    }
    __set_PRIMASK(primask);
}

static uint8_t Motor_OpenLoop_Start(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (run_foc_flag != 0U || motor_test_status != MOTOR_TEST_IDLE) {
        motor_start_requested = 0U;
        __set_PRIMASK(primask);
        return 0U;
    }

    if (!motor_start_requested || !motor_test_ready || motor_fault_latched) {
        __set_PRIMASK(primask);
        return 0U;
    }
    if (DMA1_Channel1->CNDTR != 0U || vofa_tx_bank != 0xFFU ||
        vofa_bank_state[0] != VOFA_FREE || vofa_bank_state[1] != VOFA_FREE) {
        __set_PRIMASK(primask);
        return 0U;
    }

    uint8_t requested_mode = motor_start_requested;
    test_running_mode = requested_mode; 

    Motor_Stop();
    if (!(ADC1->CR & ADC_CR_ADEN) || !(ADC2->CR & ADC_CR_ADEN) ||
        !(ADC1->CR & ADC_CR_JADSTART) || !(ADC2->CR & ADC_CR_JADSTART) ||
        offset_u < 100.0f || offset_u > 3995.0f ||
        offset_v < 100.0f || offset_v > 3995.0f ||
        offset_w < 100.0f || offset_w > 3995.0f ||
        TIM1->ARR != PWM_ARR_INT || TIM1->PSC != 0U || TIM1->RCR != 1U) {
        
        fault_snap.adc1_isr   = ADC1->ISR;
        fault_snap.adc2_isr   = ADC2->ISR;
        fault_snap.adc1_cr    = ADC1->CR;
        fault_snap.adc2_cr    = ADC2->CR;
        fault_snap.tim1_sr    = TIM1->SR;
        fault_snap.tim1_cr1   = TIM1->CR1;
        fault_snap.check_cnt  = TIM1->CNT;
        fault_snap.clear_cnt  = TIM1->CNT;
        fault_snap.stage      = (uint8_t)motor_test_status;
        fault_snap.sub_reason = ADC_FAULT_START_CONFIG;

        Motor_Test_Trip(MOTOR_TEST_ADC_FAULT);
        __set_PRIMASK(primask);
        return 0U;
    }

    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM1->CR1 &= ~TIM_CR1_CMS;
    TIM1->CR1 &= ~TIM_CR1_DIR;
    TIM1->CNT = 0U;
    TIM1->CR1 |= (1U << TIM_CR1_CMS_Pos);
    TIM1->CCR1 = PWM_ARR_INT / 2U;
    TIM1->CCR2 = PWM_ARR_INT / 2U;
    TIM1->CCR3 = PWM_ARR_INT / 2U;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR = 0U;
    NVIC_ClearPendingIRQ(TIM1_UP_TIM16_IRQn);
    ADC1->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
    ADC2->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    static_align_active = (requested_mode == 2U);
    static_align_ticks = 0U;
    static_align_stats = (ZeroWindowStats){0};
    run_diag = (RunDiagnostics){0};
    run_diag.min_period = UINT32_MAX;
    zero_before = (ZeroWindowStats){0};
    zero_after = (ZeroWindowStats){0};
    fault_snap = (FaultSnapshot){0};
    phase_stats = (PhaseDiffStats){0};
    open_loop_angle = 0.0f;
    elec_angle = 0.0f;
    open_loop_ticks = 0U;
    handoff_run_ticks = 0U;
    handoff_trans_ticks = 0U;
    preflight_ticks = preflight_good = 0U;
    postzero_samples = postzero_overlimit = 0U;
    postzero_adc1_isr = postzero_adc2_isr = 0U;
    zero_observe_ticks = 0U;
    test_peak_a = 0.0f;

    handoff_delta_angle = 0.0f;

    pid_id.integral = 0.0f;
    pid_iq.integral = 0.0f;
    smo = (SMO_Observer){0};

    control_dt = (2.0f * (float)TIM1->ARR) / 170000000.0f;
    vofa_stream_active = 1U;
    vofa_text_requested = 0U;
    vofa_downsample = 0U;
    vofa_dropped = 0U;
    motor_test_status = MOTOR_TEST_PREFLIGHT;
    run_foc_flag = 1U;
    TIM1->DIER |= TIM_DIER_UIE;
    TIM1->CR1 |= TIM_CR1_CEN;
    __set_PRIMASK(primask);
    return 1U;
}

void Motor_Test_Service(void)
{
    Motor_Vofa_Service();
    if (motor_start_requested) (void)Motor_OpenLoop_Start();
    if (vofa_stream_active) return;
    static MotorTestStatus last_status = MOTOR_TEST_IDLE;
    MotorTestStatus status = motor_test_status;
    if (status == last_status) return;
    last_status = status;
    const char *message;
    switch (status) {
        case MOTOR_TEST_PREFLIGHT: message = "PREFLIGHT (MOE=0)"; break;
        case MOTOR_TEST_PREZERO: message = "PREZERO (3s, MOE=0)"; break;
        case MOTOR_TEST_POSTZERO: message = "POSTZERO (3s, MOE=0)"; break;
        case MOTOR_TEST_ALIGN: message = "ALIGN"; break;
        case MOTOR_TEST_RAMP: message = "RAMP"; break;
        case MOTOR_TEST_RUN: message = "RUN"; break;
        case MOTOR_TEST_HANDOFF: message = "HANDOFF (Blending)"; break;
        case MOTOR_TEST_CLOSED: message = "CLOSED LOOP (SMO)"; break;
        case MOTOR_TEST_DONE: message = "DONE: time limit, output OFF"; break;
        case MOTOR_TEST_STOPPED: message = "STOPPED: output OFF"; break;
        case MOTOR_TEST_ADC_FAULT: message = "FAULT: ADC/timing, reset required"; break;
        case MOTOR_TEST_ADC_RANGE: message = "FAULT: ADC near rail, reset required"; break;
        case MOTOR_TEST_OVERCURRENT: message = "FAULT: current limit, reset required"; break;
        default: message = "IDLE"; break;
    }
    printf("[OpenLoop] %s; peak=%.3fA (nominal)\r\n", message, (double)test_peak_a);
}

//=============================================================================
// 10. VOFA 通信协议与 DMA 发送
//=============================================================================
static void Motor_Vofa_Record(float ch0, float ch1, float ch2, float ch3, float ch4)
{
    if (!vofa_stream_active || ++vofa_downsample < 10U) return;
    vofa_downsample = 0U;
    uint8_t bank = 0xFFU;
    for (uint8_t i = 0U; i < 2U; ++i) {
        if (vofa_bank_state[i] == VOFA_FILLING) { bank = i; break; }
    }
    if (bank == 0xFFU) {
        for (uint8_t i = 0U; i < 2U; ++i) {
            if (vofa_bank_state[i] == VOFA_FREE) {
                bank = i;
                vofa_bank_used[i] = 0U;
                vofa_bank_state[i] = VOFA_FILLING;
                break;
            }
        }
    }
    if (bank == 0xFFU) { ++vofa_dropped; return; }
    uint16_t at = vofa_bank_used[bank];
    
    DataLog[bank][at][0].f = ch0;          // CH0: Iu
    DataLog[bank][at][1].f = ch1;          // CH1: Iv
    DataLog[bank][at][2].f = ch2;          // CH2: speed_ratio
    DataLog[bank][at][3].f = ch3;          // CH3: open_loop_angle
    DataLog[bank][at][4].f = ch4;          // CH4: smo.angle_est
    DataLog[bank][at][5].u = 0x7F800000U;  // JustFloat 帧尾
    
    vofa_bank_used[bank] = at + 1U;
    if (vofa_bank_used[bank] == SAMPLE_NUM) {
        vofa_bank_sequence[bank] = vofa_next_sequence++;
        __DMB();
        vofa_bank_state[bank] = VOFA_READY;
    }
}

void Motor_Vofa_RequestText(void)
{
    if (run_foc_flag == 0U) vofa_text_requested = 1U;
}

static void Motor_Vofa_Service(void)
{
    if (!vofa_stream_active) return;
    if (vofa_tx_bank != 0xFFU) {
        uint32_t flags = DMA1->ISR;
        if (flags & (DMA_ISR_TCIF1 | DMA_ISR_TEIF1)) {
            DMA1_Channel1->CCR &= ~DMA_CCR_EN;
            DMA1->IFCR = DMA_IFCR_CGIF1;
            if (flags & DMA_ISR_TEIF1) vofa_dropped += vofa_bank_used[vofa_tx_bank];
            vofa_bank_used[vofa_tx_bank] = 0U;
            __DMB();
            vofa_bank_state[vofa_tx_bank] = VOFA_FREE;
            vofa_tx_bank = 0xFFU;
        }
    }
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (run_foc_flag == 0U && motor_test_status != MOTOR_TEST_POSTZERO) {
        for (uint8_t i = 0U; i < 2U; ++i) {
            if (vofa_bank_state[i] == VOFA_FILLING && vofa_bank_used[i] != 0U) {
                vofa_bank_sequence[i] = vofa_next_sequence++;
                vofa_bank_state[i] = VOFA_READY;
            }
        }
    }
    if (vofa_tx_bank == 0xFFU) {
        uint8_t bank = 0xFFU;
        for (uint8_t i = 0U; i < 2U; ++i) {
            if (vofa_bank_state[i] == VOFA_READY &&
                (bank == 0xFFU || (int32_t)(vofa_bank_sequence[i] - vofa_bank_sequence[bank]) < 0)) {
                bank = i;
            }
        }
        if (bank != 0xFFU) {
            vofa_bank_state[bank] = VOFA_SENDING;
            vofa_tx_bank = bank;
            DMA1_Channel1->CCR &= ~DMA_CCR_EN;
            DMA1->IFCR = DMA_IFCR_CGIF1;
            DMA1_Channel1->CMAR = (uint32_t)DataLog[bank];
            DMA1_Channel1->CNDTR = (uint32_t)vofa_bank_used[bank] * FRAME_SIZE * sizeof(VofaData_t);
            __DMB();
            DMA1_Channel1->CCR |= DMA_CCR_EN;
        }
    }
    uint8_t text_ready = vofa_text_requested && !run_foc_flag &&
                        motor_test_status != MOTOR_TEST_POSTZERO &&
                        vofa_tx_bank == 0xFFU &&
                        vofa_bank_state[0] == VOFA_FREE &&
                        vofa_bank_state[1] == VOFA_FREE &&
                        (USART2->ISR & USART_ISR_TC);
    if (text_ready) {
        vofa_text_requested = 0U;
        vofa_stream_active = 0U;
    }
    __set_PRIMASK(primask);
    if (text_ready) {
        printf("[VOFA] text mode; dropped=%lu frames.\r\n", (unsigned long)vofa_dropped);
        printf("[PostZero] samples=%lu/45000 overlimit=%lu status=%u MOE=%u UIE=%u\r\n",
               (unsigned long)postzero_samples, (unsigned long)postzero_overlimit,
               (unsigned int)motor_test_status,
               (unsigned int)((TIM1->BDTR & TIM_BDTR_MOE) != 0U),
               (unsigned int)((TIM1->DIER & TIM_DIER_UIE) != 0U));
        printf("[PostZero] missing-data ISR1=%08lX ISR2=%08lX\r\n",
               (unsigned long)postzero_adc1_isr, (unsigned long)postzero_adc2_isr);
        Zero_Stats_Report();
        Run_Diag_Report();
        if (static_align_active) Static_Align_Report();
    }
}

//=============================================================================
// 11. 统计与文本诊断报告
//=============================================================================
static void Zero_Stats_Add(ZeroWindowStats *stats, uint32_t u, uint32_t v, uint32_t w)
{
    const uint32_t values[4] = {u, v, w, u + v + w};
    for (uint32_t ch = 0U; ch < 4U; ++ch) {
        stats->sum[ch] += values[ch];
        if (stats->count == 0U || values[ch] < stats->min[ch]) stats->min[ch] = values[ch];
        if (stats->count == 0U || values[ch] > stats->max[ch]) stats->max[ch] = values[ch];
    }
    ++stats->count;
}

static void Zero_Stats_Report(void)
{
    const ZeroWindowStats *windows[2] = {&zero_before, &zero_after};
    const char *names[2] = {"before", "after"};
    const float offsets[4] = {offset_u, offset_v, offset_w, offset_u + offset_v + offset_w};
    float means[2][4] = {{0}};
    printf("[ZeroCompare] Last 1s of each 3s window; nominal A; same offsets.\r\n");
    for (uint32_t win = 0U; win < 2U; ++win) {
        const ZeroWindowStats *stats = windows[win];
        printf("[ZeroCompare] %s n=%lu/%lu complete=%u\r\n", names[win],
               (unsigned long)stats->count, (unsigned long)ZERO_STATS_TICKS,
               (unsigned int)(stats->count == ZERO_STATS_TICKS));
        if (stats->count == 0U) continue;
        float pp[4];
        for (uint32_t ch = 0U; ch < 4U; ++ch) {
            means[win][ch] = ((float)stats->sum[ch] / (float)stats->count - offsets[ch]) *
                             INV_CURRENT_COUNTS_PER_AMP;
            pp[ch] = (float)(stats->max[ch] - stats->min[ch]) * fabsf(INV_CURRENT_COUNTS_PER_AMP);
        }
        printf("[ZeroCompare] %s mean U=%.4f V=%.4f W=%.4f SUM=%.4f\r\n",
               names[win], (double)means[win][0], (double)means[win][1],
               (double)means[win][2], (double)means[win][3]);
        printf("[ZeroCompare] %s p-p  U=%.4f V=%.4f W=%.4f SUM=%.4f\r\n",
               names[win], (double)pp[0], (double)pp[1], (double)pp[2], (double)pp[3]);
    }
    if (zero_before.count == ZERO_STATS_TICKS && zero_after.count == ZERO_STATS_TICKS) {
        printf("[ZeroCompare] delta(after-before) U=%.4f V=%.4f W=%.4f SUM=%.4f\r\n",
               (double)(means[1][0] - means[0][0]), (double)(means[1][1] - means[0][1]),
               (double)(means[1][2] - means[0][2]), (double)(means[1][3] - means[0][3]));
    } else {
        printf("[ZeroCompare] INCOMPLETE: no full-window comparison.\r\n");
    }
}

static uint8_t Diag_Finite(float value)
{
    union { float f; uint32_t u; } bits = {.f = value};
    return (bits.u & 0x7F800000U) != 0x7F800000U;
}

static void Run_Diag_Check(float current_angle, float previous_angle, float expected_step, float s, float c)
{
    ++run_diag.angle_checks;
    float step = current_angle - previous_angle;
    if (step < -3.14159265f) step += 6.2831853f;
    if (step >  3.14159265f) step -= 6.2831853f;

    float error = fabsf(step - expected_step);
    if (!Diag_Finite(current_angle) || !Diag_Finite(error) || error > 0.05f)
        ++run_diag.angle_bad;
    if (Diag_Finite(error) && error > run_diag.max_angle_error) run_diag.max_angle_error = error;
    if (!Diag_Finite(s) || !Diag_Finite(c)) {
        ++run_diag.trig_bad;
        run_diag.have_trig = 0U;
        return;
    }
    float norm_error = fabsf(s*s + c*c - 1.0f);
    if (norm_error > run_diag.max_norm_error) run_diag.max_norm_error = norm_error;
    float change = 0.0f;
    if (run_diag.have_trig) {
        change = fmaxf(fabsf(s-run_diag.previous_s), fabsf(c-run_diag.previous_c));
        if (change > run_diag.max_trig_step) run_diag.max_trig_step = change;
    }
    if (norm_error > 0.001f || (run_diag.have_trig && change > fabsf(expected_step)+0.05f))
        ++run_diag.trig_bad;
    run_diag.previous_s = s; run_diag.previous_c = c; run_diag.have_trig = 1U;
}

static void Run_Diag_Report(void)
{
    static const char *names[10] = {
        "idle", "preflight", "prezero", "align", "ramp", 
        "run", "handoff", "closed", "postzero", "done"
    };

    delay_ms(10);
    printf("[RunDiag] field=%.1frpm mod=%.3f; peak nominal A; sampled stage at ISR entry\r\n",
           (double)(static_align_active ? 0.0f : OPEN_LOOP_TARGET_RPM),
           (double)(static_align_active ? 0.0f : OPEN_LOOP_MAX_MOD));
    
    for (unsigned i = 1U; i <= 8U; ++i) {
        delay_ms(10);
        printf("[RunDiag] %s n=%lu peak=%.4fA\r\n", names[i],
               (unsigned long)run_diag.samples[i], (double)run_diag.peak[i]);
    }
    
    if (fault_snap.sub_reason != ADC_FAULT_NONE) {
        delay_ms(10);
        printf("[FaultSnap] sub_reason=%u stage=%s TIM1_CR1=%04lX TIM1_SR=%04lX\r\n",
               fault_snap.sub_reason, 
               (fault_snap.stage < 10) ? names[fault_snap.stage] : "unknown",
               (unsigned long)fault_snap.tim1_cr1, (unsigned long)fault_snap.tim1_sr);
        delay_ms(10);
        printf("[FaultSnap] CNT_check=%lu CNT_clear=%lu | DIR=%u\r\n",
               (unsigned long)fault_snap.check_cnt, (unsigned long)fault_snap.clear_cnt,
               (unsigned int)((fault_snap.tim1_cr1 & TIM_CR1_DIR) != 0U));
        delay_ms(10);
        printf("[FaultSnap] ADC1_ISR=%08lX ADC1_CR=%08lX | ADC2_ISR=%08lX ADC2_CR=%08lX\r\n",
               (unsigned long)fault_snap.adc1_isr, (unsigned long)fault_snap.adc1_cr,
               (unsigned long)fault_snap.adc2_isr, (unsigned long)fault_snap.adc2_cr);
    }

    delay_ms(10);
    printf("[RunDiag] angle n=%lu bad=%lu max_error=%.8frad\r\n",
           (unsigned long)run_diag.angle_checks, (unsigned long)run_diag.angle_bad,
           (double)run_diag.max_angle_error);
    delay_ms(10);
    printf("[RunDiag] trig bad=%lu norm_error=%.8f max_step=%.8f\r\n",
           (unsigned long)run_diag.trig_bad, (double)run_diag.max_norm_error,
           (double)run_diag.max_trig_step);
    delay_ms(10);
    printf("[RunDiag] DWT_valid=%u calls=%lu body_max=%lucyc over_budget=%lu\r\n",
           (unsigned)run_diag.clock_ok, (unsigned long)run_diag.calls,
           (unsigned long)run_diag.max_cycles, (unsigned long)run_diag.over_budget);
    delay_ms(10);
    printf("[RunDiag] entry_period min=%lu max=%lucyc expected=%u; 170cyc/us\r\n",
           (unsigned long)(run_diag.calls>1U ? run_diag.min_period : 0U),
           (unsigned long)run_diag.max_period, 2U*PWM_ARR_INT);
    delay_ms(10);

    // ==================== 稳态相位差平稳性诊断报告 ====================
    if (phase_stats.samples > 0U) {
        float mean_diff = phase_stats.sum / (float)phase_stats.samples;
        float pp_diff   = phase_stats.max_diff - phase_stats.min_diff;
        float mean_deg  = mean_diff * 57.29578f;
        float pp_deg    = pp_diff   * 57.29578f;

        printf("[PhaseDiag] 500rpm RUN steady window (n=%lu/%lu, complete=%u):\r\n",
               (unsigned long)phase_stats.samples, (unsigned long)PHASE_STATS_WINDOW_TICKS,
               (unsigned int)phase_stats.complete);
        delay_ms(10);
        printf("[PhaseDiag] phase_diff (open - smo): mean=%.4frad (%.1fdeg), min=%.4f, max=%.4f\r\n",
               (double)mean_diff, (double)mean_deg, (double)phase_stats.min_diff, (double)phase_stats.max_diff);
        delay_ms(10);
        printf("[PhaseDiag] phase_diff P-P jitter=%.4frad (%.2fdeg) -> %s\r\n",
               (double)pp_diff, (double)pp_deg,
               (pp_diff < 0.10f) ? "PASS: PHASE LOCKED & STABLE" :
               ((pp_diff < 0.20f) ? "ACCEPTABLE: MINOR RIPPLE" : "WARN: UNSTABLE DRIFT"));
        delay_ms(10);
    } else {
        printf("[PhaseDiag] INCOMPLETE: no steady-state window captured.\r\n");
        delay_ms(10);
    }
}

static void Static_Align_Report(void)
{
    delay_ms(10);
    printf("[StaticAlign] angle=%.4frad target_Id=%.2fA hold=1s; Current PI ON\r\n",
           (double)STATIC_ALIGN_TARGET_ANGLE, (double)STATIC_ALIGN_TARGET_ID);
    delay_ms(10);
    printf("[StaticAlign] last 0.5s n=%lu/%lu complete=%u\r\n",
           (unsigned long)static_align_stats.count, (unsigned long)STATIC_ALIGN_STATS_TICKS,
           (unsigned)(static_align_stats.count == STATIC_ALIGN_STATS_TICKS));
    if (!static_align_stats.count) {
        delay_ms(10);
        printf("[StaticAlign] INCOMPLETE: no hold-window samples\r\n");
        delay_ms(10);
        return;
    }
    float offsets[3] = {offset_u, offset_v, offset_w};
    float mean[3], pp[3];
    for (unsigned i=0; i<3; ++i) {
        mean[i] = ((float)static_align_stats.sum[i] / (float)static_align_stats.count - offsets[i]) *
                  INV_CURRENT_COUNTS_PER_AMP;
        pp[i] = (float)(static_align_stats.max[i]-static_align_stats.min[i]) * fabsf(INV_CURRENT_COUNTS_PER_AMP);
    }
    float sum = mean[0]+mean[1]+mean[2];
    
    float st, ct;
    CORDIC_SinCos(STATIC_ALIGN_TARGET_ANGLE, &st, &ct);
    float i_alpha = mean[0] - sum/3.0f;
    float i_beta  = (mean[1] - mean[2]) * 0.57735027f;
    float id =  i_alpha * ct + i_beta * st;
    float iq = -i_alpha * st + i_beta * ct;

    delay_ms(10);
    printf("[StaticAlign] mean U=%.4f V=%.4f W=%.4f SUM=%.4f A nominal, uncompensated\r\n",
           (double)mean[0], (double)mean[1], (double)mean[2], (double)sum);
    delay_ms(10);
    printf("[StaticAlign] p-p U=%.4f V=%.4f W=%.4f A\r\n", (double)pp[0], (double)pp[1], (double)pp[2]);
    delay_ms(10);
    printf("[StaticAlign] mean Id=%.4f Iq=%.4f A; commanded angle=%.4frad, zero-sequence corrected\r\n",
           (double)id, (double)iq, (double)STATIC_ALIGN_TARGET_ANGLE);
    delay_ms(10);
    printf("[StaticAlign] %s; rotor alignment NOT automatically verified\r\n",
           static_align_stats.count == STATIC_ALIGN_STATS_TICKS ? "window complete" : "INCOMPLETE window");
    delay_ms(10);
}