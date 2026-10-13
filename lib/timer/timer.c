#include "timer.h"
#include "led.h"
#include <math.h>
#include "control.h"
#include "adc_foc.h"
#include <stdio.h>
#include "usart.h"
#include "delay.h"
#include <string.h>

//=============================================================================
// 1. 系统固化参数与控制宏 (Voltage-Mode Speed Control Standard Baseline)
//=============================================================================
#define CURRENT_COUNTS_PER_AMP      (-34.05f)
#define INV_CURRENT_COUNTS_PER_AMP  (-0.029368575f)

// 控制坐标系偏转基线 (+98.0 度电角度，1.5s 平滑过渡)
#define COORD_ROTATION_DELTA_RAD    (1.7097f)
#define COORD_ROTATION_RAMP_TICKS   (22500U)

// Target ramp: 50 RPM/s at 15 kHz, normalized by 500 RPM.
#define SPD_TARGET_SLEW_STEP        (50.0f / (15000.0f * 500.0f))

// 速度外环直接端压微调器参数 (500Hz 执行)
#define SPD_LOOP_DIVIDER            (30U)     // 15kHz / 30 = 500Hz
#define SPD_LOOP_DT                 (0.002f)  // 2.0 ms
#define SPD_TRIM_KP                 (0.080f)  // 提升比例增益 (由 0.060 提至 0.080，增强阻尼与冲顶抑制)
#define SPD_TRIM_KI                 (0.015f)  // 削减积分增益 (由 0.040 降至 0.015，根除爬升积分堆积引起的50RPM超调)
#define SPD_TRIM_LIMIT              (0.0120f) // 对称调压限幅 (+-0.0120)
#define SPD_TRIM_SLEW_MAX           (0.000080f)// 放宽变化率限制 (由 0.000040 提至 0.000080，允许拐点快速刹车退压)

// Experimental soft constraint: phase-peak envelope, NOT a torque-current loop.
#define SOFT_CURRENT_ON_A          (3.6f)
#define SOFT_CURRENT_OFF_A         (3.3f)
#define SOFT_ENVELOPE_DECAY_A       (0.0005f) // 7.5 A/s, fast attack / slow release
#define SOFT_TRIM_RETREAT_STEP      (0.000040f) // per 500Hz update, bounded by trim limit

// 轻量高频阻尼配置
#define DAMPING_LIMIT               (0.002f)

// 电压基座系数与基础电气参数
#define CLOSED_BASE_MOD_PER_RATIO   (0.0525f) // 500 RPM anchor
#define CLOSED_FF_600_MOD           (0.0571f) // Measured 600 RPM operating point
#define OPEN_LOOP_ALIGN_MOD         (0.025f)
#define OPEN_LOOP_MAX_MOD           (0.058f)
#define BASELINE_REF_RPM            (500.0f)
#define OPEN_LOOP_TRIP_A            (4.5f)    // 实测硬件保护门限 4.5A
#define OPEN_LOOP_ALIGN_TICKS       (4500U)   // 0.3s
#define OPEN_LOOP_RAMP_TICKS        (45000U)  // 3.0s
#define HANDOFF_WAIT_TICKS          (30000U)  // 2.0s 锁相确认

// 动态超速保护红线 (目标指令的 1.35x 持续 1500 拍即触发保护)
#define OVERSPEED_RELATIVE_LIMIT    (1.35f)
#define OVERSPEED_TICKS_LIMIT       (1500U)

// SMO 观测器参数
#define SMO_GAIN                    (0.40f)  
#define SMO_E_BOUNDARY              (0.08f)  
#define SMO_LPF_FC                  (200.0f) 
#define PLL_KP                      (90.0f)  
#define PLL_KI                      (1500.0f)

#define ZERO_OBSERVE_TICKS          (45000U) 
#define ZERO_STATS_TICKS            (15000U) 
#define V_BUS                       (12.0f)  
#define SMO_RS                      (0.035f)
#define SMO_LS                      (0.000015f)

// 静态对齐测试参数
#define STATIC_ALIGN_TARGET_ANGLE   (0.0f)   
#define STATIC_ALIGN_TARGET_ID      (0.3f)   
#define STATIC_ALIGN_HOLD_TICKS     (15000U) 
#define STATIC_ALIGN_STATS_TICKS    (7500U)  

//=============================================================================
// 2. 全局状态机与核心控制变量
//=============================================================================
volatile MotorTestStatus motor_test_status = MOTOR_TEST_IDLE;
static volatile uint8_t  motor_test_ready = 0U;
static volatile uint8_t  motor_start_requested = 0U;
static volatile uint8_t  motor_fault_latched = 0U;

static uint32_t open_loop_ticks = 0U;
static uint32_t handoff_run_ticks = 0U;
static uint32_t zero_observe_ticks = 0U;
static uint32_t preflight_ticks = 0U;
static uint32_t preflight_good = 0U;
static float    control_dt = 0.0f;
static float    handoff_delta_angle = 0.0f;
static volatile float test_peak_a = 0.0f;

static volatile uint32_t postzero_samples = 0U;
static volatile uint32_t postzero_overlimit = 0U;
static volatile uint32_t postzero_adc1_isr = 0U, postzero_adc2_isr = 0U;
static uint8_t closed_initialized = 0U;

// 解决外部引用的实体定义
float target_pos = 0.0f;
float target_speed = 50.0f;

// 指令目标与斜坡跟踪
static volatile float user_command_target_rpm = BASELINE_REF_RPM;
static volatile float internal_target_ratio   = 1.000f;
static volatile float internal_ramped_ratio   = 1.000f;

static uint32_t overspeed_counter = 0U;
typedef struct {
    float    speed_ratio;
    float    peak_a;
    float    vd;
    float    vq;
    uint8_t  stage;
    uint32_t tripped_ticks;
} OverspeedSnapshot;
static volatile OverspeedSnapshot overspeed_snap;

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

// 速度外环状态
typedef struct {
    float   p_term;
    float   integral;
    float   last_output;
    uint8_t saturated;
} SpeedVoltageLoop_t;
static SpeedVoltageLoop_t spd_loop;
static uint32_t           spd_loop_div_cnt = 0U;
static float soft_current_envelope = 0.0f;
static uint8_t soft_current_active = 0U;
static uint32_t soft_current_entries = 0U;

// 实时遥测镜像 (供外部安全异步读取)
static Actuator_Telemetry_t actuator_telem;

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

typedef struct {
    float i_alpha_est;
    float i_beta_est;
    float e_alpha_filt;
    float e_beta_filt;
    float angle_est;       
    float last_angle_est;  
    float actual_step;     
    float pll_angle;       
    float pll_speed;       
} SMO_Observer;
static SMO_Observer smo;

//=============================================================================
// 3. FOC 控制核心变量与前向声明
//=============================================================================
float open_loop_angle = 0.0f;
float elec_angle = 0.0f;
volatile float Vd = 0.0f, Vq = 0.0f;
volatile float Valpha = 0.0f, Vbeta = 0.0f;
float target_id = 0.0f, target_iq = 0.0f;

extern PID_Controller pid_id, pid_iq;
extern float offset_u, offset_v, offset_w;
extern volatile uint8_t run_foc_flag;

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
// 4. 对外固化标准 API 与指令兼容接口
//=============================================================================
void Motor_Set_Target_RPM(float rpm)
{
    if (rpm < 300.0f) rpm = 300.0f;
    if (rpm > 800.0f) rpm = 800.0f;
    
    user_command_target_rpm = rpm;
    internal_target_ratio   = rpm / BASELINE_REF_RPM;
}

// Read the requested target even while stopped (telemetry contains the ramped target).
float Motor_Get_Command_Target_RPM(void)
{
    return user_command_target_rpm;
}

void Motor_Get_Telemetry(Actuator_Telemetry_t* telem)
{
    if (telem == NULL) return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *telem = actuator_telem;
    telem->trim_saturated = spd_loop.saturated;
    telem->soft_current_active = soft_current_active;
    telem->phase_envelope_a = soft_current_envelope;
    telem->soft_current_entries = soft_current_entries;
    telem->status = (uint8_t)motor_test_status;
    telem->moe = (uint8_t)((TIM1->BDTR & TIM_BDTR_MOE) != 0U);
    telem->measurement_valid = (motor_test_status == MOTOR_TEST_CLOSED &&
                                run_foc_flag != 0U && telem->moe != 0U &&
                                closed_initialized != 0U);
    __set_PRIMASK(primask);
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

void Motor_Test_RequestStart(void)       { Motor_Test_RequestMode(1U); }
void Motor_Test_RequestOpen(void)        { Motor_Test_RequestMode(1U); }
void Motor_Test_RequestAngle(void)       { Motor_Test_RequestMode(1U); }
void Motor_Test_RequestStaticAlign(void) { Motor_Test_RequestMode(2U); }
void Motor_Test_RequestComparison(unsigned rpm, unsigned with_p)
{
    (void)rpm;
    (void)with_p;
    Motor_Test_RequestMode(1U);
}

//=============================================================================
// 5. 定时器中断与核心控制
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
            ADC2->ISR = ADC_ISR_JEOC | ADC_ISR_JEOC;
            return;
        }
    } else {
        if ((isr1 & ADC_ISR_JEOS) == 0U || (isr2 & ADC_ISR_JEOS) == 0U) {
            if (preflight && preflight_ticks < 8U) {
                preflight_good = 0U;
                ADC1->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
                ADC2->ISR = ADC_ISR_JEOC | ADC_ISR_JEOS;
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
    ADC2->ISR = ADC_ISR_JEOC | ADC_ISR_JEOS;
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
            Vd = 0.0f;
            Vq = 0.0f;
            Valpha = 0.0f;
            Vbeta  = 0.0f;
            TIM1->CCR1 = PWM_ARR_INT / 2U;
            TIM1->CCR2 = PWM_ARR_INT / 2U;
            TIM1->CCR3 = PWM_ARR_INT / 2U;
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
    static float last_applied_elec_angle = 0.0f;
    float previous_angle = last_applied_elec_angle;
    float expected_step = 0.0f;
    static float current_rot_delta = 0.0f;
    static uint32_t closed_ticks = 0U;

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
        expected_step = 0.0f;
    } else {
        if (open_loop_ticks < OPEN_LOOP_ALIGN_TICKS + OPEN_LOOP_RAMP_TICKS) {
            ++open_loop_ticks;
        }

        if (open_loop_ticks <= OPEN_LOOP_ALIGN_TICKS) {
            open_loop_angle = 0.0f;
            Vd = OPEN_LOOP_ALIGN_MOD * (float)open_loop_ticks / (float)OPEN_LOOP_ALIGN_TICKS;
            Vq = 0.0f;
            elec_angle = open_loop_angle;
            expected_step = 0.0f;
        } 
        else if (open_loop_ticks < OPEN_LOOP_ALIGN_TICKS + OPEN_LOOP_RAMP_TICKS) {
            uint32_t ramp_ticks = open_loop_ticks - OPEN_LOOP_ALIGN_TICKS;
            float ramp = (float)ramp_ticks / (float)OPEN_LOOP_RAMP_TICKS;
            motor_test_status = MOTOR_TEST_RAMP;
            
            float omega_e = BASELINE_REF_RPM * (6.2831853f / 60.0f) * POLE_PAIRS;
            expected_step = omega_e * ramp * control_dt;
            open_loop_angle += expected_step;
            if (open_loop_angle >= 6.2831853f) open_loop_angle -= 6.2831853f;

            Vd = OPEN_LOOP_ALIGN_MOD + (OPEN_LOOP_MAX_MOD - OPEN_LOOP_ALIGN_MOD) * ramp;
            Vq = 0.0f;
            elec_angle = open_loop_angle;
        } 
        else {
            float omega_e = BASELINE_REF_RPM * (6.2831853f / 60.0f) * POLE_PAIRS;
            expected_step = omega_e * control_dt;
            open_loop_angle += expected_step;
            if (open_loop_angle >= 6.2831853f) open_loop_angle -= 6.2831853f;

            if (motor_test_status == MOTOR_TEST_RAMP) {
                motor_test_status = MOTOR_TEST_RUN;
            }

            if (motor_test_status == MOTOR_TEST_RUN) {
                elec_angle = open_loop_angle;
                Vd = OPEN_LOOP_MAX_MOD;
                Vq = 0.0f;

                if (++handoff_run_ticks >= HANDOFF_WAIT_TICKS) {
                    float target_omega_e = BASELINE_REF_RPM * (6.2831853f / 60.0f) * POLE_PAIRS;
                    if (smo.pll_speed > 0.0f && fabsf(smo.pll_speed - target_omega_e) < (target_omega_e * 0.25f)) {
                        float delta0 = open_loop_angle - smo.angle_est;
                        while (delta0 >  3.14159265f) delta0 -= 6.2831853f;
                        while (delta0 < -3.14159265f) delta0 += 6.2831853f;
                        handoff_delta_angle = delta0;

                        motor_test_status = MOTOR_TEST_CLOSED;
                        closed_initialized = 0U;
    soft_current_envelope = 0.0f;
    soft_current_active = 0U;
    soft_current_entries = 0U;
                        overspeed_counter = 0U;
                        current_rot_delta = 0.0f;
                        closed_ticks = 0U;
                        internal_ramped_ratio = 1.000f;
                    }
                }
            } 
            else if (motor_test_status == MOTOR_TEST_CLOSED) {
                float delta_step = 0.0f;
                if (COORD_ROTATION_DELTA_RAD != 0.0f) {
                    if (closed_ticks < COORD_ROTATION_RAMP_TICKS) {
                        float prev_delta = current_rot_delta;
                        float progress = (float)(closed_ticks + 1U) / (float)COORD_ROTATION_RAMP_TICKS;
                        current_rot_delta = COORD_ROTATION_DELTA_RAD * progress;
                        delta_step = current_rot_delta - prev_delta;
                    } else {
                        current_rot_delta = COORD_ROTATION_DELTA_RAD;
                        delta_step = 0.0f;
                    }
                } else {
                    current_rot_delta = 0.0f;
                    delta_step = 0.0f;
                }

                float comp_angle = smo.angle_est + handoff_delta_angle - current_rot_delta;
                while (comp_angle >= 6.2831853f) comp_angle -= 6.2831853f;
                while (comp_angle < 0.0f)        comp_angle += 6.2831853f;
                elec_angle = comp_angle;

                expected_step = smo.actual_step - delta_step;
            }
        }
    }

    // --- F. CORDIC 正余弦与 Park 变换 ---
    float st, ct;
    CORDIC_SinCos(elec_angle, &st, &ct);

    if (motor_test_status >= MOTOR_TEST_RAMP && motor_test_status <= MOTOR_TEST_CLOSED) {
        if (closed_initialized != 0U || motor_test_status != MOTOR_TEST_CLOSED) {
            Run_Diag_Check(elec_angle, previous_angle, expected_step, st, ct);
        }
    }
    last_applied_elec_angle = elec_angle;

    // 当前控制角下的实测电流分解 (Id_rot, Iq_rot)
    float i_d_rot =  i_alpha * ct + i_beta * st;
    float i_q_rot = -i_alpha * st + i_beta * ct;

    // --- G. 闭环控制执行 (标准固化基线) ---
    float s_delta, c_delta;
    CORDIC_SinCos(current_rot_delta, &s_delta, &c_delta);

    if (static_align_active) {
        Vd = PID_Calc_Current(&pid_id, target_id, i_d_rot);
        Vq = PID_Calc_Current(&pid_iq, target_iq, i_q_rot);
    }
    else if (motor_test_status == MOTOR_TEST_CLOSED) {
        float base_target_we = BASELINE_REF_RPM * (6.2831853f / 60.0f) * POLE_PAIRS;
        float cur_ratio = (base_target_we > 0.1f) ? (smo.pll_speed / base_target_we) : 0.0f;
        float cur_rpm   = cur_ratio * BASELINE_REF_RPM;

        if (!closed_initialized) {
            overspeed_counter = 0U;
            memset((void*)&spd_loop, 0, sizeof(spd_loop));
            spd_loop_div_cnt = 0U;
            internal_ramped_ratio = 1.000f;
            soft_current_envelope = peak;
            soft_current_active = 0U;
            soft_current_entries = 0U;
            closed_initialized = 1U;
        }

        // Use the same max(abs(Iu), abs(Iv), abs(Iw)) as the hard trip.
        // Fast attack preserves peaks; slow decay avoids toggling at phase crossings.
        soft_current_envelope -= SOFT_ENVELOPE_DECAY_A;
        if (soft_current_envelope < peak) soft_current_envelope = peak;
        if (!soft_current_active && soft_current_envelope >= SOFT_CURRENT_ON_A) {
            soft_current_active = 1U;
            ++soft_current_entries;
        } else if (soft_current_active && soft_current_envelope <= SOFT_CURRENT_OFF_A) {
            soft_current_active = 0U;
        }

        // 目标转速平滑斜坡规划器 (Slew Rate Limiter)
        if (internal_ramped_ratio < internal_target_ratio) {
            if (!soft_current_active) internal_ramped_ratio += SPD_TARGET_SLEW_STEP;
            if (internal_ramped_ratio > internal_target_ratio) internal_ramped_ratio = internal_target_ratio;
        } else if (internal_ramped_ratio > internal_target_ratio) {
            internal_ramped_ratio -= SPD_TARGET_SLEW_STEP;
            if (internal_ramped_ratio < internal_target_ratio) internal_ramped_ratio = internal_target_ratio;
        }

        // 速度外环直接微调端压基座 (500Hz 周期调度)
        if (closed_ticks >= COORD_ROTATION_RAMP_TICKS) {
            if (++spd_loop_div_cnt >= SPD_LOOP_DIVIDER) {
                spd_loop_div_cnt = 0U;
                float err = internal_ramped_ratio - cur_ratio;
                spd_loop.p_term = SPD_TRIM_KP * err;

                if ((!soft_current_active || err <= 0.0f) &&
                    (!spd_loop.saturated || (spd_loop.last_output > 0.0f && err < 0.0f) || (spd_loop.last_output < 0.0f && err > 0.0f))) {
                    spd_loop.integral += (SPD_TRIM_KI * err) * SPD_LOOP_DT;
                    if (spd_loop.integral > SPD_TRIM_LIMIT) spd_loop.integral = SPD_TRIM_LIMIT;
                    if (spd_loop.integral < -SPD_TRIM_LIMIT) spd_loop.integral = -SPD_TRIM_LIMIT;
                }

                float raw_out = spd_loop.p_term + spd_loop.integral;
                if (raw_out > SPD_TRIM_LIMIT) {
                    raw_out = SPD_TRIM_LIMIT;
                    spd_loop.saturated = 1U;
                } else if (raw_out < -SPD_TRIM_LIMIT) {
                    raw_out = -SPD_TRIM_LIMIT;
                    spd_loop.saturated = 1U;
                } else {
                    spd_loop.saturated = 0U;
                }

                // Override positive pressure while constrained, even before PI saturation.
                if (soft_current_active) {
                    float ceiling = spd_loop.last_output - SOFT_TRIM_RETREAT_STEP;
                    if (ceiling < -SPD_TRIM_LIMIT) ceiling = -SPD_TRIM_LIMIT;
                    if (raw_out > ceiling) raw_out = ceiling;
                }

                // 变化率平滑限制
                float delta_trim = raw_out - spd_loop.last_output;
                if (delta_trim > SPD_TRIM_SLEW_MAX) {
                    spd_loop.last_output += SPD_TRIM_SLEW_MAX;
                } else if (delta_trim < -SPD_TRIM_SLEW_MAX) {
                    spd_loop.last_output -= SPD_TRIM_SLEW_MAX;
                } else {
                    spd_loop.last_output = raw_out;
                }
                if (soft_current_active) {
                    // Track the constrained output to avoid hidden positive windup.
                    float tracked_i = spd_loop.last_output - spd_loop.p_term;
                    if (tracked_i < -SPD_TRIM_LIMIT) tracked_i = -SPD_TRIM_LIMIT;
                    if (tracked_i > SPD_TRIM_LIMIT) tracked_i = SPD_TRIM_LIMIT;
                    spd_loop.integral = tracked_i;
                }
            }
        }

        // Calibrate only the 500..600 RPM interval using the ramped target.
        // Clip the correction weight: below 500 keep the original curve;
        // above 600 retain the original slope with a constant offset so no jump occurs.
        // Speeds outside the calibrated interval still require separate validation.
        float ff_weight = (internal_ramped_ratio - 1.0f) / 0.2f;
        if (ff_weight < 0.0f) ff_weight = 0.0f;
        if (ff_weight > 1.0f) ff_weight = 1.0f;
        float ff_correction = (CLOSED_FF_600_MOD - CLOSED_BASE_MOD_PER_RATIO * 1.2f) * ff_weight;
        float feedforward_vd = CLOSED_BASE_MOD_PER_RATIO * internal_ramped_ratio + ff_correction;
        float active_base_vd = feedforward_vd + spd_loop.last_output;
        float vd_rot_ff = active_base_vd * c_delta;
        float vq_rot_ff = active_base_vd * s_delta;

        // 叠加轻量高频阻尼
        float damp_d = -0.002f * i_d_rot;
        float damp_q = -0.002f * i_q_rot;
        if (damp_d >  DAMPING_LIMIT) damp_d =  DAMPING_LIMIT;
        if (damp_d < -DAMPING_LIMIT) damp_d = -DAMPING_LIMIT;
        if (damp_q >  DAMPING_LIMIT) damp_q =  DAMPING_LIMIT;
        if (damp_q < -DAMPING_LIMIT) damp_q = -DAMPING_LIMIT;

        Vd = vd_rot_ff + damp_d;
        Vq = vq_rot_ff + damp_q;

        // 动态超速保护
        float current_overspeed_bound = internal_ramped_ratio * OVERSPEED_RELATIVE_LIMIT;
        if (cur_ratio > current_overspeed_bound) {
            if (++overspeed_counter >= OVERSPEED_TICKS_LIMIT) {
                overspeed_snap.speed_ratio   = cur_ratio;
                overspeed_snap.peak_a        = peak;
                overspeed_snap.vd            = Vd;
                overspeed_snap.vq            = Vq;
                overspeed_snap.stage         = (uint8_t)motor_test_status;
                overspeed_snap.tripped_ticks = closed_ticks;
                Motor_Test_Trip(MOTOR_TEST_OVERSPEED);
                return;
            }
        } else {
            overspeed_counter = 0U; 
        }

        // 更新执行器实时遥测结构体
        actuator_telem.actual_rpm  = cur_rpm;
        actuator_telem.speed_ratio = cur_ratio;
        actuator_telem.target_rpm  = internal_ramped_ratio * BASELINE_REF_RPM;
        actuator_telem.i_u         = iu_uncomp;
        actuator_telem.i_d         = i_d_rot;
        actuator_telem.i_q         = i_q_rot;
        actuator_telem.base_vd     = active_base_vd;
        actuator_telem.v_trim      = spd_loop.last_output;
        actuator_telem.status      = (uint8_t)motor_test_status;
        actuator_telem.fault_code  = 0U;

        closed_ticks++;
    }

    Valpha = Vd * ct - Vq * st;
    Vbeta  = Vd * st + Vq * ct;

    // --- I. 滑模观测器 (SMO) 运行 ---
    float v_actual_scale = V_BUS * 0.57735027f;
    float v_alpha_volts  = Valpha * v_actual_scale;
    float v_beta_volts   = Vbeta  * v_actual_scale;
    SMO_Update(v_alpha_volts, v_beta_volts, i_alpha, i_beta, control_dt);

    SVPWM_Output_Standard(Valpha, Vbeta);

    if (!preflight) {
        float cur_rpm = (smo.pll_speed / ((BASELINE_REF_RPM * 6.2831853f / 60.0f) * POLE_PAIRS)) * BASELINE_REF_RPM;
        // CH0: Iu | CH1: actual_rpm | CH2: target_rpm | CH3: v_trim | CH4: Iq_rot
        Motor_Vofa_Record(iu_uncomp, cur_rpm, internal_ramped_ratio * BASELINE_REF_RPM, spd_loop.last_output, i_q_rot);
    }
}

//=============================================================================
// 6. 滑模观测器 (SMO) 实现
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

    float actual_step = final_angle - smo.last_angle_est;
    while (actual_step >  3.14159265f) actual_step -= 6.2831853f;
    while (actual_step < -3.14159265f) actual_step += 6.2831853f;
    smo.actual_step = actual_step;
    smo.last_angle_est = final_angle;

    smo.angle_est = final_angle;
}

//=============================================================================
// 7. 定时器与 PWM 硬件初始化 (PA11 专属交由 FDCAN1，彻底移除 TIM1 覆盖)
//=============================================================================
void TIM1_PWM_Init(u16 arr)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // 仅配置 PA8..PA10 (PWM三相上桥), PA12 (BKIN/驱动)，彻底排除 PA11 (CAN_RX)
    GPIO_Set(GPIOA, PIN8|PIN9|PIN10|PIN12, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_170M, GPIO_PUPD_PU); 
    GPIO_Set(GPIOB, PIN15, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_170M, GPIO_PUPD_PU); // 下桥 V 相
    // 注意: PB9 专属交由 FDCAN1_TX，不再在此处配置为通用定时器 AF
    GPIO_Set(GPIOC, PIN13, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_170M, GPIO_PUPD_PU); // 下桥 U 相

    GPIO_AF_Set(GPIOA, 8, 6);   // TIM1_CH1
    GPIO_AF_Set(GPIOA, 9, 6);   // TIM1_CH2
    GPIO_AF_Set(GPIOA, 10, 6);  // TIM1_CH3
    GPIO_AF_Set(GPIOC, 13, 4);  // TIM1_CH1N
    GPIO_AF_Set(GPIOA, 12, 6);  // TIM1_ETR/BKIN
    GPIO_AF_Set(GPIOB, 15, 4);  // TIM1_CH3N
    // PA11 保持 FDCAN1_RX (AF9)，禁止执行 GPIO_AF_Set(GPIOA, 11, 11)

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
// 8. SVPWM 算法
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
// 9. 电流环 PID 计算器
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
// 10. 电机启停与控制状态机
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
    Vd = 0.0f;
    Vq = 0.0f;
    Valpha = 0.0f;
    Vbeta  = 0.0f;
    actuator_telem.status = (uint8_t)MOTOR_TEST_STOPPED;
}

static void Motor_Test_Trip(MotorTestStatus reason)
{
    Motor_Stop();
    motor_fault_latched = 1U;
    motor_test_status = reason;
    actuator_telem.status = (uint8_t)reason;
    actuator_telem.fault_code = (uint8_t)reason;
}

void Motor_Test_SetReady(uint8_t ready)
{
    motor_test_ready = (ready != 0U);
}

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

    if (run_foc_flag != 0U ||   
       (motor_test_status != MOTOR_TEST_IDLE && motor_test_status != MOTOR_TEST_STOPPED)) {
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
    ADC2->ISR = ADC_ISR_JEOC | ADC_ISR_JEOC;

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
    overspeed_snap = (OverspeedSnapshot){0};
    overspeed_counter = 0U;

    open_loop_angle = 0.0f;
    elec_angle = 0.0f;
    open_loop_ticks = 0U;
    handoff_run_ticks = 0U;
    preflight_ticks = preflight_good = 0U;
    postzero_samples = postzero_overlimit = 0U;
    postzero_adc1_isr = postzero_adc2_isr = 0U;
    zero_observe_ticks = 0U;
    test_peak_a = 0.0f;

    handoff_delta_angle = 0.0f;
    closed_initialized = 0U;
    soft_current_envelope = 0.0f;
    soft_current_active = 0U;
    soft_current_entries = 0U;

    spd_loop_div_cnt = 0U;
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
        case MOTOR_TEST_CLOSED: message = "CLOSED LOOP (Standard_Actuator_Baseline)"; break;
        case MOTOR_TEST_DONE: message = "DONE: output OFF"; break;
        case MOTOR_TEST_STOPPED: message = "STOPPED: output OFF"; break;
        case MOTOR_TEST_ADC_FAULT: message = "FAULT: ADC/timing, reset required"; break;
        case MOTOR_TEST_ADC_RANGE: message = "FAULT: ADC near rail, reset required"; break;
        case MOTOR_TEST_OVERCURRENT: message = "FAULT: current limit (4.5A), reset required"; break;
        case MOTOR_TEST_OVERSPEED: message = "FAULT: estimated overspeed, reset required"; break;
        default: message = "IDLE"; break;
    }
    printf("[Actuator] %s; peak=%.3fA\r\n", message, (double)test_peak_a);
}

//=============================================================================
// 11. VOFA 通信协议与 DMA 发送
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
    
    DataLog[bank][at][0].f = ch0;          // CH0: Iu (A)
    DataLog[bank][at][1].f = ch1;          // CH1: actual_rpm
    DataLog[bank][at][2].f = ch2;          // CH2: target_rpm
    DataLog[bank][at][3].f = ch3;          // CH3: v_trim_spd
    DataLog[bank][at][4].f = ch4;          // CH4: Iq_rot (A)
    DataLog[bank][at][5].u = 0x7F800000U;  // JustFloat 结束符
    
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
        Zero_Stats_Report();
        Run_Diag_Report();
        if (static_align_active) Static_Align_Report();
    }
}

//=============================================================================
// 12. 统计与文本诊断报告
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
    for (uint32_t win = 0U; win < 2U; ++win) {
        const ZeroWindowStats *stats = windows[win];
        if (stats->count == 0U) continue;
        float pp[4];
        for (uint32_t ch = 0U; ch < 4U; ++ch) {
            means[win][ch] = ((float)stats->sum[ch] / (float)stats->count - offsets[ch]) *
                             INV_CURRENT_COUNTS_PER_AMP;
            pp[ch] = (float)(stats->max[ch] - stats->min[ch]) * fabsf(INV_CURRENT_COUNTS_PER_AMP);
        }
        printf("[ZeroCompare] %s mean U=%.4f V=%.4f W=%.4f SUM=%.4f p-p=%.4f\r\n",
               names[win], (double)means[win][0], (double)means[win][1],
               (double)means[win][2], (double)means[win][3], (double)pp[3]);
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
    while (step >  3.14159265f) step -= 6.2831853f;
    while (step < -3.14159265f) step += 6.2831853f;

    float error = fabsf(step - expected_step);
    if (!Diag_Finite(current_angle) || !Diag_Finite(error) || error > 0.0001f)
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
    if (norm_error > 0.001f || (run_diag.have_trig && change > fabsf(expected_step)+0.001f))
        ++run_diag.trig_bad;
    run_diag.previous_s = s; run_diag.previous_c = c; run_diag.have_trig = 1U;
}

static void Static_Align_Report(void)
{
    printf("[StaticAlign] angle=%.4frad target_Id=%.2fA hold=1s\r\n",
           (double)STATIC_ALIGN_TARGET_ANGLE, (double)STATIC_ALIGN_TARGET_ID);
    if (!static_align_stats.count) return;
    float offsets[3] = {offset_u, offset_v, offset_w};
    float mean[3];
    for (unsigned i=0; i<3; ++i) {
        mean[i] = ((float)static_align_stats.sum[i] / (float)static_align_stats.count - offsets[i]) *
                  INV_CURRENT_COUNTS_PER_AMP;
    }
    printf("[StaticAlign] mean U=%.4f V=%.4f W=%.4f A uncompensated\r\n",
           (double)mean[0], (double)mean[1], (double)mean[2]);
}

static void Run_Diag_Report(void)
{
    float err_pct = (actuator_telem.target_rpm > 1.0f) ?
                    ((actuator_telem.actual_rpm - actuator_telem.target_rpm) / actuator_telem.target_rpm * 100.0f) : 0.0f;

    printf("[ActuatorDiag] Telemetry State: actual_rpm=%.1f target_rpm=%.1f (SteadyErr=%+.2f%%) Iq=%.3fA base_vd=%.4f trim=%.5f Sat=%u\r\n",
           (double)actuator_telem.actual_rpm, (double)actuator_telem.target_rpm, (double)err_pct,
           (double)actuator_telem.i_q, (double)actuator_telem.base_vd, (double)actuator_telem.v_trim, (unsigned int)spd_loop.saturated);
    printf("[RunDiag] calls=%lu body_max=%lucyc over_budget=%lu angle_bad=%lu\r\n",
           (unsigned long)run_diag.calls, (unsigned long)run_diag.max_cycles,
           (unsigned long)run_diag.over_budget, (unsigned long)run_diag.angle_bad);
}