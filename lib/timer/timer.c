#include "timer.h"
// #include "as5047p.h"
#include "led.h"
#include <math.h>
#include "align.h"
#include "control.h"
#include "math.h"
#include "adc_foc.h"
#include <stdio.h>
#include "usart.h"
//-----------------------------------------------------
// --- 宏定义与常量 ---

//-------------------------------------------------
// 双缓冲区
VofaData_t DataLog[2][SAMPLE_NUM][FRAME_SIZE];

volatile uint8_t  write_bank = 0;    // 当前正在写入哪个缓冲 (0 或 1)
volatile uint32_t write_ptr = 0;     // 写入指针
volatile uint8_t  bank_ready = 0xFF; // 哪一个缓冲准备好了发送 (0, 1 或 0xFF表示无)
volatile uint8_t  is_dma_busy = 0;   // DMA 发送状态
//-----------------------------------------------
// 外部函数声明
//-----------------------------------------------
float open_loop_angle = 0.0f;
//-----------------------------------------------
//外部变量声明
//-----------------------------------------------
extern PID_Controller vel_pid;
extern int16_t my_zero_offset;// 14位机械偏置 (0-16383)
extern float offset_u, offset_v, offset_w;

// 用于主循环打印的调试变量
extern volatile float debug_id;
extern volatile float debug_iq;
extern volatile float debug_Vq;
extern volatile float debug_Vd;

extern volatile float debug_iu;
extern volatile float debug_iv;
extern volatile float debug_iw;

volatile float debug_raw_u;
volatile float debug_raw_v;
volatile float debug_raw_w;

volatile float debug_iu_filt;
volatile float debug_iv_filt;
volatile float debug_iw_filt;

volatile float debug_Ialpha;
volatile float debug_Ibeta;
volatile float debug_iq_filt;
volatile float debug_id_filt;
volatile float debug_elec_angle;
volatile float debug_Valpha;
volatile float debug_Vbeta;

volatile float debug_speed_act = 0;
volatile float debug_iq_target = 0;
//-----------------------------------------------
extern PID_Controller pid_id;
extern PID_Controller pid_iq;
extern PID_Controller pid_speed;
extern PID_Controller pid_pos;  

extern volatile uint8_t run_foc_flag;
//-----------------------------------------------
// 变量声明
//-----------------------------------------------
u16 Timer1_Counter = 0;
//--------------------------------------------
float i_alpha, i_beta;      // Clark 变换结果
float i_d, i_q;            // Park 变换结果
float theta_elec = 0.0f;          // 电角度 (弧度)
float sin_t, cos_t;        // 角度的正余弦值

float elec_angle;

int16_t raw_diff;

// --- 变量定义 ---
float target_speed = 0.0f;      // 目标速度 (单位: rad/s)
float actual_speed_filt = 0.0f; // 实际速度 (滤波后)
float target_iq = 0.0f;         // 速度环的输出，作为电流环的输入
float target_id = 0.0f;      
float target_pos;       // 目标位置 (rad)

static float i_d_f = 0.0f, i_q_f = 0.0f; // 电流滤波变量

volatile float Vd = 0.0f;
volatile float Vq = 0.0f;
volatile float force_angle = 0.0f; // 强制输出时的电角度
volatile float Valpha;
volatile float Vbeta;

float open_loop_init_angle = 0.0f;
float open_loop_Vq = 0.2f;     // 给 1.0V 电压，根据你的母线电压调整

float actual_pos_rad = 0.0f;            // 实际累加位置 (rad, 多圈)
float last_mech_angle_for_pos = 0.0f;   // 用于计算位置增量的上一次角度

// --- 慢速旋转控制变量 ---
float move_speed_deg_per_s = 100.0f; // 设定转速：每秒转多少度 (根据需要修改)
uint8_t slow_rotation_en = 1;       // 慢速旋转使能开关

// --- 往复测试变量 ---
uint8_t test_mode_en = 0;    // 测试模式使能
uint8_t test_state = 0;      // 0: 停止, 1: 正转中, 2: 反转中
float test_start_pos = 0;    // 起始位置
float test_target_range = 31.4159f; // 5圈的弧度
float move_step = 0.1f;     // 每毫秒移动的步长（控制旋转速度）
//-----------------------------------------------
//==============================================
// TIM1 更新中断服务函数
//==============================================
void TIM1_UP_TIM16_IRQHandler(void)
{
    if(TIM1->SR & TIM_SR_UIF)
    {
        // 立即清除中断标志
        TIM1->SR &= ~TIM_SR_UIF;
        Timer1_Counter++;

        // 1. 获取 14 位机械角度
        uint16_t raw_val = (uint16_t)(TIM4->CNT) & ENCODER_MASK;

        // 2. 计算机械角度差（利用无符号数截断特性，无需 if 处理负数）
        uint16_t mech_diff = (raw_val - (uint16_t)my_zero_offset) & ENCODER_MASK;

        // 3. 计算电角度分量 (0 - 16383)
        uint16_t elec_diff = (mech_diff * POLE_PAIRS) & ENCODER_MASK;

        // 4. 转换为弧度 (0 - 2*PI)
        float elec_angle = (float)elec_diff * 0.000383495197f; // 0.000383495197f = 2*PI / 16384

        debug_elec_angle = elec_angle;

        // 机械角度弧度制 (用于速度环)
        float current_mech_rad = (float)mech_diff * (6.2831853f / 16384.0f);

        // 电流环逻辑 (15kHz)
        // 1. 计算原始偏差（ADC值 - 你的静态偏置）
        int32_t raw_u = (int32_t)(ADC1->JDR1 & 0x0FFF) - (int32_t)offset_u;
        int32_t raw_v = (int32_t)(ADC2->JDR1 & 0x0FFF) - (int32_t)offset_v;
        int32_t raw_w = (int32_t)(ADC2->JDR2 & 0x0FFF) - (int32_t)offset_w;

        // 必须同时清除 ADC1 和 ADC2 的标志位
        ADC1->ISR |= ADC_ISR_JEOS; 
        ADC2->ISR |= ADC_ISR_JEOS;
        
        debug_raw_u = raw_u;
        debug_raw_v = raw_v;
        debug_raw_w = raw_w;

        // 零序分量补偿（消除采样偏置漂移）
        const float LSB_PER_AMP = -10.0f; 
        float com = (raw_u + raw_v + raw_w) * 0.3333333f;
        float iu = ((float)raw_u - com) / LSB_PER_AMP;
        float iv = ((float)raw_v - com) / LSB_PER_AMP;
        float iw = ((float)raw_w - com) / LSB_PER_AMP;

        debug_iu = iu;
        debug_iv = iv;
        debug_iw = iw;
    //=============================================================================
        // 速度环逻辑：15分频 (运行频率 1kHz)
        // static uint16_t speed_cnt = 0;
        // static float last_mech_angle = 0.0f;
        // speed_cnt++;
        // if (speed_cnt >= 15) 
        // {
        //     speed_cnt = 0;

        //     // 1. 计算角度差
        //     float delta_angle = current_mech_rad - last_mech_angle;
            
        //     // 2. 环形边界处理
        //     if (delta_angle >  3.1415926f) delta_angle -= 6.2831853f;
        //     else if (delta_angle < -3.1415926f) delta_angle += 6.2831853f;

        //     // 3. 计算速度 (rad/s)
        //     float instant_speed = delta_angle * 1000.0f; // rad/s

        //     // 4. 速度滤波 (0.03 较平滑，若响应太慢可改至 0.05-0.1)
        //     actual_speed_filt = (instant_speed * 0.05f) + (actual_speed_filt * 0.95f);
            
        //     // 5. 更新历史位置
        //     last_mech_angle = current_mech_rad;

        //     if (run_foc_flag) 
        //     {
        //         target_iq = PID_Calc_Speed(&pid_speed, target_speed, actual_speed_filt);
        //         target_id = 0.0f;
        //     }
        // }
    //=================================================================================
        // 3. 【外环逻辑】位置环 + 速度环 (15分频 = 1kHz)
        static uint16_t outer_loop_cnt = 0;
        outer_loop_cnt++;
        if (outer_loop_cnt >= 15) 
        {
            outer_loop_cnt = 0;

            // A. 位置累加 (处理 0~2PI 跳变，实现多圈)
            float delta_pos = current_mech_rad - last_mech_angle_for_pos;
            if (delta_pos >  3.1415926f) delta_pos -= 6.2831853f;
            else if (delta_pos < -3.1415926f) delta_pos += 6.2831853f;
            actual_pos_rad += delta_pos; 
            last_mech_angle_for_pos = current_mech_rad;

            // B. 计算速度 (rad/s)
            float instant_speed = delta_pos * 1000.0f; 
            actual_speed_filt = (instant_speed * 0.05f) + (actual_speed_filt * 0.95f); // 低通滤波

            // B. 自动测试状态机
            if (run_foc_flag && test_mode_en) 
            {
                switch (test_state) 
                {
                    case 1: // 正转阶段
                        target_pos += move_step;
                        if (target_pos >= test_start_pos + test_target_range) {
                            test_state = 2; // 到达5圈，切换反转
                        }
                        break;

                    case 2: // 反转阶段
                        target_pos -= move_step;
                        if (target_pos <= test_start_pos) {
                            test_state = 1; // 回到起点，切换正转
                        }
                        break;
                }
            }
            else if (run_foc_flag && !test_mode_en) //if (run_foc_flag) 
            {
                // ================== 新增：慢速旋转目标生成 ==================
                if(slow_rotation_en) 
                {
                    // 将 deg/s 转换为 rad/ms (因为控制周期是 1ms)
                    // 公式：速度 * (PI/180) * 0.001
                    float step = move_speed_deg_per_s * 0.0174533f * 0.001f;
                    target_pos += step; 
                }
                // ==========================================================

               
            }
            // else
            // {
            //     // 当电机未启动时，让目标位置跟随当前位置，防止启动瞬间“弹射”
            //     target_pos = actual_pos_rad;
            //     PID_Reset(&pid_pos); // 建议增加一个重置积分的函数
            //     PID_Reset(&pid_speed);
            // }
            // === 位置环控制 ===
            target_speed = PID_Calc_Pos(&pid_pos, target_pos, actual_pos_rad);

            // === 速度环控制 ===
            target_iq = PID_Calc_Speed(&pid_speed, target_speed, actual_speed_filt);
            target_id = 0.0f;
        }
    //=================================================================================
        // === zero offset 0位对齐 ===
        // elec_angle = 0.0f;          // U相测试 Vd=0.5, Vq=0 对应电角度 0度 (0.0f)
        // elec_angle = 2.094395f;     // V相测试 Vd=0.5, Vq=0 对应电角度 120度 (2.094395f)
        // elec_angle = 4.1887902f;    // W相测试 Vd=0.5, Vq=0 对应电角度 240度 (4.1887902f或-2.094395f)

        // 动态测试，IU，Iv，Iw 验证线序，应该是正弦波，出峰先U再V然后W
        // open_loop_angle += 0.0002f; // 步进值，控制旋转速度
        // if (open_loop_angle > 6.2831853f) open_loop_angle -= 6.2831853f;
        // if (open_loop_angle < 0.0f)       open_loop_angle += 6.2831853f; 

        // 使用 CORDIC 计算 Sin/Cos
        float st, ct;
        CORDIC_SinCos(elec_angle, &st, &ct);
        // CORDIC_SinCos(open_loop_angle, &st, &ct);

        // Clark & Park 变换
        // float i_alpha = iu;
        // float i_beta = (iv - iw) * 0.5773503f;
        float i_alpha = iu;
        float i_beta = (iv - iw) * 0.57735027f;

        debug_Ialpha = i_alpha;
        debug_Ibeta = i_beta;

        // Park 变换
        float i_d = i_alpha * ct + i_beta * st;
        float i_q = -i_alpha * st + i_beta * ct;

        debug_id = i_d;
        debug_iq = i_q;

        // 电流低通滤波 
        i_d_f = (i_d * 0.02f) + (i_d_f * 0.98f);
        i_q_f = (i_q * 0.02f) + (i_q_f * 0.98f);

        debug_iq_filt = i_q_f;
        debug_id_filt = i_d_f;

        // --- 电流环 PID ---
        if (run_foc_flag == 1) 
        {
            // ===== PID 计算 Vd, Vq ======
            Vd = PID_Calc_Current(&pid_id, target_id, i_d_f);
            Vq = PID_Calc_Current(&pid_iq, target_iq, i_q_f);

            // Vd = Vd*0.5f;
            // Vq = Vq*0.5f;
            // ===== 开环旋转逻辑 判断电角度与旋转方向的正确性，调整pwm输出的相序 ========
            // open_loop_angle += 0.0005f; // 步进值，控制旋转速度
            // if(open_loop_angle > 6.2831853f) open_loop_angle -= 6.2831853f;

            // // 开环验证模式下，强制使用开环角度计算 Sin/Cos
            // CORDIC_SinCos(open_loop_angle, &st, &ct);
            // Vd = 0.0f;
            // Vq = 0.6f;       
        }
        else 
        {
            Vd = 1.0f; 
            Vq = 0.0f;
            // ======= 强制模式：预对齐 =======
            // Vd = 0.8f*0.5f; 
            // Vq = 0.0f*0.5f;
        }

        debug_Vq = Vq;
        debug_Vd = Vd;

        //====== 逆变换与输出 ==========
        // 逆 Park 变换
        Valpha = Vd * ct - Vq * st;
        Vbeta  = Vd * st + Vq * ct;

        debug_Valpha = Valpha;
        debug_Vbeta = Vbeta;
        
        // SVPWM 调制输出到寄存器
        SVPWM_Output_Standard(Valpha, Vbeta);
        
        debug_speed_act = actual_speed_filt;
        debug_iq_target = target_iq;

        // --- 5. 数据记录逻辑 (增加 10 倍下采样) ---
        static uint16_t log_downsample_cnt = 0; // 静态计数器
        log_downsample_cnt++;

        if (log_downsample_cnt >= 10) { // 每 10 次中断记录一次
            log_downsample_cnt = 0;     // 重置计数器

            // 如果当前缓冲区没在发送，就记录
            if (bank_ready != write_bank) {
                // DataLog[write_bank][write_ptr][0].f = debug_raw_u;
                // DataLog[write_bank][write_ptr][1].f = debug_raw_v;
                // DataLog[write_bank][write_ptr][2].f = debug_raw_w;

                // DataLog[write_bank][write_ptr][0].f = debug_iu;
                // DataLog[write_bank][write_ptr][1].f = debug_iv;
                // DataLog[write_bank][write_ptr][2].f = debug_iw;

                // DataLog[write_bank][write_ptr][0].f = TIM1->CCR1;
                // DataLog[write_bank][write_ptr][1].f = TIM1->CCR2;
                // DataLog[write_bank][write_ptr][2].f = TIM1->CCR3;

                DataLog[write_bank][write_ptr][0].f = debug_iq_filt; 
                DataLog[write_bank][write_ptr][1].f = debug_id_filt; 
                DataLog[write_bank][write_ptr][2].f = debug_Vq; 

                DataLog[write_bank][write_ptr][3].u = 0x7F800000;      // VOFA 结束符
                
                write_ptr++;
                
                if (write_ptr >= SAMPLE_NUM) {
                    bank_ready = write_bank;  // 标记当前 Bank 已满
                    write_bank = !write_bank; // 切换 Bank
                    write_ptr = 0;            // 重置指针
                }
            }
        }
        // LED 翻转逻辑
        if (Timer1_Counter >= 1500) 
        { // 15kHz下，7500次是500ms
            Timer1_Counter = 0;
            LED0_TOGGLE();
            // printf("elec_angle:%.2f\r\n", debug_elec_angle);
            // printf("CNT:%d|mech_diff:%d\r\n", TIM4->CNT,mech_diff);
            // printf("Target:%.2f, Act:%.2f\r\n",target_speed, actual_speed_filt);
            // printf("current_mech_rad:%.2f\r\n", current_mech_rad);
            // printf("actual_speed:%.2f, target_iq:%.2f\r\n", actual_speed_filt, target_iq);
        }
    }
}
//==============================================
// @brief  初始化 TIM1 产生中心对齐 PWM (避坑版)
// @param  arr: 自动重装载值 (170MHz下, 5666对应15kHz)
// @param  psc: 分频系数 (通常为0)
//==============================================
void TIM1_PWM_Init(u16 arr)
{
    // 1. 时钟开启
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    GPIO_Set(GPIOA,PIN8|PIN9|PIN10|PIN11|PIN12,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_170M,GPIO_PUPD_PU); 
    GPIO_Set(GPIOB,PIN9|PIN15,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_170M,GPIO_PUPD_PU); 
    GPIO_Set(GPIOC,PIN13,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_170M,GPIO_PUPD_PU); 

    // 3. 复位 AF 映射 (AF6 为 TIM1)
    GPIO_AF_Set(GPIOA, 8, 6); 
    GPIO_AF_Set(GPIOA, 9, 6); 
    GPIO_AF_Set(GPIOA, 10, 6);
    GPIO_AF_Set(GPIOC, 13, 4); 
    GPIO_AF_Set(GPIOA, 12, 6); 
    GPIO_AF_Set(GPIOB, 15, 4);

    //PA11 才是 TIM1_CH4，AF 映射为 6 TIM1_CH4用于示波器监测CCR4
    GPIO_AF_Set(GPIOA, 11, 11);  // PA11 AF6 = TIM1_CH4

    // 4. 基础设置
    TIM1->ARR = arr;
    TIM1->PSC = 0;
    TIM1->RCR = 1; // 确保在波谷触发 Update 事件

    // 6. 中心对齐模式 1
    TIM1->CR1 &= ~TIM_CR1_CMS;
    TIM1->CR1 |= (1 << TIM_CR1_CMS_Pos);

    // 7. PWM 模式 1 配置
    TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M);
    TIM1->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM1->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 &= ~TIM_CCMR2_OC3M;
    TIM1->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;

    // --- 配置 CH4 作为 ADC 触发源 ---
    // TIM1->CCMR2 &= ~TIM_CCMR2_OC4M;
    // TIM1->CCMR2 |= (6 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE; // CH4 同样设为 PWM 模式 1

    //---示波器检测，如跳出ADC采样观测，屏蔽下面两条语句，恢复上面两条语句
    // --- 【修改点 2】 配置 CH4 作为示波器观测和 ADC 触发 ---
    // 建议改为 PWM 模式 2，这样在靠近 ARR (顶点) 时，PA11 会输出一个高电平窄脉冲
    TIM1->CCMR2 &= ~(TIM_CCMR2_OC4M | (1U << 24));
    // TIM1->CCMR2 |= (7 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE; // 7 = PWM Mode 2
    TIM1->CCMR2 |= (7U << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;

    // 初始触发位置：距离顶点 50 个计数值
    // 这样 ADC 会在 CNT 向上计数到 (arr-50) 时触发，留给 ADC 转换一定的硬件建立时间
    TIM1->CCR4 = arr - 100;

    // --- 【映射 TRGO2 到 OC4REF】 ---
    // TIM1->CR2 &= ~TIM_CR2_MMS2;
    // TIM1->CR2 |= (4U << 20);    // 0100: OC4REF 信号映射到 TRGO2     

    // 8. 使能输出
    TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE | 
                   TIM_CCER_CC2E | TIM_CCER_CC2NE | 
                   TIM_CCER_CC3E | TIM_CCER_CC3NE | 
                   TIM_CCER_CC4E);

    // 9. 死区与主输出
    TIM1->BDTR &= ~TIM_BDTR_DTG;
    TIM1->BDTR |= 100; 
    TIM1->BDTR |= TIM_BDTR_MOE; 

    // 11. 启动
    TIM1->EGR |= TIM_EGR_UG;   // 更新寄存器
    TIM1->CR1 |= TIM_CR1_CEN;
    
    // 12. 中断配置
    NVIC_SetPriority(TIM1_UP_TIM16_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));  // 抢占优先级 0，子优先级 0
    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
}

void SVPWM_Output_Standard(float Valpha, float Vbeta)
{
    // 1. 模长限幅 (SVPWM 最大不失调模长为 1.1547)
    float v_sq = Valpha * Valpha + Vbeta * Vbeta;
    if (v_sq > 1.333f) { 
        float inv_mod = 1.0f / sqrtf(v_sq);
        Valpha *= (inv_mod * 1.154f);
        Vbeta  *= (inv_mod * 1.154f);
    }

    // 2. 标准反克拉克变换 (修正 V/W 定义)
    // U 相位于 0°，V 相位于 120°，W 相位于 240°
    float U_u = Valpha;
    float U_v = -0.5f * Valpha + 0.8660254f * Vbeta; // 修正：V相 Beta 项为正
    float U_w = -0.5f * Valpha - 0.8660254f * Vbeta; // 修正：W相 Beta 项为负

    // 3. 寻找极值
    float max_v = U_u, min_v = U_u;
    if (U_v > max_v) max_v = U_v; if (U_v < min_v) min_v = U_v;
    if (U_w > max_v) max_v = U_w; if (U_w < min_v) min_v = U_w;

    // 4. 注入零序分量
    float V_offset = (max_v + min_v) * 0.5f;

    // 5. 归一化映射 (你的 0.433 算法非常高效，予以保留)
    float Ta = (U_u - V_offset + 1.154f) * 0.433f; 
    float Tb = (U_v - V_offset + 1.154f) * 0.433f;
    float Tc = (U_w - V_offset + 1.154f) * 0.433f;

    // 6. 转换为计数值，并预留足够的低边导通时间用于采样 (Dead-time/Sampling Window)
    uint16_t max_ccr = PWM_ARR - 200; 
    uint16_t min_ccr = 200; // 必须预留最小关闭时间以保证 ADC 采样稳定

    // 6. 转换为计数值并预留采样窗口
    uint16_t ccr_u = (uint16_t)(Ta * PWM_ARR);
    uint16_t ccr_v = (uint16_t)(Tb * PWM_ARR);
    uint16_t ccr_w = (uint16_t)(Tc * PWM_ARR);

    if (ccr_u > max_ccr) ccr_u = max_ccr;
    if (ccr_v > max_ccr) ccr_v = max_ccr;
    if (ccr_w > max_ccr) ccr_w = max_ccr;

    // if (ccr_u < min_ccr) ccr_u = min_ccr;
    // if (ccr_v < min_ccr) ccr_v = min_ccr;
    // if (ccr_w < min_ccr) ccr_w = min_ccr;

    // 7. 写入寄存器
    TIM1->CCR1 = ccr_u;
    TIM1->CCR2 = ccr_v;
    TIM1->CCR3 = ccr_w;

    // 8. 更新采样点 (CCR4 保持在波谷附近)
    TIM1->CCR4 = (uint16_t)(PWM_ARR - 100);
}

/**
 * @brief 抗饱和电流环 PID 计算
 */
float PID_Calc_Current(PID_Controller* pid, float target, float current)
{
    float error = target - current;
    float dt = 0.0000666f; // 15kHz

    // 1. 计算比例项
    float p_term = pid->kp * error;

    // 2. 预计算输出（用于抗饱和判定）
    float next_integral = pid->integral + (error * pid->ki * dt);
    float total_out = p_term + next_integral;

    // 3. 动态抗饱和条件：
    // 只有当输出未饱和，或者误差方向有助于脱离饱和时，才累加积分
    if (total_out > -pid->output_limit && total_out < pid->output_limit) {
        pid->integral = next_integral;
    }

    // 4. 积分项独立限幅（二次保险）
    if (pid->integral > pid->output_limit) pid->integral = pid->output_limit;
    else if (pid->integral < -pid->output_limit) pid->integral = -pid->output_limit;

    // 5. 最终输出限幅
    float out = p_term + pid->integral;
    if (out > pid->output_limit) out = pid->output_limit;
    else if (out < -pid->output_limit) out = -pid->output_limit;

    return out;
}

float PID_Calc_Speed(PID_Controller* pid, float target, float current) 
{
    float error = target - current;
    float dt = 0.001f;  
        // 1. 计算 P 项
    float p_term = pid->kp * error;

    // 2. 计算 I 项（只有在输出未饱和时才累加，或者直接限幅）
    pid->integral += error * pid->ki * dt;

    // 限制积分项分量本身，通常设为 output_limit 的一半或全部
    if (pid->integral > pid->output_limit) pid->integral = pid->output_limit;
    if (pid->integral < -pid->output_limit) pid->integral = -pid->output_limit;

    // 3. 总输出
    float out = p_term + pid->integral;

    // 4. 总输出限幅
    if (out > pid->output_limit) out = pid->output_limit;
    if (out < -pid->output_limit) out = -pid->output_limit;
    
    return out;
}

/**
 * @brief 位置环 PID 计算
 */
float PID_Calc_Pos(PID_Controller* pid, float target, float current) {
    // 1. 计算位置偏差
    float error = target - current;

    // 2. 比例项 (P)
    float p_term = pid->kp * error;

    // 3. 积分项 (I) - 通常为 0
    pid->integral += error * pid->ki * 0.001f; // 1ms 周期

    // 4. 总输出计算
    float out = p_term + pid->integral;

    // 5. 输出限幅 (限制最大寻道速度)
    if (out > pid->output_limit) out = pid->output_limit;
    else if (out < -pid->output_limit) out = -pid->output_limit;

    return out;
}

void Motor_OpenLoop_Tick(void) {
    // 1. 累加角度
    open_loop_init_angle += 0.01f; // 步进值，控制旋转速度
    if (open_loop_init_angle > 6.283185f) open_loop_init_angle -= 6.283185f;

    // 2. 简单的反 Park / Space Vector (这里演示最简单的 Sine PWM)
    float Ua = -open_loop_Vq * sinf(open_loop_init_angle);
    float Ub =  open_loop_Vq * cosf(open_loop_init_angle);
    
    SVPWM_Output_Standard(Ua, Ub); 
}

/**
 * @brief 重置 PID 控制器状态
 * @param pid: 指向 PID 结构体的指针
 */
void PID_Reset(PID_Controller* pid) 
{
    pid->integral = 0.0f;
    // 如果你的结构体中有 last_error (用于 D 项计算)，也应在此处清零
    // pid->last_error = 0.0f; 
}

