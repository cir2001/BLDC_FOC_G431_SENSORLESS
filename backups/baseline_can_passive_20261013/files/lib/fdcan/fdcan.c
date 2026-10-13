#include "fdcan.h"
#include "timer.h"
#include <stm32g4xx.h>
#include <string.h>
#include <stdio.h>

// STM32G4 官方标准消息 RAM 布局：
// Rx FIFO 0: 字节偏移 0x0B0 -> 0x2C Words (44 Words)
// Tx FIFO/Queue: 字节偏移 0x278 -> 0x9E Words (158 Words)
#define SRAMCAN_RF0_OFFSET_WORDS    (0x2CU)
#define SRAMCAN_TF_OFFSET_WORDS     (0x9EU)
#define MSG_ELEMENT_SIZE_WORDS      (18U)   // 每个元素 18 个 32 位字

// 170MHz 主频时钟常数与物理超时周期 (50ms = 8,500,000 cycles, 1ms = 170,000 cycles)
#define CYCLES_PER_MS               (170000U)
#define FDCAN_WATCHDOG_CYCLES       (50U * CYCLES_PER_MS)
#define TELEMETRY_INTERVAL_CYCLES   (1U * CYCLES_PER_MS)

// 通信状态机与硬件周期看门狗变量
static volatile uint32_t last_rx_cycles = 0U;
static volatile uint8_t  fdcan_connected = 0U;
static volatile uint8_t  watchdog_latched = 0U;
static volatile uint8_t  last_rx_seq = 0U;
static volatile uint8_t  first_seq_received = 0U;

// Command history is updated/read in the main-loop CAN service only.
static uint32_t accepted_count = 0U;
static uint32_t target_write_count = 0U;
static uint8_t last_accepted_mode = 0U;
static int16_t last_accepted_rpm = 0;
static volatile uint8_t diag_requested = 0U;

// 1kHz 遥测发送时钟记录
static uint32_t last_tx_cycles = 0U;

//=============================================================================
// 1. FDCAN1 硬件外设与收发器使能 (PA11=RX, PB9=TX, PC11=CAN_SHDN)
//    经典 CAN 1 Mbps (时钟源: PCLK1 = 170 MHz)
//=============================================================================
void FDCAN1_Init(void)
{
    // 1. 开启 GPIOA, GPIOB, GPIOC 与 FDCAN 外设时钟
    RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN);
    RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;

    // 选择 FDCAN 时钟源为 PCLK1 (170 MHz)
    RCC->CCIPR &= ~RCC_CCIPR_FDCANSEL;
    RCC->CCIPR |= (2U << RCC_CCIPR_FDCANSEL_Pos); // 10: PCLK1

    // 2. 引脚配置
    // PA11 (FDCAN1_RX, AF9)
    GPIOA->MODER &= ~GPIO_MODER_MODE11;
    GPIOA->MODER |= (2U << GPIO_MODER_MODE11_Pos);
    GPIOA->AFR[1] &= ~(0x0FU << ((11 - 8) * 4));
    GPIOA->AFR[1] |= (9U << ((11 - 8) * 4)); // AF9
    GPIOA->PUPDR  &= ~GPIO_PUPDR_PUPD11;

    // PB9 (FDCAN1_TX, AF9)
    GPIOB->MODER &= ~GPIO_MODER_MODE9;
    GPIOB->MODER |= (2U << GPIO_MODER_MODE9_Pos);
    GPIOB->AFR[1] &= ~(0x0FU << ((9 - 8) * 4));
    GPIOB->AFR[1] |= (9U << ((9 - 8) * 4)); // AF9
    GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED9;
    GPIOB->PUPDR   &= ~GPIO_PUPDR_PUPD9;

    // PC11 (CAN_SHDN, 推挽输出, 拉低以使能板载收发器)
    GPIOC->MODER &= ~GPIO_MODER_MODE11;
    GPIOC->MODER |= (1U << GPIO_MODER_MODE11_Pos);
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT11;
    GPIOC->BSRR = GPIO_BSRR_BR11; // 写入 0 唤醒收发器

    // 3. 进入 FDCAN 初始化模式并解锁保护
    FDCAN1->CCCR |= FDCAN_CCCR_INIT;
    while ((FDCAN1->CCCR & FDCAN_CCCR_INIT) == 0U);
    FDCAN1->CCCR |= FDCAN_CCCR_CCE;

    // 4. 配置波特率 (经典 CAN 1.000 Mbps)
    // 170MHz / (10 * (1 + 12 + 4)) = 170MHz / 170 = 1.000 Mbps, 采样点 76.5%
    FDCAN1->NBTP = ((10U - 1U) << FDCAN_NBTP_NBRP_Pos) |
                   ((12U - 1U) << FDCAN_NBTP_NTSEG1_Pos)|
                   ((4U  - 1U) << FDCAN_NBTP_NTSEG2_Pos)|
                   ((4U  - 1U) << FDCAN_NBTP_NSJW_Pos);

    // 5. 帧格式配置：首轮关闭 FDOE 与 BRSE，运行标准经典 CAN
    FDCAN1->CCCR &= ~(FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE);

    // 6. 配置 Tx FIFO 工作模式 (队列模式 TFQM = 0)
    FDCAN1->TXBC &= ~FDCAN_TXBC_TFQM;

    // 7. 配置全局过滤寄存器 RXGFC (接收未匹配的标准帧到 Rx FIFO 0)
    FDCAN1->RXGFC &= ~FDCAN_RXGFC_ANFS; // 00: Accept in Rx FIFO 0

    // 8. 退出初始化模式
    FDCAN1->CCCR &= ~FDCAN_CCCR_CCE;
    FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;
    while ((FDCAN1->CCCR & FDCAN_CCCR_INIT) != 0U);

    fdcan_connected = 0U;
    watchdog_latched = 0U;
    first_seq_received = 0U;
    accepted_count = 0U;
    target_write_count = 0U;
    last_accepted_mode = 0U;
    last_accepted_rpm = 0;
    last_rx_seq = 0U;
    diag_requested = 0U;
    last_rx_cycles = DWT->CYCCNT;
    last_tx_cycles = DWT->CYCCNT;
}

//=============================================================================
// 2. 下行控制帧解析 (严格校验帧类型、DLC、模式与新鲜度)
//=============================================================================
static void FDCAN1_Process_Rx_Message(uint32_t now_cycles)
{
    if ((FDCAN1->RXF0S & FDCAN_RXF0S_F0FL) == 0U) return;

    uint8_t get_idx = (uint8_t)((FDCAN1->RXF0S & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos);
    volatile uint32_t *rx_buf = (volatile uint32_t *)(SRAMCAN_BASE + ((SRAMCAN_RF0_OFFSET_WORDS + (get_idx * MSG_ELEMENT_SIZE_WORDS)) * 4U));

    uint32_t id_word = rx_buf[0];
    uint32_t dl_word = rx_buf[1];

    // 必须为标准 11-bit 帧且非远程帧 (RTR=0, XTD=0)
    if ((id_word & (1U << 30)) == 0U && (id_word & (1U << 29)) == 0U) {
        uint32_t id  = (id_word >> 18U) & 0x7FFU;
        uint8_t  dlc = (uint8_t)((dl_word >> 16U) & 0x0FU);

        // 严格比对 ID=0x201 且 DLC=8
        if (id == FDCAN_CMD_ID && dlc == 8U) {
            uint32_t data_low  = rx_buf[2];

            uint8_t cmd_mode = (uint8_t)(data_low & 0xFFU);
            uint8_t seq_id   = (uint8_t)((data_low >> 8U) & 0xFFU);
            int16_t target_rpm_raw = (int16_t)((data_low >> 16U) & 0xFFFFU);
            float   target_rpm = (float)target_rpm_raw;

            // 模式有效性合法化检查
            uint8_t mode_valid = (cmd_mode == ESC_CMD_STOP || 
                                  cmd_mode == ESC_CMD_RUN_CLOSED || 
                                  cmd_mode == ESC_CMD_CLEAR_FAULT);

            // 转速指令范围合理性检查 (300~800 RPM，或停机状态下的 0 RPM)
            uint8_t rpm_valid = (cmd_mode == ESC_CMD_STOP) || 
                                (target_rpm >= 300.0f && target_rpm <= 800.0f);

            // 序号新鲜度判断：首帧通过；后续帧必须递增（考虑 8 位回绕：差值在 1~127 视为新帧）
            uint8_t seq_fresh = 0U;
            if (!first_seq_received) {
                seq_fresh = 1U;
            } else {
                uint8_t diff = (uint8_t)(seq_id - last_rx_seq);
                if (diff > 0U && diff < 128U) {
                    seq_fresh = 1U;
                }
            }

            // 只有当命令合法、转速在允许区间、且数据新鲜时，才被认定为有效帧并刷新看门狗
            if (mode_valid && rpm_valid && seq_fresh) {
                accepted_count++;
                last_accepted_mode = cmd_mode;
                last_accepted_rpm = target_rpm_raw;
                last_rx_seq = seq_id;
                first_seq_received = 1U;
                last_rx_cycles = now_cycles;
                fdcan_connected = 1U;

                // 清除故障状态
                if (cmd_mode == ESC_CMD_CLEAR_FAULT) {
                    watchdog_latched = 0U;
                }

                // STOP 命令不仅复位看门狗锁存，还联动关断停机
                if (cmd_mode == ESC_CMD_STOP) {
                    watchdog_latched = 0U;
                    Motor_Test_RequestStop();
                }

                // 阶段 1：只接收目标参数，绝不使能起动！
                #if (CAN_PHASE1_PASSIVE_TEST == 0U)
                if (!watchdog_latched && cmd_mode == ESC_CMD_RUN_CLOSED) {
                    Motor_Set_Target_RPM(target_rpm);
                    target_write_count++;
                    if (motor_test_status == MOTOR_TEST_IDLE || motor_test_status == MOTOR_TEST_STOPPED) {
                        Motor_Test_RequestStart();
                    }
                }
                #else
                if (!watchdog_latched && cmd_mode == ESC_CMD_RUN_CLOSED) {
                    Motor_Set_Target_RPM(target_rpm);
                    target_write_count++;
                    // 阶段 1 纯被动测试：仅更新 target_rpm，禁止调用 Motor_Test_RequestStart()
                }
                #endif
            }
        }
    }

    // 确认释放 Rx FIFO 0 缓冲区
    FDCAN1->RXF0A = get_idx;
}

//=============================================================================
// 3. 经典 CAN 遥测数据帧打包发送 (0x301, 8 字节)
//=============================================================================
static void FDCAN1_Transmit_Telemetry(void)
{
    if ((FDCAN1->TXFQS & FDCAN_TXFQS_TFQF) != 0U) return;

    uint8_t put_idx = (uint8_t)((FDCAN1->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos);
    volatile uint32_t *tx_buf = (volatile uint32_t *)(SRAMCAN_BASE + ((SRAMCAN_TF_OFFSET_WORDS + (put_idx * MSG_ELEMENT_SIZE_WORDS)) * 4U));

    Actuator_Telemetry_t telem;
    Motor_Get_Telemetry(&telem);

    int16_t actual_rpm_smo = (int16_t)telem.actual_rpm; // 观测器估算转速
    int16_t i_q_mA         = (int16_t)(telem.i_q * 1000.0f);
    uint16_t v_bus_mV      = 12000U;
    uint8_t status         = telem.status;
    uint8_t fault_code     = telem.fault_code;

    // Header 0: 标准 11-bit ID (0x301)
    tx_buf[0] = (FDCAN_TELEM_ID << 18U);
    // Header 1: DLC=8, 经典 CAN 帧 (FDF=0, BRS=0)
    tx_buf[1] = (8U << 16U);

    // Payload
    tx_buf[2] = ((uint32_t)(uint16_t)actual_rpm_smo) | (((uint32_t)(uint16_t)i_q_mA) << 16U);
    tx_buf[3] = ((uint32_t)v_bus_mV) | (((uint32_t)status) << 16U) | (((uint32_t)fault_code) << 24U);

    // 触发发送
    FDCAN1->TXBAR = (1U << put_idx);
}

//=============================================================================
// 4. 周期服务轮询 (原始周期差分计算，彻底免除 25 秒回绕误跳闸)
//=============================================================================
void FDCAN1_Poll(uint32_t now_cycles)
{
    // 1. 优先处理接收并刷新时间
    FDCAN1_Process_Rx_Message(now_cycles);

    // 2. 严格 1kHz (170,000 cycles) 周期发送遥测
    if ((uint32_t)(now_cycles - last_tx_cycles) >= TELEMETRY_INTERVAL_CYCLES) {
        last_tx_cycles = now_cycles;
        FDCAN1_Transmit_Telemetry();
    }

    // 3. 硬件无损 50ms (8,500,000 cycles) 通信看门狗检测
    if (fdcan_connected && !watchdog_latched) {
        if ((uint32_t)(now_cycles - last_rx_cycles) >= FDCAN_WATCHDOG_CYCLES) {
            fdcan_connected = 0U;
            watchdog_latched = 1U;
            Motor_Test_RequestStop(); // 联动停止并关断 MOE
            printf("[FDCAN Watchdog] Lost valid frames > 50ms! Safety Trip.\r\n");
        }
    }
    // Print only on explicit request, after RX/TX/watchdog processing.
    // The UART parser may run in an IRQ; it only sets diag_requested.
    if (diag_requested) {
        diag_requested = 0U;
        printf("[CanDiag] passive=%u seen=%u accepted=%lu mode=%u seq=%u rx_rpm=%d target=%.1f writes=%lu connected=%u watchdog=%u status=%u MOE=%u\r\n",
               (unsigned)CAN_PHASE1_PASSIVE_TEST, (unsigned)first_seq_received,
               (unsigned long)accepted_count, (unsigned)last_accepted_mode,
               (unsigned)last_rx_seq, (int)last_accepted_rpm,
               (double)Motor_Get_Command_Target_RPM(),
               (unsigned long)target_write_count, (unsigned)FDCAN1_Is_Connected(),
               (unsigned)watchdog_latched, (unsigned)motor_test_status,
               (unsigned)((TIM1->BDTR & TIM_BDTR_MOE) != 0U));
    }
}

void FDCAN1_RequestDiag(void)
{
    diag_requested = 1U;
}

uint8_t FDCAN1_Is_Connected(void)
{
    return (fdcan_connected && !watchdog_latched);
}