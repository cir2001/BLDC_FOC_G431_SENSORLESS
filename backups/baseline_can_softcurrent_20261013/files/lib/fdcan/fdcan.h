#ifndef __FDCAN_H
#define __FDCAN_H

#include "sys.h"
#include <stdint.h>

// 节点 ID 配置：严格对齐交接基线 (ESC1: 下行 0x201, 上行 0x301)
#define ESC_NODE_ID                 (1U)
#define FDCAN_CMD_ID                (0x200U + ESC_NODE_ID)  // 0x201
#define FDCAN_TELEM_ID              (0x300U + ESC_NODE_ID)  // 0x301

// 阶段 1 测试门控：1 = 强制 MOE=0 硬件静默，仅测试通信、收发、看门狗与统计
#ifndef CAN_PHASE1_PASSIVE_TEST
#define CAN_PHASE1_PASSIVE_TEST     (1U)
#endif

// 下发控制模式枚举
typedef enum {
    ESC_CMD_STOP        = 0x00,
    ESC_CMD_RUN_CLOSED  = 0x01,
    ESC_CMD_CLEAR_FAULT = 0x02
} EscCommandMode;

// 对外接口声明
void FDCAN1_Init(void);
void FDCAN1_RequestDiag(void);
void FDCAN1_Poll(uint32_t now_cycles);
uint8_t FDCAN1_Is_Connected(void);

#endif