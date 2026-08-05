#ifndef EVENT_LOG_H
#define EVENT_LOG_H

#include <stdint.h>

// Wire log levels — MUST match rust/motion-engine/src/mcu_log.rs::mcu_level_str
// and the McuLog wire layout in rust/mcu-protocol/src/messages.rs.
#define EVENT_LOG_LEVEL_TRACE 0
#define EVENT_LOG_LEVEL_DEBUG 1
#define EVENT_LOG_LEVEL_WARN  2
#define EVENT_LOG_LEVEL_ERROR 3

// MUST mirror the canonical table in rust/runtime/src/log_codes.rs.
#define EVENT_LOG_SUBSYS_RUNTIME 0
#define EVENT_LOG_SUBSYS_MOTION  1
#define EVENT_LOG_SUBSYS_ENDSTOP 3
#define EVENT_LOG_SUBSYS_DIAG    4
#define EVENT_LOG_EVENT_RUNTIME_MCU_READY 3
#define EVENT_LOG_EVENT_RUNTIME_LOG_DROPS 4
#define EVENT_LOG_EVENT_RUNTIME_MCU_RESET 5
#define EVENT_LOG_EVENT_RUNTIME_HARD_FAULT 6
#define EVENT_LOG_EVENT_RUNTIME_FAULT_STATUS 7
#define EVENT_LOG_EVENT_RUNTIME_FG_FREEZE 8
#define EVENT_LOG_EVENT_RUNTIME_RT_PROGRESS 9
#define EVENT_LOG_EVENT_RUNTIME_LAST_DISPATCH 10
#define EVENT_LOG_EVENT_RUNTIME_ISR_PHASE     11
#define EVENT_LOG_EVENT_RUNTIME_BLOCK_SOURCE  12
#define EVENT_LOG_EVENT_RUNTIME_TIM5_IA       13
#define EVENT_LOG_EVENT_RUNTIME_DIAG_DUMP     14
#define EVENT_LOG_EVENT_RUNTIME_STEPOUT_LATE  15
#define EVENT_LOG_EVENT_RUNTIME_RING_STATE    16
#define EVENT_LOG_EVENT_RUNTIME_FG_TASK       17
#define EVENT_LOG_EVENT_RUNTIME_FG_MSG        18
#define EVENT_LOG_EVENT_RUNTIME_FG_DEMUX      19
#define EVENT_LOG_EVENT_RUNTIME_FG_MSG_HEAD   20
#define EVENT_LOG_EVENT_RUNTIME_TIMER_TOO_CLOSE      21
#define EVENT_LOG_EVENT_RUNTIME_TIMER_TOO_CLOSE_LATE 22

#define EVENT_LOG_EVENT_MOTION_AXIS_STALLED      3
#define EVENT_LOG_EVENT_MOTION_AXIS_STALLED_HEAD 4
#define EVENT_LOG_EVENT_MOTION_STEP_LOAD_LATE    5
#define EVENT_LOG_EVENT_MOTION_STEP_REARM        6
#define EVENT_LOG_EVENT_MOTION_STEP_REARM_TIGHT  7
#define EVENT_LOG_EVENT_MOTION_STEP_REARM_LATE   8

#define EVENT_LOG_EVENT_ENDSTOP_TRSYNC_TRIGGER_CMD  3
#define EVENT_LOG_EVENT_ENDSTOP_TRSYNC_DO_TRIGGER   4
#define EVENT_LOG_EVENT_ENDSTOP_STOP_CB_ENTER       5
#define EVENT_LOG_EVENT_ENDSTOP_TIM5_HALTED         7

// Safe from ISR or foreground (irq_save critical section); drops on full ring,
// never blocks. The only ABI seam — both Rust and C call it.
void event_log_emit(uint8_t level, uint8_t subsystem, uint16_t event,
                     uint16_t code, uint32_t arg0, uint32_t arg1);

// Foreground-only (calls runtime_widened_host_clock()).
void event_log_drain(void);

#endif
