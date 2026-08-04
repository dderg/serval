#ifndef __STEPPER_H
#define __STEPPER_H

#include <stdint.h>

#define RUNTIME_MOTOR_COUNT 4
#define RUNTIME_MAX_STEPPERS_PER_MOTOR 4

uint8_t runtime_motor_binding_count(uint8_t motor_idx);
void stepper_suppress_set(uint8_t motor, uint8_t stepper);
void stepper_suppress_clear_all(void);

#endif // stepper.h
