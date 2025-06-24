#ifndef GIMBAL_POSITION_SET_H
#define GIMBAL_POSITION_SET_H

#include <Arduino.h>

// Gimbal position control variables
extern int16_t target_yaw;    // range: -1800 to 1800 (in 0.1 degree steps), for craft only -900 to 900
extern int16_t target_roll;   // range: -300 to 300 (in 0.1 degree steps)
extern int16_t target_pitch;  // range: -560 to 1460 (in 0.1 degree steps)
extern uint8_t control_mode;  // 0x00: Incremental control, 0x01: Absolute control
extern uint8_t action_time;   // Command execution speed, unit: 0.1s

// Function declarations
void sendDJIPositionCommand();
void setGimbalPosition(int16_t yaw, int16_t roll, int16_t pitch, uint8_t mode = 0x01, uint8_t time = 20);

#endif
