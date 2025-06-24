#ifndef GIMBAL_POSITION_READOUT_H
#define GIMBAL_POSITION_READOUT_H

#include <Arduino.h>

// Gimbal angles structure
struct GimbalAngles
{
    int16_t yaw = 0;       // in 0.1 degree units
    int16_t roll = 0;      // in 0.1 degree units
    int16_t pitch = 0;     // in 0.1 degree units
    uint8_t data_type = 0; // 0=not ready, 1=attitude angle, 2=joint angle
    bool data_valid = false;
    unsigned long last_update = 0;
};

// Global variables
extern GimbalAngles current_angles;
extern bool enable_angle_requests;
extern unsigned long lastAngleRequest;
extern const unsigned long ANGLE_REQUEST_INTERVAL;

// Function declarations
void sendDJIAngleInfoRequest(uint8_t angle_type = 0x01);
bool getCurrentGimbalAngles(float* yaw_degrees, float* roll_degrees, float* pitch_degrees);
void printGimbalStatus();
void parseAngleResponse(uint8_t* packet, uint16_t packet_length);

#endif
