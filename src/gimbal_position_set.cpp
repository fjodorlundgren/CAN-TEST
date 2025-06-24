#include "gimbal_position_set.h"

// External functions from main.cpp
extern void sendPacketOverCAN(uint8_t *packet, uint8_t length);
extern uint16_t calculateCRC16_DJI(uint8_t *data, uint16_t length);
extern uint32_t calculateCRC32_DJI(uint8_t *data, uint16_t length);

// Gimbal position control variables
int16_t target_yaw = -700;   // range: -1800 to 1800 (in 0.1 degree steps), for craft only -900 to 900
int16_t target_roll = 0;     // range: -300 to 300 (in 0.1 degree steps)
int16_t target_pitch = -560; // range: -560 to 1460 (in 0.1 degree steps)
uint8_t control_mode = 0x01; // 0x00: Incremental control, 0x01: Absolute control
uint8_t action_time = 2;     // Command execution speed, unit: 0.1s

/**
 * Send DJI Position Control Command
 * CmdSet=0x0E, CmdID=0x00 according to DJI R SDK Protocol
 * Controls gimbal position in yaw, roll, and pitch axes
 */
void sendDJIPositionCommand()
{
    Serial.println("→ Sending position command...");
    uint8_t packet[26];
    uint16_t packet_index = 0;

    // Frame header - Following DJI R SDK Protocol structure
    packet[packet_index++] = 0xAA; // SOF
    packet[packet_index++] = 0x1A; // Length LSB (26 bytes total)
    packet[packet_index++] = 0x00; // Length MSB
    packet[packet_index++] = 0x03; // CmdType (reply required)
    packet[packet_index++] = 0x00; // ENC
    packet[packet_index++] = 0x00; // RES[0]
    packet[packet_index++] = 0x00; // RES[1]
    packet[packet_index++] = 0x00; // RES[2]
    packet[packet_index++] = 0x22; // SEQ LSB
    packet[packet_index++] = 0x11; // SEQ MSB

    // Calculate CRC16 for header (first 10 bytes)
    uint16_t crc16 = calculateCRC16_DJI(packet, 10);
    packet[packet_index++] = crc16 & 0xFF;        // CRC16 LSB
    packet[packet_index++] = (crc16 >> 8) & 0xFF; // CRC16 MSB

    // Data segment - Position control data
    packet[packet_index++] = 0x0E;                       // CmdSet
    packet[packet_index++] = 0x00;                       // CmdID (Position control)
    packet[packet_index++] = target_yaw & 0xFF;          // Yaw angle LSB
    packet[packet_index++] = (target_yaw >> 8) & 0xFF;   // Yaw angle MSB
    packet[packet_index++] = target_roll & 0xFF;         // Roll angle LSB
    packet[packet_index++] = (target_roll >> 8) & 0xFF;  // Roll angle MSB
    packet[packet_index++] = target_pitch & 0xFF;        // Pitch angle LSB
    packet[packet_index++] = (target_pitch >> 8) & 0xFF; // Pitch angle MSB
    packet[packet_index++] = control_mode;               // Control mode
    packet[packet_index++] = action_time;                // Action time

    // Calculate CRC32 for entire frame (excluding CRC32 itself)
    uint32_t crc32 = calculateCRC32_DJI(packet, 22);
    packet[packet_index++] = crc32 & 0xFF;         // CRC32 byte 0
    packet[packet_index++] = (crc32 >> 8) & 0xFF;  // CRC32 byte 1
    packet[packet_index++] = (crc32 >> 16) & 0xFF; // CRC32 byte 2
    packet[packet_index++] = (crc32 >> 24) & 0xFF; // CRC32 byte 3

    sendPacketOverCAN(packet, 26);
}

/**
 * Set gimbal target position
 * @param yaw: Target yaw angle in 0.1 degree units (-1800 to +1800)
 * @param roll: Target roll angle in 0.1 degree units (-300 to +300)
 * @param pitch: Target pitch angle in 0.1 degree units (-560 to +1460)
 * @param mode: Control mode (0x00=incremental, 0x01=absolute)
 * @param time: Action time in 0.1s units (how fast to reach target)
 */
void setGimbalPosition(int16_t yaw, int16_t roll, int16_t pitch, uint8_t mode, uint8_t time)
{
    // Clamp values to valid ranges
    target_yaw = constrain(yaw, -1800, 1800);
    target_roll = constrain(roll, -300, 300);
    target_pitch = constrain(pitch, -560, 1460);
    control_mode = mode;
    action_time = time;

    Serial.print("Set gimbal target: Yaw=");
    Serial.print(target_yaw * 0.1, 1);
    Serial.print("°, Roll=");
    Serial.print(target_roll * 0.1, 1);
    Serial.print("°, Pitch=");
    Serial.print(target_pitch * 0.1, 1);
    Serial.print("°, Mode=");
    Serial.print(control_mode == 0x01 ? "Absolute" : "Incremental");
    Serial.print(", Time=");
    Serial.print(action_time * 0.1, 1);
    Serial.println("s");
}
