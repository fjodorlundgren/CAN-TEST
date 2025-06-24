#include "gimbal_test_command.h"

// External functions from main.cpp
extern void sendPacketOverCAN(uint8_t *packet, uint8_t length);

// Test command timing variables
unsigned long lastTestCommand = 0;
const unsigned long TEST_COMMAND_INTERVAL = 3000; // Send test command every 3 seconds
bool enable_test_command = true; // Set to false to disable test commands

/**
 * Send DJI Test Command (from documentation example)
 * This is the exact command from Figure 41 in DJI R SDK Protocol documentation
 * Command: AA 1A 00 03 00 00 00 00 22 11 A2 42 0E 00 20 00 30 00 40 00 01 14 7B 40 97 BE
 * 
 * Analysis of the command:
 * - CmdSet: 0x0E (Gimbal Command Set)
 * - CmdID: 0x00 (Position Control)
 * - Yaw: 0x0020 = 32 = 3.2° 
 * - Roll: 0x0030 = 48 = 4.8°
 * - Pitch: 0x0040 = 64 = 6.4°
 * - Control Mode: 0x01 (Absolute)
 * - Action Time: 0x14 = 20 = 2.0 seconds
 */
void sendDJITestCommand()
{
    Serial.println("→ Sending DJI Test Command (from documentation)...");
    
    // Hardcoded test command from DJI R SDK Protocol documentation
    uint8_t test_packet[26] = {
        0xAA, 0x1A, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00,  // Header
        0x22, 0x11,                                        // SEQ
        0xA2, 0x42,                                        // CRC16 (pre-calculated)
        0x0E, 0x00,                                        // CmdSet + CmdID
        0x20, 0x00,                                        // Yaw: 3.2°
        0x30, 0x00,                                        // Roll: 4.8°
        0x40, 0x00,                                        // Pitch: 6.4°
        0x01,                                              // Control Mode: Absolute
        0x14,                                              // Action Time: 2.0s
        0x7B, 0x40, 0x97, 0xBE                            // CRC32 (pre-calculated)
    };
    
    Serial.println("  Command Analysis:");
    Serial.println("  - Target: Yaw=3.2°, Roll=4.8°, Pitch=6.4°");
    Serial.println("  - Mode: Absolute positioning");
    Serial.println("  - Time: 2.0 seconds to reach position");
    Serial.println("  - Source: DJI R SDK Protocol Figure 41");
    
    sendPacketOverCAN(test_packet, 26);
}

/**
 * Enable or disable test command mode
 * @param enable: true to enable test commands, false to disable
 */
void enableTestMode(bool enable)
{
    enable_test_command = enable;
    Serial.print("Test command mode: ");
    Serial.println(enable ? "ENABLED" : "DISABLED");
    
    if (enable) {
        Serial.println("Will send test command every 3 seconds");
        Serial.println("Expected gimbal movement: Yaw=3.2°, Roll=4.8°, Pitch=6.4°");
    }
}
