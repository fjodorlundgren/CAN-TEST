#include "gimbal_position_readout.h"

// External functions from main.cpp
extern void sendPacketOverCAN(uint8_t *packet, uint8_t length);
extern uint16_t calculateCRC16_DJI(uint8_t *data, uint16_t length);
extern uint32_t calculateCRC32_DJI(uint8_t *data, uint16_t length);

// Gimbal angles variables
GimbalAngles current_angles;
bool enable_angle_requests = true;
unsigned long lastAngleRequest = 0;
const unsigned long ANGLE_REQUEST_INTERVAL = 5000; // Request angles every 1 second

/**
 * Send DJI Angle Information Request Command
 * CmdSet=0x0E, CmdID=0x02 according to DJI R SDK Protocol
 * @param angle_type: 0x01 = attitude angle, 0x02 = joint angle
 */
void sendDJIAngleInfoRequest(uint8_t angle_type)
{
    Serial.println("→ Sending angle info request...");
    uint8_t packet[19]; // Correct packet size: 19 bytes total
    uint16_t packet_index = 0;

    // Frame header - Following DJI R SDK Protocol structure
    packet[packet_index++] = 0xAA; // SOF
    packet[packet_index++] = 0x13; // Length LSB (19 bytes total)
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

    // Data segment
    packet[packet_index++] = 0x0E;       // CmdSet
    packet[packet_index++] = 0x02;       // CmdID (Obtain angle information)
    packet[packet_index++] = angle_type; // ctrl_byte: 0x01=attitude, 0x02=joint

    // Calculate CRC32 for entire frame (excluding CRC32 itself)
    uint32_t crc32 = calculateCRC32_DJI(packet, 15);
    packet[packet_index++] = crc32 & 0xFF;         // CRC32 byte 0
    packet[packet_index++] = (crc32 >> 8) & 0xFF;  // CRC32 byte 1
    packet[packet_index++] = (crc32 >> 16) & 0xFF; // CRC32 byte 2
    packet[packet_index++] = (crc32 >> 24) & 0xFF; // CRC32 byte 3

    sendPacketOverCAN(packet, 19);
}

/**
 * Parse angle response from gimbal
 * @param packet: Complete received packet
 * @param packet_length: Length of the packet
 */
void parseAngleResponse(uint8_t *packet, uint16_t packet_length)
{
    // Check for angle response
    if (packet_length >= 15 &&
        packet[12] == 0x0E && // CmdSet
        packet[13] == 0x02)
    { // CmdID

        Serial.println("  [ANGLE RESPONSE DETECTED]");

        if (packet_length >= 22)
        {
            uint8_t return_code = packet[14];
            uint8_t data_type = packet[15];

            Serial.print("    Return Code: 0x");
            Serial.println(return_code, 16);
            Serial.print("    Data Type: ");
            Serial.println(data_type == 1 ? "Attitude" : (data_type == 2 ? "Joint" : "Unknown"));

            if (return_code == 0x00 && (data_type == 0x01 || data_type == 0x02))
            {
                // Extract angle data (little-endian format)
                current_angles.yaw = (int16_t)(packet[16] | (packet[17] << 8));
                current_angles.roll = (int16_t)(packet[18] | (packet[19] << 8));
                current_angles.pitch = (int16_t)(packet[20] | (packet[21] << 8));
                current_angles.data_type = data_type;
                current_angles.data_valid = true;
                current_angles.last_update = millis();

                Serial.print("    ✓ Angles: Yaw=");
                Serial.print(current_angles.yaw * 0.1, 1);
                Serial.print("° Roll=");
                Serial.print(current_angles.roll * 0.1, 1);
                Serial.print("° Pitch=");
                Serial.print(current_angles.pitch * 0.1, 1);
                Serial.println("°");
            }
            else
            {
                Serial.print("    ✗ Error: Return code 0x");
                Serial.println(return_code, 16);
            }
        }
        else
        {
            Serial.println("    ✗ Packet too short for angle data");
        }
    }
}

/**
 * Helper function to get current gimbal angles
 * @param yaw_degrees: pointer to store yaw angle in degrees
 * @param roll_degrees: pointer to store roll angle in degrees
 * @param pitch_degrees: pointer to store pitch angle in degrees
 * @return: true if data is valid and recent (within last 5 seconds)
 */
bool getCurrentGimbalAngles(float *yaw_degrees, float *roll_degrees, float *pitch_degrees)
{
    if (current_angles.data_valid && (millis() - current_angles.last_update < 5000))
    {
        *yaw_degrees = current_angles.yaw * 0.1; // Convert from 0.1 degree units to degrees
        *roll_degrees = current_angles.roll * 0.1;
        *pitch_degrees = current_angles.pitch * 0.1;
        return true;
    }
    return false;
}

/**
 * Helper function to print current gimbal status
 */
void printGimbalStatus()
{
    Serial.println("\n=== Gimbal Status ===");

    if (current_angles.data_valid)
    {
        Serial.print("Angle Type: ");
        Serial.println(current_angles.data_type == 1 ? "Attitude" : "Joint");
        Serial.print("Yaw: ");
        Serial.print(current_angles.yaw * 0.1, 1);
        Serial.println("°");
        Serial.print("Roll: ");
        Serial.print(current_angles.roll * 0.1, 1);
        Serial.println("°");
        Serial.print("Pitch: ");
        Serial.print(current_angles.pitch * 0.1, 1);
        Serial.println("°");
        Serial.print("Last Update: ");
        Serial.print(millis() - current_angles.last_update);
        Serial.println(" ms ago");
    }
    else
    {
        Serial.println("No valid angle data received yet");
    }

    Serial.println("========================\n");
}
