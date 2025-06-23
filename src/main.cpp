#include <Arduino.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can0;

#define DJI_CAN_TX_ID 0x223
#define DJI_CAN_RX_ID 0x222
#define DJI_BAUD_RATE 1000000

int16_t target_yaw = -900;   // range: -1800 to 1800 (in 0.1 degree steps), for craft only -900 to 900
int16_t target_roll = 0;     // range: -300 to 300 (in 0.1 degree steps)
int16_t target_pitch = -560; // range: -560 to 1460 (in 0.1 degree steps)
uint8_t control_mode = 0x01;
uint8_t action_time = 20;

unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL = 5000;

// Variables for angle information requests
unsigned long lastAngleRequest = 0;
const unsigned long ANGLE_REQUEST_INTERVAL = 100; // Request angles every 1 second

// Variables to store received angle information
struct GimbalAngles
{
    int16_t yaw = 0;       // in 0.1 degree units
    int16_t roll = 0;      // in 0.1 degree units
    int16_t pitch = 0;     // in 0.1 degree units
    uint8_t data_type = 0; // 0=not ready, 1=attitude angle, 2=joint angle
    bool data_valid = false;
    unsigned long last_update = 0;
};

GimbalAngles current_angles;
bool enable_angle_requests = true; // Set to false to disable angle requests

uint8_t reflect8(uint8_t data)
{
    uint8_t reflection = 0;
    for (int bit = 0; bit < 8; bit++)
    {
        if (data & (1 << bit))
        {
            reflection |= (1 << (7 - bit));
        }
    }
    return reflection;
}

uint16_t reflect16(uint16_t data)
{
    uint16_t reflection = 0;
    for (int bit = 0; bit < 16; bit++)
    {
        if (data & (1 << bit))
        {
            reflection |= (1 << (15 - bit));
        }
    }
    return reflection;
}

uint32_t reflect32(uint32_t data)
{
    uint32_t reflection = 0;
    for (int bit = 0; bit < 32; bit++)
    {
        if (data & (1 << bit))
        {
            reflection |= (1 << (31 - bit));
        }
    }
    return reflection;
}

uint16_t calculateCRC16_DJI(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xc55c;
    uint16_t poly = 0x8005;

    for (uint16_t i = 0; i < length; i++)
    {
        uint8_t byte = reflect8(data[i]);
        crc ^= (uint16_t)byte << 8;

        for (int bit = 0; bit < 8; bit++)
        {
            if (crc & 0x8000)
            {
                crc = (crc << 1) ^ poly;
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return reflect16(crc);
}

uint32_t calculateCRC32_DJI(uint8_t *data, uint16_t length)
{
    uint32_t crc = 0xc55c0000;
    uint32_t poly = 0x04c11db7;

    for (uint16_t i = 0; i < length; i++)
    {
        uint8_t byte = reflect8(data[i]);
        crc ^= (uint32_t)byte << 24;

        for (int bit = 0; bit < 8; bit++)
        {
            if (crc & 0x80000000)
            {
                crc = (crc << 1) ^ poly;
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return reflect32(crc);
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("DJI Gimbal CAN Monitor - Enhanced Debug Version");
    Serial.println("Monitoring CAN ID 0x222 (RX) and 0x223 (TX)");

    can0.begin();
    can0.setBaudRate(DJI_BAUD_RATE);
    can0.setMaxMB(16);
    can0.enableFIFO();
    can0.enableFIFOInterrupt();

    // Listen to DJI gimbal responses
    can0.setFIFOFilter(REJECT_ALL);
    can0.setFIFOFilter(0, DJI_CAN_RX_ID, STD); // Only listen to 0x222

    can0.onReceive([](const CAN_message_t &msg)
                   {
                       // Print ALL received CAN messages with timestamp
                       Serial.print("[");
                       Serial.print(millis());
                       Serial.print("ms] CAN RX ID:0x");
                       if (msg.id < 0x100) Serial.print("0");
                       if (msg.id < 0x10) Serial.print("0");
                       Serial.print(msg.id, 16);
                       Serial.print(" Len:");
                       Serial.print(msg.len);
                       Serial.print(" Data:");
                       for (int i = 0; i < msg.len; i++) {
                           Serial.print(" ");
                           if (msg.buf[i] < 0x10) Serial.print("0");
                           Serial.print(msg.buf[i], 16);
                       }
                       Serial.println();
                       
                       // Packet reconstruction logic
                       static uint8_t receive_buffer[64];
                       static uint8_t buffer_index = 0;
                       
                       // Add received bytes to buffer
                       for (int i = 0; i < msg.len; i++) {
                           if (buffer_index < 64) {
                               receive_buffer[buffer_index++] = msg.buf[i];
                           }
                       }
                       
                       // Check if we have a packet starting with 0xAA
                       if (buffer_index >= 3 && receive_buffer[0] == 0xAA) {
                           uint16_t packet_length = receive_buffer[1] | (receive_buffer[2] << 8);
                           
                           Serial.print("  [PACKET] Expected len:");
                           Serial.print(packet_length);
                           Serial.print(", Buffer has:");
                           Serial.println(buffer_index);
                           
                           if (buffer_index >= packet_length) {
                               Serial.print("  [COMPLETE] ");
                               for (int i = 0; i < packet_length; i++) {
                                   if (receive_buffer[i] < 0x10) Serial.print("0");
                                   Serial.print(receive_buffer[i], 16);
                                   Serial.print(" ");
                               }
                               Serial.println();
                               
                               // Check for angle response
                               if (packet_length >= 15 && 
                                   receive_buffer[12] == 0x0E &&  // CmdSet
                                   receive_buffer[13] == 0x02) {   // CmdID
                                   Serial.println("  [ANGLE RESPONSE DETECTED]");
                                   if (packet_length >= 22) {
                                       uint8_t return_code = receive_buffer[14];
                                       uint8_t data_type = receive_buffer[15];
                                       int16_t yaw = (int16_t)(receive_buffer[16] | (receive_buffer[17] << 8));
                                       int16_t roll = (int16_t)(receive_buffer[18] | (receive_buffer[19] << 8));
                                       int16_t pitch = (int16_t)(receive_buffer[20] | (receive_buffer[21] << 8));
                                       
                                       Serial.print("    Return Code: 0x");
                                       Serial.println(return_code, 16);
                                       Serial.print("    Data Type: ");
                                       Serial.println(data_type == 1 ? "Attitude" : (data_type == 2 ? "Joint" : "Unknown"));
                                       Serial.print("    Angles: Yaw=");
                                       Serial.print(yaw * 0.1, 1);
                                       Serial.print("° Roll=");
                                       Serial.print(roll * 0.1, 1);
                                       Serial.print("° Pitch=");
                                       Serial.print(pitch * 0.1, 1);
                                       Serial.println("°");
                                   }
                               }
                               
                               buffer_index = 0;
                           }
                       } else if (buffer_index > 0 && receive_buffer[0] != 0xAA) {
                           Serial.print("  [RESET] Invalid start:0x");
                           Serial.println(receive_buffer[0], 16);
                           buffer_index = 0;
                       } });
}

void sendPacketOverCAN(uint8_t *packet, uint8_t length)
{
    // Print outgoing packet
    Serial.print("[");
    Serial.print(millis());
    Serial.print("ms] CAN TX ID:0x");
    Serial.print(DJI_CAN_TX_ID, 16);
    Serial.print(" Total Len:");
    Serial.print(length);
    Serial.print(" Data:");
    for (int i = 0; i < length; i++)
    {
        Serial.print(" ");
        if (packet[i] < 0x10)
            Serial.print("0");
        Serial.print(packet[i], 16);
    }
    Serial.println();

    int frames = (length + 7) / 8;

    for (int frame = 0; frame < frames; frame++)
    {
        CAN_message_t msg;
        msg.id = DJI_CAN_TX_ID;
        msg.flags.extended = false;
        msg.flags.remote = false;

        int startByte = frame * 8;
        int bytesInFrame = min(8, length - startByte);
        msg.len = bytesInFrame;

        for (int i = 0; i < bytesInFrame; i++)
        {
            msg.buf[i] = packet[startByte + i];
        }

        // Print each CAN frame
        Serial.print("  Frame ");
        Serial.print(frame + 1);
        Serial.print("/");
        Serial.print(frames);
        Serial.print(": ");
        for (int i = 0; i < bytesInFrame; i++)
        {
            if (msg.buf[i] < 0x10)
                Serial.print("0");
            Serial.print(msg.buf[i], 16);
            Serial.print(" ");
        }
        Serial.println();

        can0.write(msg);
        delayMicroseconds(500);
    }
}

void sendDJIPositionCommand()
{
    Serial.println("→ Sending position command...");
    uint8_t packet[26];
    uint16_t packet_index = 0;

    packet[packet_index++] = 0xAA;
    packet[packet_index++] = 0x1A;
    packet[packet_index++] = 0x00;
    packet[packet_index++] = 0x03;
    packet[packet_index++] = 0x00;
    packet[packet_index++] = 0x00;
    packet[packet_index++] = 0x00;
    packet[packet_index++] = 0x00;
    packet[packet_index++] = 0x22;
    packet[packet_index++] = 0x11;

    uint16_t crc16 = calculateCRC16_DJI(packet, 10);
    packet[packet_index++] = crc16 & 0xFF;
    packet[packet_index++] = (crc16 >> 8) & 0xFF;

    packet[packet_index++] = 0x0E;
    packet[packet_index++] = 0x00;
    packet[packet_index++] = target_yaw & 0xFF;
    packet[packet_index++] = (target_yaw >> 8) & 0xFF;
    packet[packet_index++] = target_roll & 0xFF;
    packet[packet_index++] = (target_roll >> 8) & 0xFF;
    packet[packet_index++] = target_pitch & 0xFF;
    packet[packet_index++] = (target_pitch >> 8) & 0xFF;
    packet[packet_index++] = control_mode;
    packet[packet_index++] = action_time;

    uint32_t crc32 = calculateCRC32_DJI(packet, 22);
    packet[packet_index++] = crc32 & 0xFF;
    packet[packet_index++] = (crc32 >> 8) & 0xFF;
    packet[packet_index++] = (crc32 >> 16) & 0xFF;
    packet[packet_index++] = (crc32 >> 24) & 0xFF;

    sendPacketOverCAN(packet, 26);
}

void sendDJIAngleInfoRequest(uint8_t angle_type = 0x01)
{
    Serial.println("→ Sending angle info request...");
    uint8_t packet[19];
    uint16_t packet_index = 0;

    // Frame header
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

void loop()
{
    can0.events();

    // Send position commands every 5 seconds
    if (millis() - lastSend >= SEND_INTERVAL)
    {
        lastSend = millis();
        sendDJIPositionCommand();
    }

    // Send angle information requests every 1 second (if enabled)
    if (enable_angle_requests && millis() - lastAngleRequest >= ANGLE_REQUEST_INTERVAL)
    {
        lastAngleRequest = millis();
        sendDJIAngleInfoRequest(0x01);
    }

    delay(10);
}
