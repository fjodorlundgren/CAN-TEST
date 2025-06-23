#include <Arduino.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can0;

#define DJI_CAN_TX_ID 0x223
#define DJI_CAN_RX_ID 0x222
#define DJI_BAUD_RATE 1000000

int16_t target_yaw = 1800;  // range: -1800 to 1800 (in 0.1 degree steps), for craft only -900 to 900
int16_t target_roll = -300; // range: -300 to 300 (in 0.1 degree steps)
int16_t target_pitch = 900; // range: -560 to 1460 (in 0.1 degree steps)
uint8_t control_mode = 0x01;
uint8_t action_time = 20;

unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL = 5000;

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
    can0.begin();
    can0.setBaudRate(DJI_BAUD_RATE);
    can0.setMaxMB(16);
    can0.enableFIFO();
    can0.enableFIFOInterrupt();

    can0.setFIFOFilter(REJECT_ALL);
    can0.setFIFOFilter(0, DJI_CAN_RX_ID, STD);

    can0.onReceive([](const CAN_message_t &msg)
                   {
                       // Handle received messages here
                   });
}

void sendPacketOverCAN(uint8_t *packet, uint8_t length)
{
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

        can0.write(msg);
        delayMicroseconds(500);
    }
}

void sendDJIPositionCommand()
{
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

void loop()
{
    can0.events();

    if (millis() - lastSend >= SEND_INTERVAL)
    {
        lastSend = millis();
        sendDJIPositionCommand();
    }

    delay(10);
}
