#include <Arduino.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can0;

#define DJI_CAN_TX_ID 0x223
#define DJI_CAN_RX_ID 0x222
#define DJI_BAUD_RATE 1000000

uint32_t messages_sent = 0;
uint32_t messages_received = 0;
uint32_t send_failures = 0;
bool can_bus_active = false;

unsigned long lastCommand = 0;
const unsigned long COMMAND_INTERVAL = 10000;

// Command sequence counter
uint8_t current_command = 0;

void setupCANBus();
void sendPositionControl(int16_t yaw, int16_t roll, int16_t pitch, uint8_t ctrl_byte, uint8_t time_action);
void sendSpeedControl(int16_t yaw_speed, int16_t roll_speed, int16_t pitch_speed, uint8_t ctrl_byte);
void sendGetGimbalInfo(uint8_t info_type);
void sendSetLimitAngle(uint8_t pitch_max, uint8_t pitch_min, uint8_t yaw_max, uint8_t yaw_min, uint8_t roll_max, uint8_t roll_min);
void sendGetLimitAngle();
void sendSetMotorStiffness(uint8_t pitch_stiffness, uint8_t roll_stiffness, uint8_t yaw_stiffness);
void sendGetMotorStiffness();
void executeCommandSequence();

void setup()
{
    Serial.begin(115200);
    delay(2000);
    setupCANBus();
}

void loop()
{
    can0.events();

    if (millis() - lastCommand >= COMMAND_INTERVAL)
    {
        lastCommand = millis();
        executeCommandSequence();
    }

    delay(10);
}

void setupCANBus()
{
    can0.begin();
    can0.setBaudRate(DJI_BAUD_RATE);
    can0.setMaxMB(16);
    can0.enableFIFO();
    can0.enableFIFOInterrupt();

    can0.setFIFOFilter(REJECT_ALL);
    can0.setFIFOFilter(0, DJI_CAN_RX_ID, STD);

    can0.onReceive([](const CAN_message_t &msg)
                   { messages_received++; });

    can_bus_active = true;
}

void executeCommandSequence()
{
    switch (current_command)
    {
    case 0:
        sendPositionControl(0, 0, 100, 0x01, 20);
        break;
    case 1:
        sendSpeedControl(100, 0, 50, 0x80);
        break;
    case 3:
        sendSetLimitAngle(145, 0, 179, 0, 30, 0);
        break;
    case 5:
        sendSetMotorStiffness(80, 80, 80);
        break;
    }

    current_command = (current_command + 1) % 7;
}

void sendPositionControl(int16_t yaw, int16_t roll, int16_t pitch, uint8_t ctrl_byte, uint8_t time_action)
{
    CAN_message_t msg;
    msg.id = DJI_CAN_TX_ID;
    msg.flags.extended = false;
    msg.flags.remote = false;
    msg.len = 8;

    msg.buf[0] = 0x0E;
    msg.buf[1] = 0x00;
    msg.buf[2] = yaw & 0xFF;
    msg.buf[3] = (yaw >> 8) & 0xFF;
    msg.buf[4] = roll & 0xFF;
    msg.buf[5] = (roll >> 8) & 0xFF;
    msg.buf[6] = pitch & 0xFF;
    msg.buf[7] = (pitch >> 8) & 0xFF;

    bool success = can0.write(msg);
    if (success)
        messages_sent++;
    else
        send_failures++;

    msg.len = 2;
    msg.buf[0] = ctrl_byte;
    msg.buf[1] = time_action;
    can0.write(msg);
}

void sendSpeedControl(int16_t yaw_speed, int16_t roll_speed, int16_t pitch_speed, uint8_t ctrl_byte)
{
    CAN_message_t msg;
    msg.id = DJI_CAN_TX_ID;
    msg.flags.extended = false;
    msg.flags.remote = false;
    msg.len = 8;

    msg.buf[0] = 0x0E;
    msg.buf[1] = 0x01;
    msg.buf[2] = yaw_speed & 0xFF;
    msg.buf[3] = (yaw_speed >> 8) & 0xFF;
    msg.buf[4] = roll_speed & 0xFF;
    msg.buf[5] = (roll_speed >> 8) & 0xFF;
    msg.buf[6] = pitch_speed & 0xFF;
    msg.buf[7] = (pitch_speed >> 8) & 0xFF;

    bool success = can0.write(msg);
    if (success)
        messages_sent++;
    else
        send_failures++;

    msg.len = 1;
    msg.buf[0] = ctrl_byte;
    can0.write(msg);
}

void sendSetLimitAngle(uint8_t pitch_max, uint8_t pitch_min, uint8_t yaw_max, uint8_t yaw_min, uint8_t roll_max, uint8_t roll_min)
{
    CAN_message_t msg;
    msg.id = DJI_CAN_TX_ID;
    msg.flags.extended = false;
    msg.flags.remote = false;
    msg.len = 8;

    msg.buf[0] = 0x0E;
    msg.buf[1] = 0x03;
    msg.buf[2] = 0x01;
    msg.buf[3] = pitch_max;
    msg.buf[4] = pitch_min;
    msg.buf[5] = yaw_max;
    msg.buf[6] = yaw_min;
    msg.buf[7] = roll_max;

    bool success = can0.write(msg);
    if (success)
        messages_sent++;
    else
        send_failures++;

    msg.len = 1;
    msg.buf[0] = roll_min;
    can0.write(msg);
}

void sendSetMotorStiffness(uint8_t pitch_stiffness, uint8_t roll_stiffness, uint8_t yaw_stiffness)
{
    CAN_message_t msg;
    msg.id = DJI_CAN_TX_ID;
    msg.flags.extended = false;
    msg.flags.remote = false;
    msg.len = 5;

    msg.buf[0] = 0x0E;
    msg.buf[1] = 0x05;
    msg.buf[2] = 0x01;
    msg.buf[3] = pitch_stiffness;
    msg.buf[4] = roll_stiffness;

    bool success = can0.write(msg);
    if (success)
        messages_sent++;
    else
        send_failures++;

    msg.len = 1;
    msg.buf[0] = yaw_stiffness;
    can0.write(msg);
}
