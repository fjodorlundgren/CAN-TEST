#include <Arduino.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can0;

#define DJI_CAN_TX_ID 0x223
#define DJI_CAN_RX_ID 0x222
#define DJI_BAUD_RATE 1000000

// Command sample from documentation (section 3.3)
uint8_t dji_command_sample[] = {
    0xAA, 0x1A, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00,
    0x22, 0x11, 0xA2, 0x42, 0x0E, 0x00, 0x20, 0x00,
    0x30, 0x00, 0x40, 0x00, 0x01, 0x14, 0x7B, 0x40,
    0x97, 0xBE};

unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL = 5000; // 5 seconds

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println("DJI Command Sample Test");

    // Initialize CAN
    can0.begin();
    can0.setBaudRate(DJI_BAUD_RATE);
    can0.setMaxMB(16);
    can0.enableFIFO();
    can0.enableFIFOInterrupt();

    // Set up receive filter
    can0.setFIFOFilter(REJECT_ALL);
    can0.setFIFOFilter(0, DJI_CAN_RX_ID, STD);

    // Set up receive handler
    can0.onReceive([](const CAN_message_t &msg)
                   {
        Serial.print("Received: ");
        for (int i = 0; i < msg.len; i++) {
            Serial.printf("%02X ", msg.buf[i]);
        }
        Serial.println(); });

    Serial.println("Setup complete");
}

void sendCommandSample()
{
    Serial.println("Sending DJI command sample...");

    // Calculate number of frames needed (8 bytes per CAN frame)
    int totalBytes = sizeof(dji_command_sample);
    int frames = (totalBytes + 7) / 8; // Round up division

    Serial.printf("Sending %d bytes in %d frames\n", totalBytes, frames);

    for (int frame = 0; frame < frames; frame++)
    {
        CAN_message_t msg;
        msg.id = DJI_CAN_TX_ID;
        msg.flags.extended = false;
        msg.flags.remote = false;

        int startByte = frame * 8;
        int bytesInFrame = min(8, totalBytes - startByte);
        msg.len = bytesInFrame;

        // Copy data for this frame
        for (int i = 0; i < bytesInFrame; i++)
        {
            msg.buf[i] = dji_command_sample[startByte + i];
        }

        // Send frame
        bool success = can0.write(msg);

        Serial.printf("Frame %d: ", frame + 1);
        for (int i = 0; i < bytesInFrame; i++)
        {
            Serial.printf("%02X ", msg.buf[i]);
        }
        Serial.printf("-> %s\n", success ? "OK" : "FAILED");

        // Small delay between frames
        delayMicroseconds(500);
    }

    Serial.println("Command sample sent");
    Serial.println();
}

void loop()
{
    can0.events();

    if (millis() - lastSend >= SEND_INTERVAL)
    {
        lastSend = millis();
        sendCommandSample();
    }

    delay(10);
}
