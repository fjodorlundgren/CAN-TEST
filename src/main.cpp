#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "gimbal_position_set.h"
#include "gimbal_position_readout.h"
// #include "gimbal_test_command.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

#define DJI_CAN_TX_ID 0x223
#define DJI_CAN_RX_ID 0x222
#define DJI_BAUD_RATE 500000

// Timing variables
unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL = 5000;

// CAN monitoring variables
static unsigned long total_rx_messages = 0;
static unsigned long last_rx_count_check = 0;
const unsigned long RX_CHECK_INTERVAL = 10000; // Check every 10 seconds

// Configuration option (change this to try different setups)
#define CAN_CONFIG_OPTION 1 // 1=Mailboxes, 2=FIFO, 3=Mixed

// =============================================================================
// CRC CALCULATION FUNCTIONS
// =============================================================================

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

// =============================================================================
// CAN MESSAGE HANDLING
// =============================================================================

void sendPacketOverCAN(uint8_t *packet, uint8_t length)
{
    // Print outgoing packet for debugging
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

    // Split packet into CAN frames (max 8 bytes per frame)
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

        // Print each CAN frame for debugging
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

        can1.write(msg);
        delayMicroseconds(500); // Small delay between frames
    }
}

void onCANReceive(const CAN_message_t &msg)
{
    // Increment message counter
    total_rx_messages++;

    // Print ALL received CAN messages with timestamp
    Serial.print("[RX#");
    Serial.print(total_rx_messages);
    Serial.print(" @ ");
    Serial.print(millis());
    Serial.print("ms] ID:0x");
    if (msg.id < 0x100)
        Serial.print("0");
    if (msg.id < 0x10)
        Serial.print("0");
    Serial.print(msg.id, 16);
    Serial.print(" Len:");
    Serial.print(msg.len);
    Serial.print(" Data:");
    for (int i = 0; i < msg.len; i++)
    {
        Serial.print(" ");
        if (msg.buf[i] < 0x10)
            Serial.print("0");
        Serial.print(msg.buf[i], 16);
    }

    // Check if this looks like a DJI packet
    if (msg.id == DJI_CAN_RX_ID)
    {
        Serial.print(" *** DJI GIMBAL RESPONSE ***");
    }
    else
    {
        Serial.print(" (Unknown device)");
    }
    Serial.println();

    // Packet reconstruction logic
    static uint8_t receive_buffer[64];
    static uint8_t buffer_index = 0;

    // Add received bytes to buffer
    for (int i = 0; i < msg.len; i++)
    {
        if (buffer_index < 64)
        {
            receive_buffer[buffer_index++] = msg.buf[i];
        }
    }

    // Check if we have a packet starting with 0xAA
    if (buffer_index >= 3 && receive_buffer[0] == 0xAA)
    {
        uint16_t packet_length = receive_buffer[1] | (receive_buffer[2] << 8);

        Serial.print("  [PACKET] Expected len:");
        Serial.print(packet_length);
        Serial.print(", Buffer has:");
        Serial.println(buffer_index);

        if (buffer_index >= packet_length)
        {
            Serial.print("  [COMPLETE] ");
            for (int i = 0; i < packet_length; i++)
            {
                if (receive_buffer[i] < 0x10)
                    Serial.print("0");
                Serial.print(receive_buffer[i], 16);
                Serial.print(" ");
            }
            Serial.println();

            // Parse angle response using gimbal_position_readout module
            parseAngleResponse(receive_buffer, packet_length);

            // Reset buffer for next packet
            buffer_index = 0;
        }
    }
    else if (buffer_index > 0 && receive_buffer[0] != 0xAA)
    {
        Serial.print("  [RESET] Invalid start:0x");
        Serial.println(receive_buffer[0], 16);
        buffer_index = 0;
    }
}

// =============================================================================
// SETUP AND MAIN LOOP
// =============================================================================

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("DJI Gimbal CAN Monitor - RX Configuration Testing");
    Serial.println("=== Modules ===");
    Serial.println("- main.cpp: CRC + CAN handling");
    Serial.println("- gimbal_position_set.cpp: Position control (CmdID 0x00)");
    Serial.println("- gimbal_position_readout.cpp: Angle readout (CmdID 0x02)");
    Serial.println("- gimbal_test_command.cpp: Documentation test command");
    Serial.println("===============");
    Serial.print("*** CAN RX CONFIG OPTION: ");
    Serial.print(CAN_CONFIG_OPTION);
    Serial.println(" ***");
    Serial.println("*** TEST MODE: Sending doc example every 3s ***");
    Serial.println("Expected: TX on 0x223, RX on 0x222");
    Serial.println("============================================");

    // Initialize CAN interface
    Serial.println("Initializing CAN interface...");
    can1.begin();
    can1.setBaudRate(DJI_BAUD_RATE);

#if CAN_CONFIG_OPTION == 1
    // Configuration Option 1: Mailboxes only
    Serial.println("Config 1: Using Mailboxes (MB0=0x222, MB1=ANY)");
    can1.setMaxMB(16);

    // Set up mailbox 0 for receiving on ID 0x222
    can1.setMB(MB0, RX, STD);
    can1.setMBFilter(MB0, DJI_CAN_RX_ID);

    // Set up mailbox 1 for receiving ANY ID (debug)
    can1.setMB(MB1, RX, STD);
    // Accept all IDs by using the public setMBFilter overload with a zero mask
    can1.setMBFilter(MB1, 0x000, 0x000); // No mask = accept all

    Serial.print("✓ MB0 configured for ID 0x");
    Serial.println(DJI_CAN_RX_ID, 16);
    Serial.println("✓ MB1 configured for ANY ID");

#elif CAN_CONFIG_OPTION == 2
    // Configuration Option 2: FIFO only
    Serial.println("Config 2: Using FIFO with ACCEPT_ALL");
    can1.setMaxMB(16);
    can1.enableFIFO();
    can1.enableFIFOInterrupt();
    can1.setFIFOFilter(ACCEPT_ALL);
    Serial.println("✓ FIFO enabled with ACCEPT_ALL filter");

#elif CAN_CONFIG_OPTION == 3
    // Configuration Option 3: FIFO with specific filters
    Serial.println("Config 3: Using FIFO with specific filters");
    can1.setMaxMB(16);
    can1.enableFIFO();
    can1.enableFIFOInterrupt();
    can1.setFIFOFilter(REJECT_ALL);
    can1.setFIFOFilter(0, DJI_CAN_RX_ID, STD);
    can1.setFIFOFilter(1, 0x000, STD); // Accept any ID in slot 1
    Serial.print("✓ FIFO filter 0 set for ID 0x");
    Serial.println(DJI_CAN_RX_ID, 16);
    Serial.println("✓ FIFO filter 1 set for ANY ID");
#endif

    // Set up CAN receive callback
    can1.onReceive(onCANReceive);

    Serial.println("Setup complete. Starting communication...");
    Serial.println("If you confirmed replies with CAN USB adapter but see");
    Serial.println("NO RX messages here, the issue is Teensy CAN RX config.");
    Serial.println("Try changing CAN_CONFIG_OPTION at top of main.cpp");
    Serial.println("============================================");
}

void loop()
{
    can1.events();

    // Send test command every 3 seconds (highest priority)
    /*   if (enable_test_command && millis() - lastTestCommand >= TEST_COMMAND_INTERVAL)
       {
           lastTestCommand = millis();
           sendDJITestCommand(); // From gimbal_test_command module
       }
   */
    // Send position commands every 5 seconds
    if (millis() - lastSend >= SEND_INTERVAL)
    {
        lastSend = millis();
        sendDJIPositionCommand(); // From gimbal_position_set module
    }

    // Send angle information requests every 1 second (if enabled)
    if (enable_angle_requests && millis() - lastAngleRequest >= ANGLE_REQUEST_INTERVAL)
    {
        lastAngleRequest = millis();
        sendDJIAngleInfoRequest(0x01); // From gimbal_position_readout module
    }

    // Periodic CAN traffic monitoring
    if (millis() - last_rx_count_check >= RX_CHECK_INTERVAL)
    {
        last_rx_count_check = millis();
        Serial.println("\n=== CAN BUS STATUS ===");
        Serial.print("Total RX messages in last 10s: ");
        Serial.println(total_rx_messages);
        Serial.print("Config option: ");
        Serial.println(CAN_CONFIG_OPTION);

        if (total_rx_messages == 0)
        {
            Serial.println("⚠️  WARNING: NO CAN RX detected!");
            Serial.println("Since you confirmed gimbal replies with USB adapter:");
            Serial.println("• Issue is Teensy CAN RX configuration");
            Serial.println("• Try different CAN_CONFIG_OPTION (1, 2, or 3)");
            Serial.println("• Check Teensy CAN pins: CAN1_RX=23, CAN1_TX=22");
            Serial.println("• Verify 120Ω termination resistors");
        }
        else
        {
            Serial.println("✓ CAN RX working - check messages above");
        }
        Serial.println("========================\n");

        // Reset counter for next period
        total_rx_messages = 0;
    }

    delay(10);
}

// =============================================================================
// UTILITY FUNCTIONS (FOR TESTING)
// =============================================================================

void testGimbalPosition()
{
    setGimbalPosition(0, 0, 0, 0x01, 30); // Center position, 3 seconds
}

void testPrintStatus()
{
    printGimbalStatus(); // From gimbal_position_readout module

    float yaw, roll, pitch;
    if (getCurrentGimbalAngles(&yaw, &roll, &pitch))
    {
        Serial.println("Current angles are valid and recent");
    }
    else
    {
        Serial.println("No recent angle data available");
    }
}
