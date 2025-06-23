#include "main.h"

// =============================================================================
// Global Variable Definitions
// =============================================================================

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can0;

int16_t target_yaw = kDefaultTargetYaw;
int16_t target_roll = kDefaultTargetRoll;
int16_t target_pitch = kDefaultTargetPitch;
uint8_t control_mode = kDefaultControlMode;
uint8_t action_time = kDefaultActionTime;

unsigned long last_send_time = 0;
unsigned long last_angle_request_time = 0;

// =============================================================================
// Bit Reflection Functions
// =============================================================================

uint8_t Reflect8(uint8_t data)
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

uint16_t Reflect16(uint16_t data)
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

uint32_t Reflect32(uint32_t data)
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

// =============================================================================
// CRC Calculation Functions
// =============================================================================

uint16_t CalculateCrc16Dji(const uint8_t *data, uint16_t length)
{
  uint16_t crc = kCrc16InitialValue;

  for (uint16_t i = 0; i < length; i++)
  {
    uint8_t byte = Reflect8(data[i]);
    crc ^= static_cast<uint16_t>(byte) << 8;

    for (int bit = 0; bit < 8; bit++)
    {
      if (crc & 0x8000)
      {
        crc = (crc << 1) ^ kCrc16Polynomial;
      }
      else
      {
        crc <<= 1;
      }
    }
  }

  return Reflect16(crc);
}

uint32_t CalculateCrc32Dji(const uint8_t *data, uint16_t length)
{
  uint32_t crc = kCrc32InitialValue;

  for (uint16_t i = 0; i < length; i++)
  {
    uint8_t byte = Reflect8(data[i]);
    crc ^= static_cast<uint32_t>(byte) << 24;

    for (int bit = 0; bit < 8; bit++)
    {
      if (crc & 0x80000000)
      {
        crc = (crc << 1) ^ kCrc32Polynomial;
      }
      else
      {
        crc <<= 1;
      }
    }
  }

  return Reflect32(crc);
}

// =============================================================================
// CAN Communication Functions
// =============================================================================

void SetupCanCommunication()
{
  can0.begin();
  can0.setBaudRate(kDjiBaudRate);
  can0.setMaxMB(16);
  can0.enableFIFO();
  can0.enableFIFOInterrupt();

  // Configure message filtering to accept only DJI response messages
  can0.setFIFOFilter(REJECT_ALL);
  can0.setFIFOFilter(0, kDjiCanRxId, STD);

  // Set up message reception callback
  can0.onReceive([](const CAN_message_t &msg)
                 {
    // Handle received DJI response messages
    HandleAngleResponse(msg); });
}

void SendPacketOverCan(const uint8_t *packet, uint8_t length)
{
  const int frames = (length + 7) / 8; // Calculate required number of frames

  for (int frame = 0; frame < frames; frame++)
  {
    CAN_message_t msg;
    msg.id = kDjiCanTxId;
    msg.flags.extended = false;
    msg.flags.remote = false;

    const int start_byte = frame * 8;
    const int bytes_in_frame = min(8, length - start_byte);
    msg.len = bytes_in_frame;

    // Copy data to CAN message buffer
    for (int i = 0; i < bytes_in_frame; i++)
    {
      msg.buf[i] = packet[start_byte + i];
    }

    can0.write(msg);
    delayMicroseconds(kFrameDelayUs);
  }
}

// =============================================================================
// DJI Command Functions
// =============================================================================

void SendDjiPositionCommand()
{
  uint8_t packet[26];
  uint16_t packet_index = 0;

  // Construct packet header (10 bytes)
  packet[packet_index++] = kDjiPacketStartMarker; // 0xAA - Start marker
  packet[packet_index++] = kDjiPacketLength;      // 0x1A - Total packet length
  packet[packet_index++] = 0x00;                  // Reserved
  packet[packet_index++] = 0x03;                  // Command type
  packet[packet_index++] = 0x00;                  // Reserved
  packet[packet_index++] = 0x00;                  // Reserved
  packet[packet_index++] = 0x00;                  // Reserved
  packet[packet_index++] = 0x00;                  // Reserved
  packet[packet_index++] = 0x22;                  // Device address
  packet[packet_index++] = 0x11;                  // Source address

  // Calculate and add CRC16 for header
  const uint16_t crc16 = CalculateCrc16Dji(packet, 10);
  packet[packet_index++] = crc16 & 0xFF;        // CRC16 low byte
  packet[packet_index++] = (crc16 >> 8) & 0xFF; // CRC16 high byte

  // Construct payload (10 bytes)
  packet[packet_index++] = kDjiPayloadLength; // 0x0E - Payload length
  packet[packet_index++] = 0x00;              // Reserved

  // Add gimbal position data (little-endian format)
  packet[packet_index++] = target_yaw & 0xFF;
  packet[packet_index++] = (target_yaw >> 8) & 0xFF;
  packet[packet_index++] = target_roll & 0xFF;
  packet[packet_index++] = (target_roll >> 8) & 0xFF;
  packet[packet_index++] = target_pitch & 0xFF;
  packet[packet_index++] = (target_pitch >> 8) & 0xFF;
  packet[packet_index++] = control_mode;
  packet[packet_index++] = action_time;

  // Calculate and add CRC32 for entire packet
  const uint32_t crc32 = CalculateCrc32Dji(packet, 22);
  packet[packet_index++] = crc32 & 0xFF;         // CRC32 byte 0
  packet[packet_index++] = (crc32 >> 8) & 0xFF;  // CRC32 byte 1
  packet[packet_index++] = (crc32 >> 16) & 0xFF; // CRC32 byte 2
  packet[packet_index++] = (crc32 >> 24) & 0xFF; // CRC32 byte 3

  SendPacketOverCan(packet, 26);
}

// =============================================================================
// Angle Reading Functions
// =============================================================================

void RequestGimbalAngles(uint8_t request_type)
{
  uint8_t packet[16];
  uint16_t packet_index = 0;

  // Construct packet header (10 bytes)
  packet[packet_index++] = kDjiPacketStartMarker; // 0xAA - Start marker
  packet[packet_index++] = 0x10;                  // 0x10 - Total packet length (16 bytes)
  packet[packet_index++] = 0x00;                  // Reserved
  packet[packet_index++] = 0x03;                  // Command type
  packet[packet_index++] = 0x00;                  // Reserved
  packet[packet_index++] = 0x00;                  // Reserved
  packet[packet_index++] = 0x00;                  // Reserved
  packet[packet_index++] = 0x00;                  // Reserved
  packet[packet_index++] = 0x22;                  // Device address
  packet[packet_index++] = 0x11;                  // Source address

  // Calculate and add CRC16 for header
  const uint16_t crc16 = CalculateCrc16Dji(packet, 10);
  packet[packet_index++] = crc16 & 0xFF;        // CRC16 low byte
  packet[packet_index++] = (crc16 >> 8) & 0xFF; // CRC16 high byte

  // Construct payload (4 bytes)
  packet[packet_index++] = 0x04;         // Payload length
  packet[packet_index++] = 0x0E;         // Command set
  packet[packet_index++] = 0x02;         // Command ID (angle request)
  packet[packet_index++] = request_type; // Request type (0x01=attitude, 0x02=joint)

  // Calculate and add CRC32 for entire packet
  const uint32_t crc32 = CalculateCrc32Dji(packet, 12);
  packet[packet_index++] = crc32 & 0xFF;         // CRC32 byte 0
  packet[packet_index++] = (crc32 >> 8) & 0xFF;  // CRC32 byte 1
  packet[packet_index++] = (crc32 >> 16) & 0xFF; // CRC32 byte 2
  packet[packet_index++] = (crc32 >> 24) & 0xFF; // CRC32 byte 3

  SendPacketOverCan(packet, 16);
}

void ProcessDjiResponse(const uint8_t *packet_data, uint8_t packet_length)
{
  // Minimum DJI packet size check
  if (packet_length < 16)
  {
    return;
  }

  // Verify packet start marker
  if (packet_data[0] != kDjiPacketStartMarker)
  {
    return;
  }

  // Extract command information from payload
  const uint8_t cmd_set = packet_data[13]; // Offset 13 in full packet
  const uint8_t cmd_id = packet_data[14];  // Offset 14 in full packet

  // Handle gimbal command set responses
  if (cmd_set == 0x0E && cmd_id == 0x02)
  {
    // Angle information response
    if (packet_length >= 22)
    { // Ensure enough data for angle response
      const uint8_t return_code = packet_data[15];
      const uint8_t data_type = packet_data[16];

      if (return_code == 0x00)
      { // Success
        // Extract angles (little-endian format, 0.1 degree resolution)
        const int16_t yaw = static_cast<int16_t>(
            packet_data[17] | (packet_data[18] << 8));
        const int16_t roll = static_cast<int16_t>(
            packet_data[19] | (packet_data[20] << 8));
        const int16_t pitch = static_cast<int16_t>(
            packet_data[21] | (packet_data[22] << 8));

        // Convert to actual degrees (divide by 10)
        const float yaw_deg = yaw / 10.0f;
        const float roll_deg = roll / 10.0f;
        const float pitch_deg = pitch / 10.0f;

        // Determine angle type and display accordingly
        const char *angle_type = (data_type == 0x01) ? "Attitude" : "Joint";

        // For now, you can add Serial output here for debugging
        // Serial.printf("%s Angles - Yaw: %.1f°, Roll: %.1f°, Pitch: %.1f°\n",
        //               angle_type, yaw_deg, roll_deg, pitch_deg);

        // TODO: Store angles in separate global variables for attitude vs joint
      }
    }
  }
}

// Global variables for packet reconstruction
static uint8_t reconstruction_buffer[32];
static uint8_t reconstruction_index = 0;
static bool reconstruction_in_progress = false;

void HandleAngleResponse(const CAN_message_t &msg)
{
  // Simple packet reconstruction logic
  // This assumes packets are received in order without interleaving

  if (!reconstruction_in_progress)
  {
    // Check if this looks like the start of a DJI packet
    if (msg.len > 0 && msg.buf[0] == kDjiPacketStartMarker)
    {
      reconstruction_in_progress = true;
      reconstruction_index = 0;
    }
    else
    {
      return; // Not a valid start frame
    }
  }

  // Copy data to reconstruction buffer
  for (int i = 0; i < msg.len && reconstruction_index < sizeof(reconstruction_buffer); i++)
  {
    reconstruction_buffer[reconstruction_index++] = msg.buf[i];
  }

  // Check if we have enough data to determine packet length
  if (reconstruction_index >= 2)
  {
    const uint8_t expected_length = reconstruction_buffer[1];

    // Check if we have received the complete packet
    if (reconstruction_index >= expected_length)
    {
      ProcessDjiResponse(reconstruction_buffer, reconstruction_index);
      reconstruction_in_progress = false;
      reconstruction_index = 0;
    }
  }

  // Safety check - reset if buffer is getting too full
  if (reconstruction_index >= sizeof(reconstruction_buffer))
  {
    reconstruction_in_progress = false;
    reconstruction_index = 0;
  }
}

// =============================================================================
// Arduino Framework Functions
// =============================================================================

void setup()
{
  SetupCanCommunication();
}

void loop()
{
  // Process incoming CAN messages and handle callbacks
  can0.events();

  // Send DJI position command at regular intervals (every 5 seconds)
  if (millis() - last_send_time >= kSendIntervalMs)
  {
    last_send_time = millis();
    SendDjiPositionCommand();
  }

  // Request gimbal angles at 2 Hz (every 500ms), alternating between types
  if (millis() - last_angle_request_time >= kAngleRequestIntervalMs)
  {
    last_angle_request_time = millis();

    // Alternate between attitude and joint angles to get both types
    static bool request_attitude = true;
    if (request_attitude)
    {
      RequestGimbalAngles(0x01); // Request attitude angles
    }
    else
    {
      RequestGimbalAngles(0x02); // Request joint angles
    }
    request_attitude = !request_attitude; // Toggle for next request
  }

  delay(kLoopDelayMs);
}
