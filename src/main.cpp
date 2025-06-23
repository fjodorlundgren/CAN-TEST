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

// =============================================================================
// Bit Reflection Functions
// =============================================================================

uint8_t Reflect8(uint8_t data) {
  uint8_t reflection = 0;
  for (int bit = 0; bit < 8; bit++) {
    if (data & (1 << bit)) {
      reflection |= (1 << (7 - bit));
    }
  }
  return reflection;
}

uint16_t Reflect16(uint16_t data) {
  uint16_t reflection = 0;
  for (int bit = 0; bit < 16; bit++) {
    if (data & (1 << bit)) {
      reflection |= (1 << (15 - bit));
    }
  }
  return reflection;
}

uint32_t Reflect32(uint32_t data) {
  uint32_t reflection = 0;
  for (int bit = 0; bit < 32; bit++) {
    if (data & (1 << bit)) {
      reflection |= (1 << (31 - bit));
    }
  }
  return reflection;
}

// =============================================================================
// CRC Calculation Functions
// =============================================================================

uint16_t CalculateCrc16Dji(const uint8_t* data, uint16_t length) {
  uint16_t crc = kCrc16InitialValue;
  
  for (uint16_t i = 0; i < length; i++) {
    uint8_t byte = Reflect8(data[i]);
    crc ^= static_cast<uint16_t>(byte) << 8;
    
    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ kCrc16Polynomial;
      } else {
        crc <<= 1;
      }
    }
  }
  
  return Reflect16(crc);
}

uint32_t CalculateCrc32Dji(const uint8_t* data, uint16_t length) {
  uint32_t crc = kCrc32InitialValue;
  
  for (uint16_t i = 0; i < length; i++) {
    uint8_t byte = Reflect8(data[i]);
    crc ^= static_cast<uint32_t>(byte) << 24;
    
    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x80000000) {
        crc = (crc << 1) ^ kCrc32Polynomial;
      } else {
        crc <<= 1;
      }
    }
  }
  
  return Reflect32(crc);
}

// =============================================================================
// CAN Communication Functions
// =============================================================================

void SetupCanCommunication() {
  can0.begin();
  can0.setBaudRate(kDjiBaudRate);
  can0.setMaxMB(16);
  can0.enableFIFO();
  can0.enableFIFOInterrupt();
  
  // Configure message filtering to accept only DJI response messages
  can0.setFIFOFilter(REJECT_ALL);
  can0.setFIFOFilter(0, kDjiCanRxId, STD);
  
  // Set up message reception callback
  can0.onReceive([](const CAN_message_t& msg) {
    // Handle received DJI response messages here
    // This could be expanded to parse status responses
  });
}

void SendPacketOverCan(const uint8_t* packet, uint8_t length) {
  const int frames = (length + 7) / 8;  // Calculate required number of frames
  
  for (int frame = 0; frame < frames; frame++) {
    CAN_message_t msg;
    msg.id = kDjiCanTxId;
    msg.flags.extended = false;
    msg.flags.remote = false;
    
    const int start_byte = frame * 8;
    const int bytes_in_frame = min(8, length - start_byte);
    msg.len = bytes_in_frame;
    
    // Copy data to CAN message buffer
    for (int i = 0; i < bytes_in_frame; i++) {
      msg.buf[i] = packet[start_byte + i];
    }
    
    can0.write(msg);
    delayMicroseconds(kFrameDelayUs);
  }
}

// =============================================================================
// DJI Command Functions
// =============================================================================

void SendDjiPositionCommand() {
  uint8_t packet[26];
  uint16_t packet_index = 0;
  
  // Construct packet header (10 bytes)
  packet[packet_index++] = kDjiPacketStartMarker;  // 0xAA - Start marker
  packet[packet_index++] = kDjiPacketLength;       // 0x1A - Total packet length
  packet[packet_index++] = 0x00;                   // Reserved
  packet[packet_index++] = 0x03;                   // Command type
  packet[packet_index++] = 0x00;                   // Reserved
  packet[packet_index++] = 0x00;                   // Reserved
  packet[packet_index++] = 0x00;                   // Reserved
  packet[packet_index++] = 0x00;                   // Reserved
  packet[packet_index++] = 0x22;                   // Device address
  packet[packet_index++] = 0x11;                   // Source address
  
  // Calculate and add CRC16 for header
  const uint16_t crc16 = CalculateCrc16Dji(packet, 10);
  packet[packet_index++] = crc16 & 0xFF;           // CRC16 low byte
  packet[packet_index++] = (crc16 >> 8) & 0xFF;   // CRC16 high byte
  
  // Construct payload (10 bytes)
  packet[packet_index++] = kDjiPayloadLength;      // 0x0E - Payload length
  packet[packet_index++] = 0x00;                   // Reserved
  
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
  packet[packet_index++] = crc32 & 0xFF;           // CRC32 byte 0
  packet[packet_index++] = (crc32 >> 8) & 0xFF;   // CRC32 byte 1
  packet[packet_index++] = (crc32 >> 16) & 0xFF;  // CRC32 byte 2
  packet[packet_index++] = (crc32 >> 24) & 0xFF;  // CRC32 byte 3
  
  SendPacketOverCan(packet, 26);
}

// =============================================================================
// Arduino Framework Functions
// =============================================================================

void setup() {
  SetupCanCommunication();
}

void loop() {
  // Process incoming CAN messages and handle callbacks
  can0.events();
  
  // Send DJI position command at regular intervals
  if (millis() - last_send_time >= kSendIntervalMs) {
    last_send_time = millis();
    SendDjiPositionCommand();
  }
  
  delay(kLoopDelayMs);
}
