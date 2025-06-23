#include "main.h"

// =============================================================================
// Global Variable Definitions for Parameter Push Test
// =============================================================================

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can0;

// Variables to track parameter push timing
unsigned long last_parameter_push_time = 0;
unsigned long parameter_push_count = 0;
bool parameter_push_enabled = false;

// Current angle storage
float current_attitude_yaw = 0.0f;
float current_attitude_roll = 0.0f;
float current_attitude_pitch = 0.0f;
float current_joint_yaw = 0.0f;
float current_joint_roll = 0.0f;
float current_joint_pitch = 0.0f;

// =============================================================================
// Bit Reflection Functions (Required for CRC)
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
// CRC Calculation Functions (Required for DJI Protocol)
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

void SendPacketOverCan(const uint8_t* packet, uint8_t length) {
  const int frames = (length + 7) / 8;
  
  for (int frame = 0; frame < frames; frame++) {
    CAN_message_t msg;
    msg.id = kDjiCanTxId;
    msg.flags.extended = false;
    msg.flags.remote = false;
    
    const int start_byte = frame * 8;
    const int bytes_in_frame = min(8, length - start_byte);
    msg.len = bytes_in_frame;
    
    for (int i = 0; i < bytes_in_frame; i++) {
      msg.buf[i] = packet[start_byte + i];
    }
    
    can0.write(msg);
    delayMicroseconds(kFrameDelayUs);
  }
}

// =============================================================================
// Parameter Push Functions
// =============================================================================

void EnableParameterPush() {
  uint8_t packet[16];
  uint16_t packet_index = 0;
  
  // Construct packet header (10 bytes)
  packet[packet_index++] = kDjiPacketStartMarker;  // 0xAA - Start marker
  packet[packet_index++] = 0x10;                   // 0x10 - Total packet length (16 bytes)
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
  
  // Construct payload (4 bytes) - Parameter Push Enable Command
  packet[packet_index++] = 0x04;                   // Payload length
  packet[packet_index++] = 0x0E;                   // Command set
  packet[packet_index++] = 0x07;                   // Command ID (parameter push settings)
  packet[packet_index++] = 0x01;                   // ctrl_byte: 0x01 = Enable parameter push
  
  // Calculate and add CRC32 for entire packet
  const uint32_t crc32 = CalculateCrc32Dji(packet, 12);
  packet[packet_index++] = crc32 & 0xFF;           // CRC32 byte 0
  packet[packet_index++] = (crc32 >> 8) & 0xFF;   // CRC32 byte 1
  packet[packet_index++] = (crc32 >> 16) & 0xFF;  // CRC32 byte 2
  packet[packet_index++] = (crc32 >> 24) & 0xFF;  // CRC32 byte 3
  
  SendPacketOverCan(packet, 16);
  
  Serial.println("Parameter push enable command sent!");
}

void ProcessParameterPush(const uint8_t* packet_data, uint8_t packet_length) {
  // Verify this is a parameter push message (CmdSet=0x0E, CmdID=0x08)
  if (packet_length < 35) {  // Minimum size for parameter push
    return;
  }
  
  const uint8_t cmd_set = packet_data[13];
  const uint8_t cmd_id = packet_data[14];
  
  if (cmd_set == 0x0E && cmd_id == 0x08) {
    // This is a parameter push message
    parameter_push_count++;
    
    // Calculate time since last push (for frequency analysis)
    unsigned long current_time = millis();
    unsigned long time_since_last = current_time - last_parameter_push_time;
    last_parameter_push_time = current_time;
    
    // Extract control byte to see what data is valid
    const uint8_t ctrl_byte = packet_data[15];
    const bool angles_valid = (ctrl_byte & 0x01) != 0;
    const bool limits_valid = (ctrl_byte & 0x02) != 0;
    const bool stiffness_valid = (ctrl_byte & 0x04) != 0;
    
    if (angles_valid && packet_length >= 28) {
      // Extract attitude angles (offsets 16-21)
      const int16_t att_yaw = static_cast<int16_t>(
          packet_data[16] | (packet_data[17] << 8));
      const int16_t att_roll = static_cast<int16_t>(
          packet_data[18] | (packet_data[19] << 8));
      const int16_t att_pitch = static_cast<int16_t>(
          packet_data[20] | (packet_data[21] << 8));
      
      // Extract joint angles (offsets 22-27)
      const int16_t joint_yaw = static_cast<int16_t>(
          packet_data[22] | (packet_data[23] << 8));
      const int16_t joint_roll = static_cast<int16_t>(
          packet_data[24] | (packet_data[25] << 8));
      const int16_t joint_pitch = static_cast<int16_t>(
          packet_data[26] | (packet_data[27] << 8));
      
      // Convert to degrees
      current_attitude_yaw = att_yaw / 10.0f;
      current_attitude_roll = att_roll / 10.0f;
      current_attitude_pitch = att_pitch / 10.0f;
      current_joint_yaw = joint_yaw / 10.0f;
      current_joint_roll = joint_roll / 10.0f;
      current_joint_pitch = joint_pitch / 10.0f;
      
      // Display results with timing information
      Serial.printf("Push #%lu (Δt=%lums): ", parameter_push_count, time_since_last);
      Serial.printf("Att[Y:%.1f° R:%.1f° P:%.1f°] ", 
                    current_attitude_yaw, current_attitude_roll, current_attitude_pitch);
      Serial.printf("Joint[Y:%.1f° R:%.1f° P:%.1f°]\\n", 
                    current_joint_yaw, current_joint_roll, current_joint_pitch);
      
      // Calculate and display average frequency every 10 pushes
      if (parameter_push_count % 10 == 0 && parameter_push_count > 0) {
        float avg_frequency = 1000.0f / time_since_last;  // Approximate from last interval
        Serial.printf("*** Estimated push frequency: %.2f Hz ***\\n", avg_frequency);
      }
    }
  }
}

// Global variables for packet reconstruction
static uint8_t reconstruction_buffer[64];  // Larger buffer for parameter push
static uint8_t reconstruction_index = 0;
static bool reconstruction_in_progress = false;

void HandleParameterPushMessage(const CAN_message_t& msg) {
  if (!reconstruction_in_progress) {
    if (msg.len > 0 && msg.buf[0] == kDjiPacketStartMarker) {
      reconstruction_in_progress = true;
      reconstruction_index = 0;
    } else {
      return;
    }
  }
  
  // Copy data to reconstruction buffer
  for (int i = 0; i < msg.len && reconstruction_index < sizeof(reconstruction_buffer); i++) {
    reconstruction_buffer[reconstruction_index++] = msg.buf[i];
  }
  
  // Check if we have enough data to determine packet length
  if (reconstruction_index >= 2) {
    const uint8_t expected_length = reconstruction_buffer[1];
    
    if (reconstruction_index >= expected_length) {
      ProcessParameterPush(reconstruction_buffer, reconstruction_index);
      reconstruction_in_progress = false;
      reconstruction_index = 0;
    }
  }
  
  // Safety check
  if (reconstruction_index >= sizeof(reconstruction_buffer)) {
    reconstruction_in_progress = false;
    reconstruction_index = 0;
  }
}

void SetupCanCommunication() {
  can0.begin();
  can0.setBaudRate(kDjiBaudRate);
  can0.setMaxMB(16);
  can0.enableFIFO();
  can0.enableFIFOInterrupt();
  
  can0.setFIFOFilter(REJECT_ALL);
  can0.setFIFOFilter(0, kDjiCanRxId, STD);
  
  can0.onReceive([](const CAN_message_t& msg) {
    HandleParameterPushMessage(msg);
  });
}

// =============================================================================
// Arduino Framework Functions
// =============================================================================

void setup() {
  // Initialize Serial for monitoring
  Serial.begin(115200);
  delay(2000);  // Wait for Serial to initialize
  
  Serial.println("=== DJI Parameter Push Test ===");
  Serial.println("This test enables parameter push and monitors the frequency");
  Serial.println();
  
  // Setup CAN communication
  SetupCanCommunication();
  
  // Wait a moment for CAN to initialize
  delay(1000);
  
  // Enable parameter push
  EnableParameterPush();
  
  Serial.println("Waiting for parameter push messages...");
  Serial.println("Format: Push #count (Δt=time_ms): Attitude[Y R P] Joint[Y R P]");
  Serial.println();
}

void loop() {
  // Process incoming CAN messages
  can0.events();
  
  // Send parameter push enable command every 30 seconds as a keepalive
  static unsigned long last_keepalive = 0;
  if (millis() - last_keepalive >= 30000) {
    last_keepalive = millis();
    EnableParameterPush();
    Serial.println("Sent parameter push keepalive command");
  }
  
  // Print status every 10 seconds if no pushes received
  static unsigned long last_status = 0;
  if (millis() - last_status >= 10000) {
    last_status = millis();
    if (parameter_push_count == 0) {
      Serial.println("No parameter push messages received yet...");
    }
  }
  
  delay(10);
}
