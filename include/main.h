#ifndef MAIN_H_
#define MAIN_H_

#include <Arduino.h>
#include <FlexCAN_T4.h>

// =============================================================================
// Constants
// =============================================================================

// DJI CAN communication constants
const uint16_t kDjiCanTxId = 0x223;
const uint16_t kDjiCanRxId = 0x222;
const uint32_t kDjiBaudRate = 1000000;

// DJI gimbal control ranges and defaults (in 0.1 degree steps)
const int16_t kDefaultTargetYaw = 1800;    // Range: -1800 to 1800 (-180° to 180°)
const int16_t kDefaultTargetRoll = 200;   // Range: -300 to 300 (-30° to 30°)
const int16_t kDefaultTargetPitch = 500;   // Range: -560 to 1460 (-56° to 146°)

// DJI command packet constants
const uint8_t kDjiPacketStartMarker = 0xAA;
const uint8_t kDjiPacketLength = 0x1A;
const uint8_t kDjiPayloadLength = 0x0E;
const uint8_t kDefaultControlMode = 0x01;
const uint8_t kDefaultActionTime = 20;

// Timing constants
const unsigned long kSendIntervalMs = 5000;  // Send command every 5 seconds
const unsigned long kAngleRequestIntervalMs = 500;  // Request angles every 500ms (2 Hz)
const unsigned long kFrameDelayUs = 500;    // Delay between CAN frames
const unsigned long kLoopDelayMs = 10;      // Main loop delay

// CRC algorithm constants
const uint16_t kCrc16InitialValue = 0xc55c;
const uint16_t kCrc16Polynomial = 0x8005;
const uint32_t kCrc32InitialValue = 0xc55c0000;
const uint32_t kCrc32Polynomial = 0x04c11db7;

// =============================================================================
// Global Variables
// =============================================================================

// CAN bus interface
extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can0;

// Current gimbal target positions (in 0.1 degree steps)
extern int16_t target_yaw;
extern int16_t target_roll;
extern int16_t target_pitch;
extern uint8_t control_mode;
extern uint8_t action_time;

// Timing variables
extern unsigned long last_send_time;
extern unsigned long last_angle_request_time;

// =============================================================================
// Function Declarations
// =============================================================================

/**
 * @brief Reflects (reverses) the bits in an 8-bit value
 * @param data The 8-bit value to reflect
 * @return The bit-reflected value
 * 
 * This function is required for DJI's CRC calculation algorithm.
 * It reverses the bit order so that MSB becomes LSB and vice versa.
 */
uint8_t Reflect8(uint8_t data);

/**
 * @brief Reflects (reverses) the bits in a 16-bit value
 * @param data The 16-bit value to reflect
 * @return The bit-reflected value
 */
uint16_t Reflect16(uint16_t data);

/**
 * @brief Reflects (reverses) the bits in a 32-bit value
 * @param data The 32-bit value to reflect
 * @return The bit-reflected value
 */
uint32_t Reflect32(uint32_t data);

/**
 * @brief Calculates 16-bit CRC using DJI's specific algorithm
 * @param data Pointer to the data buffer
 * @param length Number of bytes to process
 * @return 16-bit CRC checksum
 * 
 * Uses DJI's custom CRC16 implementation with specific initial value,
 * polynomial, and bit reflection for header validation.
 */
uint16_t CalculateCrc16Dji(const uint8_t* data, uint16_t length);

/**
 * @brief Calculates 32-bit CRC using DJI's specific algorithm
 * @param data Pointer to the data buffer
 * @param length Number of bytes to process
 * @return 32-bit CRC checksum
 * 
 * Uses DJI's custom CRC32 implementation for full packet validation.
 */
uint32_t CalculateCrc32Dji(const uint8_t* data, uint16_t length);

/**
 * @brief Sends a data packet over CAN bus in 8-byte frames
 * @param packet Pointer to the packet data
 * @param length Total packet length in bytes
 * 
 * Automatically fragments packets larger than 8 bytes into multiple
 * CAN frames and transmits them with appropriate timing delays.
 */
void SendPacketOverCan(const uint8_t* packet, uint8_t length);

/**
 * @brief Constructs and sends a DJI position command packet
 * 
 * Creates a 26-byte DJI command packet with proper header, payload,
 * and CRC checksums, then transmits it via CAN bus.
 */
void SendDjiPositionCommand();

/**
 * @brief Initializes CAN bus communication for DJI protocol
 * 
 * Sets up CAN interface with appropriate baud rate, filters,
 * and message handling for DJI device communication.
 */
void SetupCanCommunication();

/**
 * @brief Requests current gimbal angle information from DJI device
 * @param request_type Type of angles to request (0x01 = attitude, 0x02 = joint)
 * 
 * Sends a DJI angle information request command. The response will be
 * handled by the CAN message callback function.
 */
void RequestGimbalAngles(uint8_t request_type);

/**
 * @brief Handles incoming DJI angle response messages
 * @param msg Reference to the received CAN message
 * 
 * Processes angle information responses from the DJI device and
 * extracts yaw, roll, and pitch values for monitoring.
 */
void HandleAngleResponse(const CAN_message_t& msg);

/**
 * @brief Processes a complete DJI response packet from multiple CAN frames
 * @param packet_data Pointer to the complete packet data
 * @param packet_length Length of the complete packet
 * 
 * Parses the reconstructed DJI response packet and handles different
 * command responses (angles, status, etc.).
 */
void ProcessDjiResponse(const uint8_t* packet_data, uint8_t packet_length);

#endif  // MAIN_H_
