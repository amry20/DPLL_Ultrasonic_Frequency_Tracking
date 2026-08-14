#pragma once
/*
    * com.h
    *
    * Serial communication for the DPLL project.
    *
    * This module handles the UART interface for status reporting and command input.
    * It is initialized in main.cpp setup() and runs in the main loop.
    *
    * The STM32F407 has multiple USARTs; this project uses USART1 (PA9=TX, PA10=RX).    
    * Protocol:
    *   *   - Header 8 byte
    *   *   *   - Byte 0: 0xAA (start byte)
    *   *   *   - Byte 1 - 2: Opcode (uint16_t, little-endian) 
    *   *   *   - Byte 3 - 4: Address (uint16_t, little-endian) 
    *   *   *   - Byte 5 - 6: Length + 1 check sum (uint16_t, little-endian) 
    *   *   *   - Byte 7: End byte (0xBB)
    *   *   - Payload variable length
*/
#include <Arduino.h>
#define COM_HEADER_SIZE 8
#define COM_PAYLOAD_MAX_SIZE 512
#define START_BYTE 0xAA
#define END_BYTE 0xBB
typedef struct ComHeader
{
    uint8_t startByte;
    uint16_t opcode;
    uint16_t address;
    uint16_t length;
    uint8_t EndByte;
}__attribute__((packed))ComHeader;
typedef struct ComPacket
{
    ComHeader header;
    uint8_t payload[COM_PAYLOAD_MAX_SIZE];
}__attribute__((packed))ComPacket;
namespace com {
  void begin();

  // Build a complete frame into `buffer` and return its total length in bytes.
  //
  // Frame layout (little-endian multi-byte fields):
  //   [0]     START_BYTE (0xAA)
  //   [1..2]  opcode   (uint16 LE)
  //   [3..4]  address  (uint16 LE)
  //   [5..6]  length   = payloadLength + 1 (the +1 is the checksum byte)
  //   [7]     END_BYTE (0xBB)
  //   [8..]   payload  (payloadLength bytes)
  //   [last]  checksum (8-bit sum of the payload bytes)
  //
  // `buffer` must hold at least COM_HEADER_SIZE + payloadLength + 1 bytes.
  uint16_t makePacket(uint8_t* buffer, uint16_t opcode, uint16_t address,
                      const uint8_t* payload, uint16_t payloadLength);

  // Build a ComPacket from the given fields and push it onto the TX queue.
  // Returns true if the packet was queued, false if the payload is too large
  // or the queue is full.
  bool sendPacket(uint16_t opcode, uint16_t address,
                  const uint8_t* payload, uint16_t payloadLength);
}