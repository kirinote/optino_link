/*
 * Plastic Optical Fiber Shield
 *  designed for Arduino UNO R4
 *
 * Optino Link Unit Header File
 * OLU.h
 *
 * Optino Link Protocol
 *
 * Version History:
 *   v1.0 - Initial release
 *
 * Physical Layer Assumption:
 * Half-duplex optical ring network.
 * Packets are forwarded downstream.
 * No collision detection at physical layer.
 *
 * Network Assumption:
 * Single host unit initiates ASSIGN.
 * Concurrent ASSIGN frames from multiple hosts
 * are undefined.
 *
 * USB Bridge Assumption:
 * USB (PC link) is treated as a trusted channel.
 * CRC validation is NOT performed on USB ingress.
 * CRC protection applies only to the optical ring segment.
 * CRC is regenerated on USB-to-ring forwarding.
 *
 * Rationale:
 * The USB link is assumed to be reliable and point-to-point.
 * End-to-end integrity protection is required only for
 * the optical ring network where physical errors may occur.
 *
 * Packet Structure (fixed length):
 * Initial definition: 6 bytes total
 *
 *   Byte 0: Header (0x55)
 *   Byte 1: Destination UID
 *   Byte 2: Source UID
 *   Byte 3: Command
 *   Byte 4: Payload (PAYLOAD_SIZE = 1)
 *   Byte 5: CRC-8-ATM over bytes [0..4]
 *
 * Synchronization:
 * Receiver resynchronizes using header detection
 * with lightweight restart strategy on CRC failure.
 * (No full sliding window reconstruction.)
 * 
 * Reserved UID:
 *   0x00 - Repeater, Static UID unit
 *   0xFE - Broadcast, All Active UID units
 *   0xFF - Unassigned, Active UID unit
 *
 * Command:
 *   0x00 - Not applicable
 *   0x01 - Assign
 *   0x02 - Assign done
 *   0x03 - I/O Output control
 *   0x04 - I/O Input capture
 *   0x05 - Data transfer request
 *   0x06 - Response as application layer
 *
 * EEPROM Address:
 *   0x0000 UID (Flash emulated EEPROM on UNO R4)
 *
 * Security:
 * No authentication or encryption.
 * Network is assumed trusted.
 *
 * Technical Restriction:
 * On R4 Minima, the default I2C port is unavailable.
 * Use Wire1 on R4 WiFi instead.
 *
 * Copyright (c) 2026 K. Yoshi
 * This software is released under the MIT License.
 * https://opensource.org/licenses/MIT
 */

#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

/* Conditional compilation */
//#define PERSISTENT_UID
//#define USE_EXT_EEPROM
#define ENABLE_PC_COMM

#if defined(USE_EXT_EEPROM) && !defined(PERSISTENT_UID)
#error "USE_EXT_EEPROM requires PERSISTENT_UID to be defined."
#endif
#if defined(USE_EXT_EEPROM) && !defined(ARDUINO_UNOR4_WIFI)
#warning "External EEPROM is intended for UNO R4 WiFi."
#endif

#define TFT_SCLK 13
#define TFT_MOSI 11
#define TFT_CS 10
#define TFT_BL 9
#define TFT_DC 8
#define TFT_RST -1

#define HEADER 0x55
#define ASSIGN 0x01
#define ASSIGN_DONE 0x02
#define OUT_CTRL 0x03
#define IN_CAPT 0x04
#define REQUEST 0x05
#define RESPONSE 0x06

#define EEPROM_UID_ADDR 0x0000
#define EXT_EEPROM_ADDR 0x50

/* 0x00 = repeater role
 * Transparent forwarding unit.
 *
 * Characteristics:
 *   - Non-addressable (never participates in logical UID space)
 *   - Does NOT accept ASSIGN (UID remains 0x00)
 *   - Forwards ALL frames unchanged, including ASSIGN
 *   - Executes no application-layer commands
 *
 * Purpose:
 * Extends physical reach without increasing
 * logical unit count or affecting UID chain.
 */
#define REPEATER 0x00

/* Maximum assignable UID */
#define UID_MAX 0xFD

/* 0xFE = broadcast to assigned units only.
 * 0xFF excluded by design.
 */
#define ACT_UNITS 0xFE

/* PWM duty inverted.
 * TFT display backlight is turned off by conducting
 * between the drain and source of the MOSFET.
 */
#define BACKLIGHT_MAX 0x00  // ON
#define BACKLIGHT_DIM 0xFF  // OFF

#define ASSIGN_TIMEOUT 500UL

#define PAYLOAD_SIZE 1                  // Payload size per packet
#define PACKET_SIZE (PAYLOAD_SIZE + 5)  // Head, Dest, Src, Cmd, Pay, CRC
#define DATA_LENGTH (PACKET_SIZE - 1)   // CRC excluded
#define RX_BUF_SIZE 4096                // Ring buffer size
#define RX_BUF_MASK (RX_BUF_SIZE - 1)   // Buffer mask

#if (RX_BUF_SIZE & (RX_BUF_SIZE - 1)) != 0
#error "RX_BUF_SIZE must be power of two"
#endif

/* RX buffer (polled context only, non-ISR safe):
 * 4 kBytes ring buffer.
 * Large size chosen to tolerate GUI latency.
 * Drops bytes on buffer overflow.
 * Overflow counter stops incrementing at 0xFF.
 * Adjust size according to available SRAM.
 */
byte rxBuf[RX_BUF_SIZE];
uint16_t rxHead = 0;
uint16_t rxTail = 0;

/* GUI update flags (set by protocol handlers):
 * All flags are reset by updateGUI().
 */
bool uidUpdated = 0;
bool rxdUpdated = 0;
bool creUpdated = 0;
bool usgUpdated = 0;
bool rndUpdated = 0;

/* assignLock:
 * True if a UID ASSIGN process is ongoing,
 * preventing duplicate starts and managing timeout.
 */
bool assignLock = 0;

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
UART OptLink(A4, A5);  // RA4M1 Board dependent hardware UART

const int buttonPins[4] = { A0, A1, A2, A3 };
const int inputPins[4] = { D0, D1, D2, D3 };
const int outputPins[4] = { D4, D5, D6, D7 };
bool lastStates[4] = { 0, 0, 0, 0 };
bool newHost = 0;        // Source of Assign command
byte uid = 0xFF;         // My UID
byte highestUID = 0xFF;  // Highest UID assigned in the last ASSIGN cycle

/* Global TX context:
 * Reserved for future asynchronous TX state handling.
 */
byte dest = 0;  // Destination UID
byte cmd = 0;   // Command No.

byte randVal;                 // Random value
byte crcErr = 0;              // Number of CRC errors
byte rxOVFlow = 0;            // Number of Rx buffer overflow
byte userData[PAYLOAD_SIZE];  // User data
byte rxd[PACKET_SIZE];        // Received packet
byte txd[PACKET_SIZE];        // Transmit packet
byte work[PACKET_SIZE];       // Stored working data
byte crc8_table_local[256];   // CRC table copy on SRAM
uint32_t assignST = 0;        // Assign start time

#ifdef ENABLE_PC_COMM
byte bridgeTx[DATA_LENGTH] = { 0 };  // USB to Ring staging buffer, CRC excluded
byte bridgeRx[DATA_LENGTH] = { 0 };  // Ring to USB single-slot buffer, CRC excluded
#endif

const byte crc8_table[256] PROGMEM = {
  0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
  0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
  0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
  0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
  0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
  0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
  0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
  0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
  0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
  0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
  0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
  0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
  0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
  0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
  0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
  0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
  0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
  0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
  0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
  0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
  0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
  0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
  0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
  0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
  0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
  0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
  0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
  0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
  0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
  0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
  0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
  0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};
