/*
 * Plastic Optical Fiber Shield
 *  designed for Arduino UNO R4
 *
 * Optino Link Unit Explanatory Sketch
 * OLU1_hello_world.ino
 *
 * Copyright (c) 2026 K. Yoshi
 * This software is released under the MIT License.
 * https://opensource.org/licenses/MIT
 */

#include "OLU.h"

/* Parser context */
static byte pktIndex = 0;
static byte pktCRC = 0;

/* Parser reset */
inline void resetParser() {
  pktIndex = 0;
  pktCRC = 0;
}

void setup(void) {

#ifdef USE_EXT_EEPROM
  Wire1.begin();
  Wire1.setClock(400000);
#endif

#ifdef ENABLE_PC_COMM
  Serial.begin(115200);
  uint32_t start = millis();
  while (!Serial && (millis() - start < 2000)) {};
#endif

  restoreUID();
  initPins();
  initDisplay();
  renderLabels();
  initCRCTable();
  randVal = randomGen();
  initGUI();
  OptLink.begin(4000000);
}

void loop() {

#ifdef ENABLE_PC_COMM
  /* USB to Optical Ring bridge */
  handleUsbToRing();
#endif

  /* Stage UART RX bytes into ring buffer (polled mode) */
  while (OptLink.available()) {
    storeRX(OptLink.read());
  }

  byte b;

  /* Fixed-length frame parser
   *   - Uses single static buffer.
   *   - No dynamic memory allocation.
   *   - Operates in streaming manner.
   */
  while (readRX(b)) {

    /* Bounds check */
    if (pktIndex >= PACKET_SIZE) {
      resetParser();
      continue;
    }

    if (pktIndex == 0) {
      if (b == HEADER) {
        rxd[0] = b;
        pktCRC = crc8_table_local[b];
        pktIndex = 1;
      }
      continue;
    }

    rxd[pktIndex] = b;

    if (pktIndex < DATA_LENGTH) {
      pktCRC = crc8_table_local[pktCRC ^ b];
      pktIndex++;
    } else {
      if (pktCRC == b) {
        parsePacket();
        resetParser();
      } else {

        /* CRC failure:
         * Discard current frame and restart at next HEADER.
         * Only performs lightweight resynchronization.
         * Partial or lost frames may occur under high load
         * or burst errors.
         */
        if (crcErr < 0xFF) crcErr++;
        creUpdated = 1;

        if (b == HEADER) {
          rxd[0] = HEADER;
          pktCRC = crc8_table_local[HEADER];
          pktIndex = 1;
        } else {
          resetParser();
        }
      }
    }
  }

  /* GUI is refreshed every 200 mSec.
   * Heavy SPI activity may delay UART servicing.
   */
  static uint32_t lastGUI = 0;
  const uint32_t GUI_INTERVAL = 200;
  uint32_t now = millis();
  if (now - lastGUI >= GUI_INTERVAL) {
    lastGUI = now;
    updateGUI();
  }

  /* ASSIGN timeout supervision */
  if (assignLock) {

    /* Timeout aborts current re-enumeration process
     * and resets this unit to unassigned state.
     */
    if (millis() - assignST > ASSIGN_TIMEOUT) {
      assignLock = 0;
      newHost = 0;
      crcErr = 0;
      highestUID = 0xFF;

      /* Reset UID to unassigned */
      if (uid != 0xFF) {
        uid = 0xFF;
        storeUID();
      }
      uidUpdated = 1;
      creUpdated = 1;
    }
  }

  /* Button polling (main loop only):
   * Detects rising edge, triggers action()
   *
   * Design prerequisite:
   * Button inputs must be externally debounced.
   * This module does not implement debounce logic.
   */
  checkButton();

#ifdef ENABLE_PC_COMM
  /* Optical Ring to USB bridge */
  handleRingToUsb();
#endif
}

/* Protocol State Flow Overview
 *
 * Unit roles:
 *   - Host (initiates ASSIGN)
 *   - Repeater (UID=0x00)
 *   - Dynamic Unit (UID=0x01-0xFD)
 *
 * Assignment sequence:
 * Each downstream unit overwrites its current UID
 * with (received UID + 1) and forwards the packet.
 *
 * State Evaluation Order:
 *   1. Ignore own packet (loop prevention)
 *   2. Repeater forwarding
 *   3. ASSIGN handling
 *   4. OUT_CTRL execution
 *   5. REQUEST -> RESPONSE
 *
 * Own packets are ignored except ASSIGN frames,
 * which are required for enumeration completion.
 *
 * CRC failure:
 * Header-based lightweight resynchronization is used.
 * Full sliding-window reconstruction is NOT performed.
 * On CRC mismatch, the current frame is discarded and
 * synchronization restarts only if the received byte
 * equals HEADER (0x55).
 *
 * Topology requirement:
 * Designed for single downstream path per unit.
 * Behavior is undefined for branching or mesh topology.
 */

/* parsePacket():
 * Handles a single fixed-length packet.
 *   - Assumes linear downstream path (no branching).
 *   - UID assignment strictly sequential.
 *   - Host-only ASSIGN initiation.
 *   - Lightweight CRC resynchronization.
 */
void parsePacket() {

  bool deliverToUsb = 0;
  bool fromMe = (rxd[2] == uid);
  bool notForMe = (rxd[1] != uid && rxd[1] != DYN_UNITS);
  bool notAssign = (rxd[3] != ASSIGN);

  /* Invalid destination filtering
   * Drop packets targeting non-existent UID to prevent ring circulation.
   *
   * Condition:
   *   dest > highestUID AND dest <= UID_MAX
   *
   * Exclusions:
   *   - Broadcast (0xFE)
   *   - Unassigned (0xFF)
   *   - Repeater (0x00) [safety exclusion]
   *
   * Note:
   *   highestUID == 0xFF means "unknown / not assigned yet"
   *   -> filtering is disabled in that state.
   */
  if (highestUID != 0xFF) {
    byte destUID = rxd[1];
    // clang-format off
    if (destUID != DYN_UNITS && 
        destUID != 0xFF &&
        destUID != REPEATER &&
        destUID > highestUID &&
        destUID <= UID_MAX) return;
    // clang-format on
  }

  if (fromMe && notAssign) return;

  /* Repeater:
   * Pure transparent forwarding.
   * Does not participate in any logical processing.
   * Forwards ALL frames including ASSIGN.
   */
  else if (uid == REPEATER) {
    OptLink.write(rxd, PACKET_SIZE);
  }

  /* Forward packet when not addressed to this unit
   * (non-ASSIGN only, dynamic units only)
   */
  else if (notForMe && notAssign) {
    OptLink.write(rxd, PACKET_SIZE);
  }

  /* Sequential UID reassignment.
   * Each downstream unit attempts to set UID = (payload + 1)
   * and forwards the frame.
   *
   * Note: timing delays or network topology violations may cause
   * UID duplication or skipped values.
   */
  else if (rxd[3] == ASSIGN) {

    /* Host completion detection */
    if (newHost && rxd[2] == uid) {
      assignLock = 0;
      newHost = 0;
#ifdef PERSISTENT_UID
      storeUID();
#endif
      highestUID = rxd[4];
      userData[0] = highestUID;
      optTX(DYN_UNITS, ASSIGN_DONE);
      randVal = randomGen();
      crcErr = 0;
      rxdUpdated = 1;
      rndUpdated = 1;
      creUpdated = 1;
      return;
    }

    /* Downstream re-enumeration */
    if (newHost == 0) {
      if (rxd[4] < UID_MAX) {
        if (!assignLock) {
          assignLock = 1;
          assignST = millis();
        }
        txdClear();
        rxd2txd();

        /* Payload carries the most recently assigned UID value */
        uid = rxd[4] + 1;
        txd[4] = uid;
        txd[DATA_LENGTH] = calcCRC8(txd, DATA_LENGTH);
        OptLink.write(txd, PACKET_SIZE);
        randVal = randomGen();
        crcErr = 0;
        uidUpdated = 1;
        rxdUpdated = 1;
        rndUpdated = 1;
        creUpdated = 1;
      }
    }
  }

  else if (rxd[3] == ASSIGN_DONE) {
    highestUID = rxd[4];
    assignLock = 0;
    if (uid == REPEATER || rxd[2] != uid) {
      OptLink.write(rxd, PACKET_SIZE);
    }
#ifdef PERSISTENT_UID
    storeUID();
#endif
  }

  else if (rxd[1] == uid && rxd[3] == OUT_CTRL) {
    FourBitOut(rxd[4] & 0x0F);
    rxdUpdated = 1;
    deliverToUsb = 1;
  }

  /* Transport characteristics:
   *   - CRC detects frame corruption only.
   *   - No retransmission.
   *   - No ordering guarantee.
   *   - No duplicate suppression.
   *
   * Higher-layer protocol must handle reliability.
   */
  else if (rxd[1] == uid && rxd[3] == REQUEST) {
    userData[0] = randVal;
    optTX(rxd[2], RESPONSE);
    rxdUpdated = 1;
    deliverToUsb = 1;
  }

  else if (rxd[1] == uid && rxd[3] == RESPONSE) {
    /* Store the application payload in the work array */
    rxd2work();
    rxdUpdated = 1;
    deliverToUsb = 1;
  }

/* Single-slot overwrite buffer:
 * Stores the latest packet marked for USB delivery.
 * Older packets are intentionally overwritten
 * if USB transmission is delayed.
 */
#ifdef ENABLE_PC_COMM
  if (deliverToUsb) {
    copyData(bridgeRx, rxd);
  }
#endif
}

void action(int buttonIndex) {

  switch (buttonIndex) {
    case 0:  // K1
      if (assignLock) return;
      startAssignAsHost();
      break;

    case 1:  // K2
      userData[0] = 0x00;
      optTX(0x02, REQUEST);
      break;

    case 2:  // K3
      userData[0] = randVal;
      optTX(0x02, OUT_CTRL);
      break;

    case 3:  // K4

      /* Role transition for functional evaluation
       * 
       * Note: Under these roles the consistency of
       * key inputs from K1 to K3 is not considered.
       */
      if (uid == REPEATER || uid == 0xFF) {
        toggleRole();
      }

      else {
        dest = 0x03;
        cmd = OUT_CTRL;
        userData[0] = randVal;
        txdClear();
        txd[0] = HEADER;
        txd[1] = dest;
        txd[2] = uid;
        txd[3] = cmd;
        txd[4] = userData[0];
        txd[DATA_LENGTH] = calcCRC8(txd, DATA_LENGTH);

        /* Payload corruption injection (10%)
         * to simulate CRC failure
         */
        if ((byte)random(0, 10) == 0) {
          txd[4] = randomGen();
        }
        OptLink.write(txd, PACKET_SIZE);
      }
      break;
  }
}
