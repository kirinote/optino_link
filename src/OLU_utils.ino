/*
 * Plastic Optical Fiber Shield
 *  designed for Arduino UNO R4
 *
 * Optino Link Unit Utility Functions
 * OLU_utils.ino
 *
 * Copyright (c) 2026 K. Yoshi
 * This software is released under the MIT License.
 * https://opensource.org/licenses/MIT
 */

/* GUI render cache
 * Stores last values drawn on TFT.
 * 'valid' flags start false to force the first render.
 */
typedef struct {
  bool uidValid;
  bool rxdValid;
  bool crcValid;
  bool usgValid;
  bool rndValid;
  byte uid;
  byte rxd;
  byte crc;
  byte usg;
  byte rnd;
} GuiCache;

static GuiCache guiCache = { 0 };

/* Electrical characteristic on A0-A3:
 * Active-high digital input.
 * The hardware debounce is required.
 */
void initPins() {
  pinMode(D0, INPUT);
  pinMode(D1, INPUT);
  pinMode(D2, INPUT);
  pinMode(D3, INPUT);
  pinMode(D4, OUTPUT);
  pinMode(D5, OUTPUT);
  pinMode(D6, OUTPUT);
  pinMode(D7, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(TFT_BL, OUTPUT);
}

void initDisplay() {
  analogWrite(TFT_BL, BACKLIGHT_DIM);
  tft.sendCommand(ST77XX_SWRESET);
  delay(150);

  /* ST7735 green tab variant requires column/row
   * offset correction in Adafruit_ST7735.cpp.  
   */
  tft.initR(INITR_GREENTAB);
  tft.setSPISpeed(24000000);
  tft.invertDisplay(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setRotation(3);
  tft.setTextWrap(0);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  analogWrite(TFT_BL, BACKLIGHT_MAX);
}

void renderLabels() {
  tft.setCursor(0, 0);
  tft.print("RXD:");
  tft.setCursor(0, 20);
  tft.print("CRE:");
  tft.setCursor(0, 40);
  tft.print("USG:");
  tft.setCursor(80, 0);
  tft.print("UID:");
  tft.setCursor(80, 20);
  tft.print("RND:");
}

/* Clear entire display.
 * This removes all labels. Call renderLabels() afterwards.
 */
void clearDisplay() {
  analogWrite(TFT_BL, BACKLIGHT_DIM);
  tft.fillScreen(ST77XX_BLACK);
  analogWrite(TFT_BL, BACKLIGHT_MAX);
}

/* Initialize CRC Lookup Table by copying from Program Memory to RAM
 *
 * Background:
 *   - CRC calculation is called frequently, so caching in RAM improves
 *     access speed (PROGMEM reads are slow on Arduino UNO R4)
 *   - CRC-8 lookup table is fixed at 256 bytes
 *
 * Important Notes:
 *   - Must be called once during setup()
 *   - Calling calcCRC8() without initCRCTable() will access
 *     uninitialized memory, resulting in unpredictable CRC values
 */
void initCRCTable() {
  memcpy_P(crc8_table_local, crc8_table, 256);
}

byte randomGen() {
  return (byte)random(0, 256);
}

/* Ring buffer for UART staging (polled context only)
 *
 * Design:
 *   - Used exclusively from main loop.
 *   - No interrupt safety.
 *   - No concurrency protection.
 *
 * Behavior:
 *   - Non-blocking.
 *   - Drops byte on overflow.
 *   - Overflow counter saturates at 0xFF.
 *
 * If RX is migrated to ISR in the future,
 * head/tail must be made atomic or protected.
 */
void storeRX(byte b) {
  uint16_t next = (rxHead + 1) & RX_BUF_MASK;
  if (next != rxTail) {
    rxBuf[rxHead] = b;
    rxHead = next;
    if (uid != REPEATER) usgUpdated = 1;
  } else {
    if (rxOVFlow < 0xFF) {
      rxOVFlow++;
      if (uid != REPEATER) usgUpdated = 1;
    }
  }
}

/* Called from main loop context only.
 * NOT safe for interrupt context.
 */
bool readRX(byte &b) {
  if (rxHead == rxTail) return 0;
  b = rxBuf[rxTail];
  rxTail = (rxTail + 1) & RX_BUF_MASK;
  return 1;
}

/* TX helper
 *
 * Payload source:
 * Uses global userData[] as payload buffer.
 * Caller must populate userData before invocation.
 *
 * Side effects:
 *   - Overwrites txd[]
 *   - Sends immediately via UART
 */
void optTX(byte dest, byte cmd) {
  txdClear();
  txd[0] = HEADER;
  txd[1] = dest;
  txd[2] = uid;
  txd[3] = cmd;
  for (byte i = 0; i < PAYLOAD_SIZE; i++) {
    txd[4 + i] = userData[i];
  }
  txd[DATA_LENGTH] = calcCRC8(txd, DATA_LENGTH);
  OptLink.write(txd, PACKET_SIZE);
}

/* Host-side ASSIGN initiation
 *
 * Behavior:
 *   - Enters ASSIGN state (locks process).
 *   - Sets this unit UID to 0x01 (host base).
 *   - Sends ASSIGN with payload = current UID.
 *
 * Protocol rule:
 *   Downstream units overwrite their UID with
 *   (received payload + 1) and forward the frame.
 *
 * Completion:
 *   Enumeration completes when the frame
 *   returns to this unit.
 *
 * Timeout supervision is handled in loop().
 */
void startAssignAsHost() {
  assignLock = 1;
  assignST = millis();
  newHost = 1;
  uid = 0x01;
  userData[0] = uid;
  optTX(DYN_UNITS, ASSIGN);
  uidUpdated = 1;
  rxdUpdated = 1;
  creUpdated = 1;
  usgUpdated = 1;
}

/* USB to Optical Ring bridge
 *
 * Behavior:
 *   - Waits for DATA_LENGTH bytes from USB.
 *   - Validates HEADER only (no CRC on USB side).
 *   - If ASSIGN and no active process:
 *       starts host-side enumeration.
 *   - Otherwise:
 *       regenerates CRC and forwards to ring.
 *
 * Design rule:
 *   USB link is trusted.
 *   CRC protection applies only to ring segment.
 *
 * Note:
 *   Input is fixed-length and blocking only
 *   when sufficient bytes are available.
 */
#ifdef ENABLE_PC_COMM
void handleUsbToRing() {
  if (Serial.available() < DATA_LENGTH) {
    return;
  }
  for (byte i = 0; i < DATA_LENGTH; i++) {
    bridgeTx[i] = Serial.read();
  }
  if (bridgeTx[0] != HEADER) {
    clearData(bridgeTx);
    return;
  }
  if (bridgeTx[3] == ASSIGN) {
    if (!assignLock) {
      startAssignAsHost();
    }
  } else {
    copyData(txd, bridgeTx);
    txd[DATA_LENGTH] = calcCRC8(txd, DATA_LENGTH);
    OptLink.write(txd, PACKET_SIZE);
  }
  clearData(bridgeTx);
}
#endif

/* Optical Ring to USB bridge
 *
 * Behavior:
 *   - Sends the latest deliverToUsb packet to USB.
 *   - Transfers DATA_LENGTH bytes (CRC excluded).
 *
 * Buffer policy:
 *   Single-slot overwrite buffer.
 *   Older packets may be lost if USB is busy.
 *
 * Assumption:
 *   USB link is reliable and point-to-point.
 */
#ifdef ENABLE_PC_COMM
void handleRingToUsb() {
  if (bridgeRx[0] == HEADER) {
    Serial.write(bridgeRx, DATA_LENGTH);
    clearData(bridgeRx);
  }
}
#endif

void resetGUICache() {
  guiCache.uidValid = 0;
  guiCache.rxdValid = 0;
  guiCache.crcValid = 0;
  guiCache.usgValid = 0;
  guiCache.rndValid = 0;
}

void initGUI() {
  resetGUICache();
  TFTupdUID();
  TFTupdRXData();
  TFTupdCRCErr();
  TFTupdUsage();
  TFTupdRNData();
}

/* GUI update flags:
 * Set by protocol handlers; cleared by updateGUI()
 * Heavy SPI activity may delay main loop UART processing
 */
void updateGUI() {
  if (uidUpdated) {
    TFTupdUID();
    uidUpdated = 0;
  }
  if (rxdUpdated) {
    TFTupdRXData();
    rxdUpdated = 0;
  }
  if (creUpdated) {
    TFTupdCRCErr();
    creUpdated = 0;
  }
  if (usgUpdated) {
    TFTupdUsage();
    usgUpdated = 0;
  }
  if (rndUpdated) {
    TFTupdRNData();
    rndUpdated = 0;
  }
}

void printHex2(byte value) {
  if (value < 0x10) tft.print('0');
  tft.print(value, HEX);
}

void printDec2(byte value) {
  if (value < 10) tft.print('0');
  tft.print(value);
}

void TFTupdUID() {
  if (guiCache.uidValid && guiCache.uid == uid) return;
  guiCache.uid = uid;
  guiCache.uidValid = 1;
  tft.setCursor(128, 0);
  if (uid == 0xFF) {
    tft.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
    printHex2(uid);
  } else if (uid >= 0x01 && uid <= UID_MAX) {
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    printHex2(uid);
  } else if (uid == 0x00) {
    tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    tft.print("RP");
  } else {
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
    printHex2(uid);
  }
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
}

void TFTupdRXData() {
  byte value = rxd[4];
  if (guiCache.rxdValid && guiCache.rxd == value) return;
  guiCache.rxd = value;
  guiCache.rxdValid = 1;
  tft.setCursor(48, 0);
  printHex2(value);
}

void TFTupdCRCErr() {
  if (guiCache.crcValid && guiCache.crc == crcErr) return;
  guiCache.crc = crcErr;
  guiCache.crcValid = 1;
  tft.setCursor(48, 20);
  printHex2(crcErr);
}

void TFTupdUsage() {
  byte current = getRXBufUsagePercent();
  if (guiCache.usgValid && guiCache.usg == current && rxOVFlow == 0) return;
  guiCache.usg = current;
  guiCache.usgValid = 1;
  tft.setCursor(48, 40);
  if (rxOVFlow > 0) {
    tft.print("OF");
  } else {
    printDec2(current);
  }
}

void TFTupdRNData() {
  if (guiCache.rndValid && guiCache.rnd == randVal) return;
  guiCache.rnd = randVal;
  guiCache.rndValid = 1;
  tft.setCursor(128, 20);
  printHex2(randVal);
}

void checkButton() {
  for (int i = 0; i < 4; i++) {
    bool currentState = digitalRead(buttonPins[i]);
    if (currentState == 1 && lastStates[i] == 0) {
      action(i);
    }
    lastStates[i] = currentState;
  }
}

/* Placeholder for future IN_CAPT command.
 * Intentionally left unimplemented to preserve
 * opcode compatibility.
 */
byte FourBitIn() {
  byte data = 0;
  for (int i = 0; i < 4; i++) {
    if (digitalRead(inputPins[i]) == HIGH) {
      bitSet(data, i);
    }
  }
  return data;
}

void FourBitOut(byte data) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(outputPins[i], bitRead(data, i));
  }
}

/* CRC-8-ATM
 * poly: 0x07
 * init: 0x00
 * refin: false
 * refout: false
 * xorout: 0x00
 * check("123456789") = 0xF4
 */
byte calcCRC8(const byte *data, byte len) {
  byte crc = 0x00;
  while (len--) {
    crc = crc8_table_local[crc ^ *data++];
  }
  return crc;
}

byte getRXBufUsagePercent() {
  uint16_t used = (rxHead - rxTail) & RX_BUF_MASK;
  return (used * 100UL) / RX_BUF_SIZE;
}

void rxd2txd() {
  copyPacket(txd, rxd);
}

void rxd2work() {
  copyPacket(work, rxd);
}

void rxdClear() {
  clearPacket(rxd);
}

void txdClear() {
  clearPacket(txd);
}

void workClear() {
  clearPacket(work);
}

/* Memory utility wrappers
 *
 * Rationale:
 *   - Centralizes memory operations.
 *   - Improves maintainability.
 *   - Allows future instrumentation (debug, tracing).
 */
void copyPacket(byte *dst, const byte *src) {
  memcpy(dst, src, PACKET_SIZE);
}

void copyData(byte *dst, const byte *src) {
  memcpy(dst, src, DATA_LENGTH);
}

void clearPacket(byte *buf) {
  memset(buf, 0, PACKET_SIZE);
}

void clearData(byte *buf) {
  memset(buf, 0, DATA_LENGTH);
}

void storeUID() {
  if (uid == DYN_UNITS) uid = 0xFF;
#ifdef USE_EXT_EEPROM
  extEEPROM_writeByte(EEPROM_UID_ADDR, uid);
#elif defined(PERSISTENT_UID)
  EEPROM.update(EEPROM_UID_ADDR, uid);
#endif
}

void restoreUID() {
  byte stored = 0xFF;
#ifdef USE_EXT_EEPROM
  stored = extEEPROM_readByte(EEPROM_UID_ADDR);
#elif defined(PERSISTENT_UID)
  stored = EEPROM.read(EEPROM_UID_ADDR);
#else
  uid = 0xFF;
  return;
#endif

  /* 0xFE is broadcast-only and must never be stored */
  if (stored == DYN_UNITS) {
    uid = 0xFF;
  } else {
    uid = stored;
  }
}

void toggleRole() {
  if (uid == 0xFF) {
    uid = REPEATER;
    storeUID();
    rxdClear();
    crcErr = 0;
    randVal = 0x00;
    uidUpdated = 1;
    creUpdated = 1;
    clearDisplay();
    renderLabels();
    TFTupdUID();
    TFTupdCRCErr();
  } else if (uid == REPEATER) {
    uid = 0xFF;
    storeUID();
    rxdClear();
    crcErr = 0;
    randVal = randomGen();
    uidUpdated = 1;
    rxdUpdated = 1;
    creUpdated = 1;
    usgUpdated = 1;
    rndUpdated = 1;
    clearDisplay();
    renderLabels();
    initGUI();
  }
}

#ifdef USE_EXT_EEPROM
byte extEEPROM_readByte(uint16_t addr) {
  Wire1.beginTransmission(EXT_EEPROM_ADDR);
  Wire1.write((addr >> 8) & 0xFF);
  Wire1.write(addr & 0xFF);
  Wire1.endTransmission(false);
  Wire1.requestFrom(EXT_EEPROM_ADDR, 1);
  if (Wire1.available())
    return Wire1.read();
  return 0xFF;
}

void extEEPROM_writeByte(uint16_t addr, byte data) {
  Wire1.beginTransmission(EXT_EEPROM_ADDR);
  Wire1.write((addr >> 8) & 0xFF);
  Wire1.write(addr & 0xFF);
  Wire1.write(data);
  Wire1.endTransmission();
  delay(6);
}
#endif

/* Reserved functions for future use
byte packData(byte foo, byte command) {
  return ((foo & 0x0F) << 4) | (command & 0x0F);
}

byte getFoo(byte packet) {
  return (packet >> 4) & 0x0F;
}

byte getCommand(byte packet) {
  return packet & 0x0F;
}
*/
