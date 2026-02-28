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
  analogWrite(TFT_BL, BACKLIGHT_MIN);
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

void clearDisplay() {
  analogWrite(TFT_BL, BACKLIGHT_MIN);
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

void initGUI() {
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

void TFTupdUID() {
  tft.setCursor(80, 0);
  tft.print("UID:");
  if (uid == 0xFF) {
    tft.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
    printHex2(uid);
  } else if (uid >= 0x01 && uid <= UID_MAX) {
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    printHex2(uid);
  } else if (uid == 0x00) {
    tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    tft.println("RP");
  } else {
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
    printHex2(uid);
  }
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
}

void printHex2(byte value) {
  if (value < 0x10) tft.print('0');
  tft.print(value, HEX);
}

void printDec2(byte value) {
  if (value < 10) tft.print('0');
  tft.print(value);
}

void TFTupdRXData() {
  tft.setCursor(0, 0);
  tft.print("RXD:");
  printHex2(rxd[4]);
}

void TFTupdCRCErr() {
  tft.setCursor(0, 20);
  tft.print("CRE:");
  printHex2(crcErr);
}

void TFTupdUsage() {
  tft.setCursor(0, 40);
  tft.print("USG:");
  usage = getRXBufUsage();
  if (rxOVFlow > 0) {
    tft.println("OF");
  } else {
    printDec2(usage);
  }
}

void TFTupdRNData() {
  tft.setCursor(80, 20);
  tft.print("RND:");
  if (randVal < 0x10) tft.print('0');
  tft.println(randVal, HEX);
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

byte getRXBufUsage() {
  uint16_t used = (rxHead - rxTail) & RX_BUF_MASK;
  return (used * 100UL) / RX_BUF_SIZE;
}

void rxd2txd() {
  memcpy(txd, rxd, PACKET_SIZE);
}

void rxd2work() {
  memcpy(work, rxd, PACKET_SIZE);
}

void rxdClear() {
  memset(rxd, 0, sizeof(rxd));
}

void txdClear() {
  memset(txd, 0, sizeof(txd));
}

void workClear() {
  memset(work, 0, sizeof(work));
}

void storeUID() {
#ifndef DEMO_MODE
  EEPROM.update(0, uid);
#endif
}

void restoreUID() {
#ifndef DEMO_MODE
  uid = EEPROM.read(0);
#else

  /* In demo mode, the UID is set to
   * unassigned on boot.
   */
  uid = 0xFF;
#endif
}

void toggleRole() {
  if (uid == 0xFF) {
    uid = REPEATER;
    storeUID();
    rxdClear();
    crcErr = 0;
    usage = 0;
    randVal = 0x00;
    uidUpdated = 1;
    creUpdated = 1;
    clearDisplay();
    TFTupdUID();
    TFTupdCRCErr();
  } else if (uid == REPEATER) {
    uid = 0xFF;
    storeUID();
    rxdClear();
    crcErr = 0;
    usage = 0;
    randVal = randomGen();
    uidUpdated = 1;
    rxdUpdated = 1;
    creUpdated = 1;
    usgUpdated = 1;
    rndUpdated = 1;
    clearDisplay();
    initGUI();
  }
}

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
