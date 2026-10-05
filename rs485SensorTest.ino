// rs485SensorTest - bench test for RS485 Modbus-RTU sensors on the ChinampaSensorBoard
// (HardwareDesign/ChinampaSensorBoard), the only board with RS485 until Paula2 is built.
// Readings go to the USB serial port (115200) and to the six 4-digit displays.
//
// Sensors (each has its own address and baud rate; the UART is switched per transaction, so they can
// share the bus even at different speeds):
//
//  soil - 7-in-1 soil probe, "Soil Multi-parameter Sensor Manual V2.2" (~/Downloads/SoilRS485.pdf).
//         Default address 2, 9600 8N1, power 3.3-24V. Function 0x03 on 0x0000-0x0007:
//         0 temperature x10 (signed)  1 moisture x10 %  2 EC uS/cm  3 salt  4 N  5 P  6 K mg/kg  7 pH x10
//         0x0080 = address, written with function 0x10. Same code as querySoilSensor() in Paula2.ino.
//
//  do   - Renke RS-LDOS-N01-2-20 optical dissolved oxygen probe (salt-water version). Register map from
//         "RS-LDO-N01-1 Fluorescence Dissolved Oxygen Transmitter User Manual V2.0" (same Renke family).
//         Default address 1, 4800 8N1, power 10-30V (NOT from the board's 5V terminal).
//         Function 0x03, big-endian IEEE floats: 0x0000 saturation, 0x0002 mg/L, 0x0004 temperature C.
//         0x1020 salinity (ppt, uint16, default 0 on the fresh-water model), 0x1022 air pressure kPa x100,
//         0x1010 calibration, 0x07D0 address, 0x07D1 baud code. Writes use function 0x06.
//         The manual's example returns saturation as a fraction (0.9177 = 91.8%), so it is shown x100;
//         the raw float is printed too so that can be checked.
//
// Sensor cable colours (also printed by the "wiring" command):
//   DO probe (Renke manual section 2.1):  brown = V+ (10-30V)   black = V- (GND)
//                                         yellow (or green) = RS485 A   blue = RS485 B
//   Soil probe (from the cable):          red = V+ (3.3-24V)    black = V- (GND)
//                                         yellow = RS485 A      green = RS485 B
//   Careful with green: it is B on the soil probe but A on DO cables that have no yellow wire.
//   Both sensors' A wires go to J2 pin 1 and both B wires to J2 pin 2. Power both from the 12V supply
//   that feeds J1, with their black wires on the same supply negative as the board (common ground).
//
// ChinampaSensorBoard wiring (from a fresh netlist export of the KiCad 8 schematic):
//   MAX485E U1: RO -> GPIO16 (through R28/R29 divider), DI <- GPIO17, DE + /RE <- GPIO26
//   J2 screw terminal: pin 1 = A, pin 2 = B.  J14 jumper = 120 ohm termination (fit it for long cables)
//   J1 = board power input (pin 1 +, pin 2 GND), feeds the TPS5430 buck. J7 / J12 = 5V out (pin 1), GND (pin 2).
//   I2C: GPIO21/22, through the BSS138 level shifter to the 5V side.
//   HT16K33 0x70 (IC5) -> DS3 (COM0-3), DS4 (COM4-7)
//   HT16K33 0x71 (IC1) -> DS7 (COM0-3), DS8 (COM4-7)
//   HT16K33 0x72 (IC6) -> DS1 (COM0-3), DS2 (COM4-7)
//   Each display: ROW0-7 = segments a,b,c,d,e,f,g,dp; COM0 = leftmost digit.
//
// Displays: each enabled sensor gets a page; pages rotate every PAGE_MS, starting with the sensor's
// name ("SoIL" / "do") on all six displays for a moment. On a failed read the page shows "Err" + code.
//
// Serial commands (115200, newline terminated). <s> is a sensor name: soil, do
//   read [<s>]           read now (all enabled sensors, or just one)
//   run / stop           start / stop reading every READ_PERIOD_MS
//   <s> on | off         include / skip the sensor in polling and display pages
//   <s> addr <n>         address to query (0 = broadcast, only with a single sensor on the bus)
//   <s> baud <b>         baud rate the ESP32 uses for this sensor
//   <s> find             ask a lone sensor its address, trying every baud rate
//   <s> setaddr <n>      write a new address into the sensor
//   do salinity <ppt>    write the DO probe's salinity compensation (0 for fresh water)
//   do pressure <kPa>    write the DO probe's air pressure (default 101.33)
//   raw on | off         print the Modbus frames
//   wiring               print the cable colours and where they go
//   displays            show each display's silkscreen name (dS1..dS8) for 3 seconds
//   help

#include <Wire.h>

#define RS485_RX 16
#define RS485_TX 17
#define RS485_DERE 26
#define I2C_SDA 21
#define I2C_SCL 22

#define REPLY_TIMEOUT_MS 1000
#define READ_PERIOD_MS 3000
#define PAGE_MS 6000
#define PAGE_LABEL_MS 1000
#define NPK_CYCLE_MS 2000
#define DISPLAY_BRIGHTNESS 8  // 0-15

HardwareSerial &rs485 = Serial2;
uint32_t currentBaud = 0;
bool printFrames = true;
bool running = true;

// ---------------------------------------------------------------- sensors
struct SoilReading {
  float temperature;
  float moisture;
  uint16_t ec;
  uint16_t salt;
  uint16_t nitrogen;
  uint16_t phosphorus;
  uint16_t potassium;
  float ph;
};

struct DoReading {
  float saturationRaw;  // as returned by the probe
  float saturation;     // %
  float mgL;
  float temperature;
  int salinity;      // ppt, -1 if the register read failed
  float pressureKpa; // -1 if the register read failed
};

#define SENSOR_SOIL 0
#define SENSOR_DO 1
#define SENSOR_COUNT 2

struct Rs485Sensor {
  const char *key;        // serial command name
  const char *pageLabel;  // shown on the displays when its page comes up
  uint8_t address;
  uint32_t baud;
  bool enabled;
  uint16_t addressRegister;  // for find / setaddr
  bool writeWithFc10;        // soil manual uses 0x10 for writes, Renke uses 0x06
  const uint8_t *findAddresses;  // broadcast addresses tried by find
  uint8_t findAddressCount;
  int lastError;  // 0 = ok
  bool haveReading;
};

const uint8_t SOIL_FIND[] = { 0x00 };
const uint8_t DO_FIND[] = { 0xFF, 0x00 };  // Renke tools query 0xFF; 0x00 as a fallback

Rs485Sensor sensors[SENSOR_COUNT] = {
  { "soil", "SoIL", 2, 9600, true, 0x0080, true, SOIL_FIND, sizeof(SOIL_FIND), 0, false },
  { "do", "do", 1, 4800, true, 0x07D0, false, DO_FIND, sizeof(DO_FIND), 0, false },
};

SoilReading soil;
DoReading dox;

// ---------------------------------------------------------------- HT16K33 displays
// Display slots in the order readings are assigned to them. Change the order here if the physical
// layout of the board reads better another way (send "displays" to see which is which).
struct DisplaySlot {
  uint8_t i2cAddress;
  uint8_t firstCom;  // 0 = COM0-3, 4 = COM4-7
  const char *name;
};
const DisplaySlot slots[6] = {
  { 0x72, 0, "DS1" }, { 0x72, 4, "DS2" }, { 0x70, 0, "DS3" },
  { 0x70, 4, "DS4" }, { 0x71, 0, "DS7" }, { 0x71, 4, "DS8" },
};

const uint8_t chipAddresses[3] = { 0x70, 0x71, 0x72 };
bool chipPresent[3] = { false, false, false };
uint8_t displayRam[3][8];  // [chip][com] segment byte

const uint8_t DIGITS[10] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F };

uint8_t segmentsFor(char c) {
  if (c >= '0' && c <= '9') return DIGITS[c - '0'];
  switch (c) {
    case 'A': case 'a': return 0x77;
    case 'B': case 'b': return 0x7C;
    case 'C': return 0x39;
    case 'c': return 0x58;
    case 'D': case 'd': return 0x5E;
    case 'E': case 'e': return 0x79;
    case 'F': case 'f': return 0x71;
    case 'H': return 0x76;
    case 'h': return 0x74;
    case 'I': case 'i': return 0x06;
    case 'K': case 'k': return 0x75;  // closest a 7-segment gets
    case 'L': case 'l': return 0x38;
    case 'N': case 'n': return 0x54;
    case 'O': return 0x3F;
    case 'o': return 0x5C;
    case 'P': case 'p': return 0x73;
    case 'R': case 'r': return 0x50;
    case 'S': case 's': return 0x6D;
    case 'T': case 't': return 0x78;
    case 'U': return 0x3E;
    case 'u': return 0x1C;
    case 'Y': case 'y': return 0x6E;
    case '-': return 0x40;
    case '_': return 0x08;
    default: return 0x00;
  }
}

int chipIndex(uint8_t address) {
  for (int i = 0; i < 3; i++)
    if (chipAddresses[i] == address) return i;
  return -1;
}

bool ht16k33Command(uint8_t address, uint8_t cmd) {
  Wire.beginTransmission(address);
  Wire.write(cmd);
  return Wire.endTransmission() == 0;
}

void initDisplays() {
  for (int i = 0; i < 3; i++) {
    uint8_t a = chipAddresses[i];
    chipPresent[i] = ht16k33Command(a, 0x21);  // system oscillator on
    if (chipPresent[i]) {
      ht16k33Command(a, 0xA0);                       // ROW/INT pin as ROW output
      ht16k33Command(a, 0xE0 | DISPLAY_BRIGHTNESS);  // dimming
      ht16k33Command(a, 0x81);                       // display on, no blink
    }
    Serial.printf("HT16K33 0x%02X: %s\n", a, chipPresent[i] ? "found" : "NOT FOUND");
    memset(displayRam[i], 0, sizeof(displayRam[i]));
  }
}

void flushDisplays() {
  for (int i = 0; i < 3; i++) {
    if (!chipPresent[i]) continue;
    Wire.beginTransmission(chipAddresses[i]);
    Wire.write(0x00);  // display RAM start
    for (int com = 0; com < 8; com++) {
      Wire.write(displayRam[i][com]);  // ROW0-7
      Wire.write(0x00);                // ROW8-15, unused
    }
    Wire.endTransmission();
  }
}

// Right-aligns text on a 4-digit display. A '.' lights the decimal point of the character before it.
void showText(int slot, const char *text) {
  uint8_t seg[8];
  int n = 0;
  for (const char *p = text; *p && n < 8; p++) {
    if (*p == '.' && n > 0 && !(seg[n - 1] & 0x80)) seg[n - 1] |= 0x80;
    else if (*p == '.') seg[n++] = 0x80;
    else seg[n++] = segmentsFor(*p);
  }
  int chip = chipIndex(slots[slot].i2cAddress);
  if (chip < 0) return;
  uint8_t *ram = &displayRam[chip][slots[slot].firstCom];
  for (int d = 0; d < 4; d++) {
    int src = n - 4 + d;  // keep the rightmost 4 characters
    ram[d] = src >= 0 ? seg[src] : 0x00;
  }
}

void showAll(const char *text) {
  for (int s = 0; s < 6; s++) showText(s, text);
}

void showFloat1(int slot, float v) {
  char buf[12];
  if (v > 999.9f || v < -99.9f) snprintf(buf, sizeof(buf), "%s", v > 0 ? "HI" : "LO");
  else snprintf(buf, sizeof(buf), "%.1f", v);
  showText(slot, buf);
}

void showFloat2(int slot, float v) {
  if (v > 99.99f || v < -9.99f) {
    showFloat1(slot, v);
    return;
  }
  char buf[12];
  snprintf(buf, sizeof(buf), "%.2f", v);
  showText(slot, buf);
}

void showInt(int slot, long v) {
  char buf[12];
  if (v > 9999) snprintf(buf, sizeof(buf), "HI");
  else snprintf(buf, sizeof(buf), "%ld", v);
  showText(slot, buf);
}

// Letter prefix while the value fits in 3 digits, otherwise the bare number.
void showLabelled(int slot, char label, int v) {
  char buf[12];
  if (v >= 0 && v <= 999) snprintf(buf, sizeof(buf), "%c%3d", label, v);
  else snprintf(buf, sizeof(buf), "%d", v);
  showText(slot, buf);
}

void showDisplayNames() {
  for (int s = 0; s < 6; s++) {
    char buf[8];
    snprintf(buf, sizeof(buf), "d%s", slots[s].name + 1);  // "DS3" -> "dS3"
    showText(s, buf);
  }
  flushDisplays();
}

// Page layouts. Slot numbers are indexes into slots[] (0 = DS1 ... 5 = DS8).
uint8_t npkIndex = 0;

void showSoilPage() {
  showFloat1(0, soil.temperature);  // DS1 temperature C
  showFloat1(1, soil.moisture);     // DS2 moisture %
  showInt(2, soil.ec);              // DS3 EC uS/cm
  showFloat1(3, soil.ph);           // DS4 pH
  if (npkIndex == 0) showLabelled(4, 'n', soil.nitrogen);  // DS7 N / P / K in turn
  else if (npkIndex == 1) showLabelled(4, 'P', soil.phosphorus);
  else showLabelled(4, 'k', soil.potassium);
  showInt(5, soil.salt);  // DS8 salt
}

void showDoPage() {
  showFloat2(0, dox.mgL);          // DS1 mg/L
  showFloat1(1, dox.saturation);   // DS2 saturation %
  showFloat1(2, dox.temperature);  // DS3 water temperature C
  if (dox.salinity >= 0) showLabelled(3, 'S', dox.salinity);  // DS4 salinity setting, ppt
  else showText(3, "S --");
  showText(4, "");
  showText(5, "do");
}

int currentPage = -1;  // sensor index being shown
unsigned long pageStart = 0;

void updateDisplays() {
  unsigned long now = millis();
  if (currentPage < 0) {
    showAll("----");
  } else if (now - pageStart < PAGE_LABEL_MS) {
    showAll(sensors[currentPage].pageLabel);
  } else {
    Rs485Sensor &s = sensors[currentPage];
    if (s.lastError != 0) {
      showAll("");
      showText(0, "Err");
      showInt(1, s.lastError);
      showText(5, s.pageLabel);
    } else if (!s.haveReading) {
      showAll("----");
      showText(5, s.pageLabel);
    } else if (currentPage == SENSOR_SOIL) {
      showSoilPage();
    } else if (currentPage == SENSOR_DO) {
      showDoPage();
    }
  }
  flushDisplays();
}

void nextPage() {
  for (int i = 1; i <= SENSOR_COUNT; i++) {
    int candidate = (currentPage + i + SENSOR_COUNT) % SENSOR_COUNT;
    if (sensors[candidate].enabled) {
      currentPage = candidate;
      pageStart = millis();
      return;
    }
  }
  currentPage = -1;
}

// ---------------------------------------------------------------- Modbus RTU
// Error codes, also shown as "Err <n>" on the displays
#define ERR_NO_REPLY 1
#define ERR_SHORT 2
#define ERR_CRC 3
#define ERR_EXCEPTION 4
#define ERR_WRONG_ADDRESS 5
#define ERR_BAD_HEADER 6

uint16_t modbusCrc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}

void printHex(const char *label, const uint8_t *data, size_t len) {
  if (!printFrames) return;
  Serial.print(label);
  for (size_t i = 0; i < len; i++) Serial.printf("%02X ", data[i]);
  Serial.println();
}

void setBaud(uint32_t baud) {
  if (baud == currentBaud) return;
  rs485.end();
  rs485.begin(baud, SERIAL_8N1, RS485_RX, RS485_TX);
  currentBaud = baud;
  delay(5);
}

// Sends req (CRC appended here, so reqLen excludes it) and collects up to expectedLen reply bytes.
// Returns 0 or an ERR_ code; errorText explains it.
int modbusTransaction(uint32_t baud, uint8_t *req, size_t reqLen, uint8_t *resp, size_t expectedLen, size_t &respLen, String &errorText) {
  setBaud(baud);
  uint16_t crc = modbusCrc16(req, reqLen);
  req[reqLen] = crc & 0xFF;  // CRC low byte first
  req[reqLen + 1] = crc >> 8;

  delay(10);  // Modbus inter-frame gap (>= 3.5 characters, ~8 ms at 4800)
  while (rs485.available()) rs485.read();
  digitalWrite(RS485_DERE, HIGH);  // transmit
  rs485.write(req, reqLen + 2);
  rs485.flush();                   // wait for the last stop bit to leave the shift register
  digitalWrite(RS485_DERE, LOW);   // receive before the sensor answers
  printHex("  tx: ", req, reqLen + 2);

  respLen = 0;
  unsigned long start = millis();
  unsigned long lastByte = start;
  while (respLen < expectedLen && millis() - start < REPLY_TIMEOUT_MS) {
    if (rs485.available()) {
      resp[respLen++] = rs485.read();
      lastByte = millis();
    } else if (respLen > 0 && millis() - lastByte > 100) {
      break;  // went quiet mid-frame: exception or short reply
    }
  }
  if (respLen == 0) {
    errorText = "no reply within " + String(REPLY_TIMEOUT_MS) + " ms (check A/B - swap them if unsure -, sensor power, address and baud rate)";
    return ERR_NO_REPLY;
  }
  printHex("  rx: ", resp, respLen);

  if (respLen >= 5 && (resp[1] & 0x80) && modbusCrc16(resp, 3) == (uint16_t)(resp[3] | (resp[4] << 8))) {
    errorText = "Modbus exception code " + String(resp[2]);
    return ERR_EXCEPTION;
  }
  if (respLen < expectedLen) {
    errorText = "short reply (" + String(respLen) + " of " + String(expectedLen) + " bytes)";
    return ERR_SHORT;
  }
  if (modbusCrc16(resp, respLen - 2) != (uint16_t)(resp[respLen - 2] | (resp[respLen - 1] << 8))) {
    errorText = "bad CRC (noise, wrong baud rate, or A/B swapped)";
    return ERR_CRC;
  }
  if (req[0] != 0 && req[0] != 0xFF && resp[0] != req[0]) {
    errorText = "reply came from address " + String(resp[0]) + ", not " + String(req[0]);
    return ERR_WRONG_ADDRESS;
  }
  if (resp[1] != req[1]) {
    errorText = "unexpected function code " + String(resp[1]);
    return ERR_BAD_HEADER;
  }
  return 0;
}

// Function 0x03. regs receives count values (count <= 16). Returns 0 or an ERR_ code.
int readRegisters(uint32_t baud, uint8_t address, uint16_t first, uint8_t count, uint16_t *regs, uint8_t &replyAddress, String &errorText) {
  uint8_t req[8] = { address, 0x03, (uint8_t)(first >> 8), (uint8_t)(first & 0xFF), 0x00, count };
  uint8_t resp[5 + 2 * 16];
  size_t len;
  int err = modbusTransaction(baud, req, 6, resp, 5 + 2 * count, len, errorText);
  if (err) return err;
  if (resp[2] != 2 * count) {
    errorText = "unexpected byte count " + String(resp[2]);
    return ERR_BAD_HEADER;
  }
  for (int i = 0; i < count; i++) regs[i] = (resp[3 + 2 * i] << 8) | resp[4 + 2 * i];
  replyAddress = resp[0];
  return 0;
}

// Writes one register, with function 0x10 (soil manual's example) or 0x06 (Renke).
int writeRegister(uint32_t baud, uint8_t address, uint16_t reg, uint16_t value, bool useFc10, String &errorText) {
  uint8_t resp[8];
  size_t len;
  if (useFc10) {
    uint8_t req[11] = { address, 0x10, (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), 0x00, 0x01, 0x02,
                        (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    return modbusTransaction(baud, req, 9, resp, sizeof(resp), len, errorText);
  }
  uint8_t req[8] = { address, 0x06, (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
  return modbusTransaction(baud, req, 6, resp, sizeof(resp), len, errorText);
}

float floatFromRegisters(uint16_t high, uint16_t low) {
  uint32_t bits = ((uint32_t)high << 16) | low;  // big endian: first register holds the high word
  float f;
  memcpy(&f, &bits, sizeof(f));
  return f;
}

// ---------------------------------------------------------------- sensor reads
void readSoil() {
  Rs485Sensor &s = sensors[SENSOR_SOIL];
  uint16_t regs[8];
  uint8_t from;
  String error;
  Serial.printf("--- soil: address %u at %lu baud\n", s.address, (unsigned long)s.baud);
  s.lastError = readRegisters(s.baud, s.address, 0x0000, 8, regs, from, error);
  if (s.lastError) {
    Serial.printf("Failure-ReadSoilSensor-Err %d: %s\n", s.lastError, error.c_str());
    return;
  }
  soil.temperature = (int16_t)regs[0] / 10.0f;  // two's complement below 0C
  soil.moisture = regs[1] / 10.0f;
  soil.ec = regs[2];
  soil.salt = regs[3];  // manual gives no unit
  soil.nitrogen = regs[4];
  soil.phosphorus = regs[5];
  soil.potassium = regs[6];
  soil.ph = regs[7] / 10.0f;
  s.haveReading = true;

  Serial.printf("address=%u\n", from);
  Serial.printf("temperature=%.1f C   [DS1]\n", soil.temperature);
  Serial.printf("moisture=%.1f %%     [DS2]\n", soil.moisture);
  Serial.printf("ec=%u uS/cm         [DS3]\n", soil.ec);
  Serial.printf("ph=%.1f             [DS4]\n", soil.ph);
  Serial.printf("nitrogen=%u mg/kg   [DS7, n]\n", soil.nitrogen);
  Serial.printf("phosphorus=%u mg/kg [DS7, P]\n", soil.phosphorus);
  Serial.printf("potassium=%u mg/kg  [DS7, k]\n", soil.potassium);
  Serial.printf("salt=%u             [DS8]\n", soil.salt);
  Serial.println("Ok-ReadSoilSensor");
}

void readDo() {
  Rs485Sensor &s = sensors[SENSOR_DO];
  uint16_t regs[6];
  uint8_t from;
  String error;
  Serial.printf("--- do: address %u at %lu baud\n", s.address, (unsigned long)s.baud);
  s.lastError = readRegisters(s.baud, s.address, 0x0000, 6, regs, from, error);
  if (s.lastError) {
    Serial.printf("Failure-ReadDoSensor-Err %d: %s\n", s.lastError, error.c_str());
    return;
  }
  dox.saturationRaw = floatFromRegisters(regs[0], regs[1]);
  dox.saturation = dox.saturationRaw * 100.0f;
  dox.mgL = floatFromRegisters(regs[2], regs[3]);
  dox.temperature = floatFromRegisters(regs[4], regs[5]);
  s.haveReading = true;

  // Compensation settings - informative, so a failure here doesn't fail the reading
  uint16_t v;
  dox.salinity = readRegisters(s.baud, s.address, 0x1020, 1, &v, from, error) == 0 ? v : -1;
  dox.pressureKpa = readRegisters(s.baud, s.address, 0x1022, 1, &v, from, error) == 0 ? v / 100.0f : -1;

  Serial.printf("address=%u\n", s.address);
  Serial.printf("do=%.2f mg/L        [DS1]\n", dox.mgL);
  Serial.printf("saturation=%.1f %%  [DS2]  (raw float %.4f)\n", dox.saturation, dox.saturationRaw);
  Serial.printf("temperature=%.1f C  [DS3]\n", dox.temperature);
  if (dox.salinity >= 0) Serial.printf("salinity=%d ppt     [DS4]\n", dox.salinity);
  else Serial.println("salinity=? (register 0x1020 not readable)");
  if (dox.pressureKpa >= 0) Serial.printf("pressure=%.2f kPa\n", dox.pressureKpa);
  else Serial.println("pressure=? (register 0x1022 not readable)");
  Serial.println("Ok-ReadDoSensor");
}

void readSensor(int i) {
  if (i == SENSOR_SOIL) readSoil();
  else if (i == SENSOR_DO) readDo();
}

// Broadcast read of the address register at every supported baud rate. Only valid with one sensor
// on the bus.
void findSensor(int i) {
  Rs485Sensor &s = sensors[i];
  const uint32_t bauds[] = { 4800, 9600, 2400, 19200, 38400, 1200, 57600, 115200 };
  showAll("FInd");
  flushDisplays();
  for (uint32_t b : bauds) {
    for (int a = 0; a < s.findAddressCount; a++) {
      Serial.printf("--- %s: query address %u at %lu baud\n", s.key, s.findAddresses[a], (unsigned long)b);
      uint16_t reg;
      uint8_t from;
      String error;
      int err = readRegisters(b, s.findAddresses[a], s.addressRegister, 1, &reg, from, error);
      if (!err) {
        s.address = reg;
        s.baud = b;
        Serial.printf("Found %s sensor: address %u (replied as %u) at %lu baud - now using those\n", s.key, reg, from, (unsigned long)b);
        return;
      }
      Serial.printf("  %s\n", error.c_str());
    }
  }
  Serial.printf("No %s sensor answered\n", s.key);
}

// ---------------------------------------------------------------- serial commands
void printWiring() {
  Serial.println("Wiring (both sensors in parallel on J2, both powered from the 12V supply that feeds J1):");
  Serial.println("  Signal    DO probe (Renke)   Soil probe   Goes to");
  Serial.println("  V+        brown (10-30V)     red          12V supply +");
  Serial.println("  GND       black              black        12V supply - (same as J1 pin 2)");
  Serial.println("  RS485 A   yellow (or green)  yellow       J2 pin 1 (A)");
  Serial.println("  RS485 B   blue               green        J2 pin 2 (B)");
  Serial.println("  Careful: green is B on the soil probe but A on DO cables that have no yellow wire.");
  Serial.println("  No reply (Err 1)? Swap A and B first. J14 jumper = 120 ohm termination, only for long cables.");
}

void printHelp() {
  Serial.println("Commands: read [soil|do] | run | stop | <s> on|off | <s> addr <n> | <s> baud <b> | <s> find |");
  Serial.println("          <s> setaddr <n> | do salinity <ppt> | do pressure <kPa> | raw on|off | wiring | displays | help");
  for (int i = 0; i < SENSOR_COUNT; i++)
    Serial.printf("  %-4s address %u, %lu baud, %s\n", sensors[i].key, sensors[i].address, (unsigned long)sensors[i].baud,
                  sensors[i].enabled ? "on" : "off");
}

int sensorByKey(const String &key) {
  for (int i = 0; i < SENSOR_COUNT; i++)
    if (key == sensors[i].key) return i;
  return -1;
}

bool validBaud(long b) {
  return b == 1200 || b == 2400 || b == 4800 || b == 9600 || b == 19200 || b == 38400 || b == 57600 || b == 115200;
}

void handleSensorCommand(int i, String sub, String arg) {
  Rs485Sensor &s = sensors[i];
  if (sub == "on" || sub == "off") {
    s.enabled = sub == "on";
    Serial.printf("%s %s\n", s.key, s.enabled ? "on" : "off");
    if (!s.enabled && currentPage == i) nextPage();
    if (s.enabled && currentPage < 0) nextPage();
  } else if (sub == "read") {
    readSensor(i);
  } else if (sub == "addr") {
    int a = arg.toInt();
    if (arg.length() == 0 || a < 0 || a > 255) Serial.println("addr must be 0-255");
    else {
      s.address = a;
      Serial.printf("%s: querying address %u\n", s.key, s.address);
    }
  } else if (sub == "baud") {
    long b = arg.toInt();
    if (!validBaud(b)) Serial.println("baud must be 1200, 2400, 4800, 9600, 19200, 38400, 57600 or 115200");
    else {
      s.baud = b;
      Serial.printf("%s: ESP32 talks at %ld baud\n", s.key, b);
    }
  } else if (sub == "find") {
    findSensor(i);
  } else if (sub == "setaddr") {
    int a = arg.toInt();
    if (arg.length() == 0 || a < 1 || a > 247) {
      Serial.println("setaddr must be 1-247");
      return;
    }
    String error;
    int err = writeRegister(s.baud, s.address, s.addressRegister, a, s.writeWithFc10, error);
    if (err) Serial.printf("Failure-SetAddress-Err %d: %s\n", err, error.c_str());
    else {
      Serial.printf("Ok-SetAddress-%s sensor %u is now %u\n", s.key, s.address, a);
      s.address = a;
    }
  } else if (i == SENSOR_DO && (sub == "salinity" || sub == "pressure")) {
    float v = arg.toFloat();
    uint16_t reg = sub == "salinity" ? 0x1020 : 0x1022;
    uint16_t value = sub == "salinity" ? (uint16_t)v : (uint16_t)(v * 100.0f + 0.5f);
    if (arg.length() == 0 || (sub == "salinity" && (v < 0 || v > 60)) || (sub == "pressure" && (v < 50 || v > 120))) {
      Serial.println(sub == "salinity" ? "salinity must be 0-60 ppt" : "pressure must be 50-120 kPa");
      return;
    }
    String error;
    int err = writeRegister(s.baud, s.address, reg, value, false, error);
    if (err) Serial.printf("Failure-Set-%s-Err %d: %s\n", sub.c_str(), err, error.c_str());
    else Serial.printf("Ok-Set-%s-%s\n", sub.c_str(), arg.c_str());
  } else {
    printHelp();
  }
}

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  String words[3];
  int n = 0;
  while (line.length() > 0 && n < 3) {
    int sp = line.indexOf(' ');
    words[n++] = sp < 0 ? line : line.substring(0, sp);
    line = sp < 0 ? "" : line.substring(sp + 1);
    line.trim();
  }
  words[0].toLowerCase();
  words[1].toLowerCase();

  int sensor = sensorByKey(words[0]);
  if (sensor >= 0) {
    handleSensorCommand(sensor, words[1], words[2]);
  } else if (words[0] == "read") {
    int one = sensorByKey(words[1]);
    for (int i = 0; i < SENSOR_COUNT; i++)
      if (one == i || (one < 0 && sensors[i].enabled)) readSensor(i);
  } else if (words[0] == "run") {
    running = true;
    Serial.println("Reading every " + String(READ_PERIOD_MS) + " ms");
  } else if (words[0] == "stop") {
    running = false;
    Serial.println("Stopped");
  } else if (words[0] == "raw") {
    printFrames = words[1] != "off";
    Serial.printf("Modbus frames %s\n", printFrames ? "shown" : "hidden");
  } else if (words[0] == "wiring") {
    printWiring();
  } else if (words[0] == "displays") {
    showDisplayNames();
    delay(3000);
  } else {
    printHelp();
  }
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("rs485SensorTest - ChinampaSensorBoard");

  pinMode(RS485_DERE, OUTPUT);
  digitalWrite(RS485_DERE, LOW);  // receive

  Wire.begin(I2C_SDA, I2C_SCL);
  initDisplays();

  // Segment test, then each display's silkscreen name so the slot order can be checked
  showAll("8.8.8.8.");
  flushDisplays();
  delay(1500);
  showDisplayNames();
  delay(2500);

  printWiring();
  printHelp();
  nextPage();
}

void loop() {
  static String line = "";
  static unsigned long lastRead = 0;
  static unsigned long lastNpkSwitch = 0;
  static unsigned long lastDisplay = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      handleCommand(line);
      line = "";
    } else {
      line += c;
    }
  }

  unsigned long now = millis();
  if (running && now - lastRead >= READ_PERIOD_MS) {
    lastRead = now;
    for (int i = 0; i < SENSOR_COUNT; i++)
      if (sensors[i].enabled) readSensor(i);
  }
  if (now - pageStart >= PAGE_MS) nextPage();
  if (now - lastNpkSwitch >= NPK_CYCLE_MS) {
    lastNpkSwitch = now;
    npkIndex = (npkIndex + 1) % 3;
  }
  if (now - lastDisplay >= 200) {
    lastDisplay = now;
    updateDisplays();
  }
}
