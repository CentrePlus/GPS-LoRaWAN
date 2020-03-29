
// Least Significant Bit First
static const u1_t PROGMEM DEVEUI[8] = { 0xE7, 0xE1, 0x83, 0xD2, 0x59, 0xE1, 0xD5, 0x00 };
// Least Significant Bit First
static const u1_t PROGMEM APPEUI[8] = { 0x1F, 0xD0, 0x02, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
// Most Significant Bit First
static const u1_t PROGMEM APPKEY[16] = { 0xAC, 0xE6, 0x03, 0xEB, 0x84, 0xFD, 0x2C, 0xBA, 0xA5, 0x94, 0x5E, 0x37, 0x88, 0xB9, 0x19, 0x31 };

// TesterLoRa\test-920

void os_getArtEui(u1_t* buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui(u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey(u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, LMIC_UNUSED_PIN},
};
