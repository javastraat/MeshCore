#pragma once

// Small drivers for the chips on the M5Stack ENV units, which the stock
// sensor libraries in EnvironmentSensorManager do not cover:
//   ENV I   = DHT12  (0x5C, temp + humidity)  + BMP280 (already supported)
//   ENV II  = SHT30  (0x44, temp + humidity)  + BMP280 (already supported)
//   ENV III = SHT30  (0x44, temp + humidity)  + QMP6988 (0x70, temp + pressure)
//
// Every begin() checks the chip's identity (SHT30 status register with CRC,
// QMP6988 chip ID, DHT12 checksum), because these addresses are shared with
// SHT4x/INA226 (0x44), SHTC3 (0x70) and LPS22HB (0x5C).
//
// All register reads end the address write with a full STOP before reading:
// the nRF52's TWIM peripheral can hang on a repeated start with these chips.
// The QMP6988 soft reset is skipped for the same reason (the chip holds SDA
// low during it).
//
// QMP6988 compensation follows m5stack/M5Unit-ENV (MIT licence).

#include <Arduino.h>
#include <Wire.h>

namespace m5env {

static inline bool readRegs(TwoWire* w, uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {
  w->beginTransmission(addr);
  w->write(reg);
  if (w->endTransmission() != 0) return false;   // full stop, see above
  if (w->requestFrom(addr, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = w->read();
  return true;
}

static inline bool writeReg(TwoWire* w, uint8_t addr, uint8_t reg, uint8_t val) {
  w->beginTransmission(addr);
  w->write(reg);
  w->write(val);
  return w->endTransmission() == 0;
}

// Sensirion CRC-8: polynomial 0x31, init 0xFF
static inline uint8_t crc8(const uint8_t* d, uint8_t n) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < n; i++) {
    crc ^= d[i];
    for (uint8_t b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
  }
  return crc;
}

// ---------------------------------------------------------------------------
// SHT30 / SHT31 / SHT35
// ---------------------------------------------------------------------------
class SHT3X {
public:
  bool begin(TwoWire* wire, uint8_t addr) {
    _w = wire; _a = addr;
    uint8_t b[3];
    if (!command(0xF32D)) return false;            // read status register
    delay(1);
    if (_w->requestFrom(_a, (uint8_t)3) != 3) return false;
    for (uint8_t i = 0; i < 3; i++) b[i] = _w->read();
    return crc8(b, 2) == b[2];                      // only an SHT3x answers this with a valid CRC
  }

  // Single shot, high repeatability, clock stretching off (fixed wait instead).
  bool read(float& temp_c, float& rh) {
    if (!command(0x2400)) return false;
    delay(16);
    uint8_t b[6];
    if (_w->requestFrom(_a, (uint8_t)6) != 6) return false;
    for (uint8_t i = 0; i < 6; i++) b[i] = _w->read();
    if (crc8(b, 2) != b[2] || crc8(b + 3, 2) != b[5]) return false;
    temp_c = -45.0f + 175.0f * (float)(((uint16_t)b[0] << 8) | b[1]) / 65535.0f;
    rh     = 100.0f * (float)(((uint16_t)b[3] << 8) | b[4]) / 65535.0f;
    return true;
  }

private:
  TwoWire* _w = nullptr;
  uint8_t  _a = 0x44;
  bool command(uint16_t c) {
    _w->beginTransmission(_a);
    _w->write((uint8_t)(c >> 8));
    _w->write((uint8_t)c);
    return _w->endTransmission() == 0;
  }
};

// ---------------------------------------------------------------------------
// DHT12 (I2C mode, fixed address 0x5C)
// Registers 0-4: humidity int, humidity tenths, temp int (bit 7 = minus),
// temp tenths, checksum (sum of bytes 0-3).
// ---------------------------------------------------------------------------
class DHT12 {
public:
  bool begin(TwoWire* wire, uint8_t addr) {
    _w = wire; _a = addr;
    float t, h;
    return read(t, h);
  }

  bool read(float& temp_c, float& rh) {
    uint8_t b[5];
    if (!readRegs(_w, _a, 0x00, b, 5)) return false;
    if ((uint8_t)(b[0] + b[1] + b[2] + b[3]) != b[4]) return false;
    if (b[0] > 100 || b[1] > 9 || b[3] > 9) return false;      // rejects a different chip whose bytes happen to sum up
    rh = b[0] + b[1] / 10.0f;
    float t = (b[2] & 0x7F) + b[3] / 10.0f;
    temp_c = (b[2] & 0x80) ? -t : t;
    return true;
  }

private:
  TwoWire* _w = nullptr;
  uint8_t  _a = 0x5C;
};

// ---------------------------------------------------------------------------
// QMP6988 (0x70, or 0x56 with SDO high)
// ---------------------------------------------------------------------------
class QMP6988 {
public:
  bool begin(TwoWire* wire, uint8_t addr) {
    _w = wire; _a = addr;
    uint8_t id = 0;
    if (!readRegs(_w, _a, 0xD1, &id, 1) || id != 0x5C) return false;   // CHIP_ID
    if (!loadOTP()) return false;
    writeReg(_w, _a, 0xF1, 0x02);                  // IIR filter coefficient 4
    writeReg(_w, _a, 0xF4, (0x01 << 5) | (0x04 << 2) | 0x03);   // T x1, P x8, normal mode
    delay(20);
    return true;
  }

  bool read(float& temp_c, float& pressure_pa) {
    uint8_t b[6];
    if (!readRegs(_w, _a, 0xF7, b, 6)) return false;
    int32_t dp = (int32_t)((((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2]) - 8388608UL);
    int32_t dt = (int32_t)((((uint32_t)b[3] << 16) | ((uint32_t)b[4] << 8) | b[5]) - 8388608UL);
    int16_t tx = tempx(dt);
    temp_c      = tx / 256.0f;
    pressure_pa = pressx(dp, tx) / 16.0f;
    return true;
  }

private:
  TwoWire* _w = nullptr;
  uint8_t  _a = 0x70;
  int32_t  a0, b00, a1, a2;
  int64_t  bt1, bt2, bp1, b11, bp2, b12, b21, bp3;

  bool loadOTP() {
    uint8_t c[25];
    for (uint8_t i = 0; i < 25; i++) if (!readRegs(_w, _a, 0xA0 + i, &c[i], 1)) return false;   // byte by byte
    auto s16 = [&](int i) { return (int16_t)(((uint16_t)c[i] << 8) | c[i + 1]); };
    a0  = (int32_t)((((uint32_t)c[18] << 12) | ((uint32_t)c[19] << 4) | (c[24] & 0x0F)) << 12) >> 12;   // 20-bit signed
    b00 = (int32_t)((((uint32_t)c[0] << 12) | ((uint32_t)c[1] << 4) | (c[24] >> 4)) << 12) >> 12;
    a1  = 3608L  * s16(20) - 1731677965L;
    a2  = 16889L * s16(22) - 87619360L;
    bt1 = 2982LL   * s16(2)  + 107370906LL;
    bt2 = 329854LL * s16(4)  + 108083093LL;
    bp1 = 19923LL  * s16(6)  + 1133836764LL;
    b11 = 2406LL   * s16(8)  + 118215883LL;
    bp2 = 3079LL   * s16(10) - 181579595LL;
    b12 = 6846LL   * s16(12) + 85590281LL;
    b21 = 13836LL  * s16(14) + 79333336LL;
    bp3 = 2915LL   * s16(16) + 157155561LL;
    return true;
  }

  int16_t tempx(int32_t dt) {                      // temperature x 256
    int64_t w1 = (int64_t)a1 * dt;
    int64_t w2 = (((int64_t)a2 * dt) >> 14) * dt >> 10;
    w2 = ((w1 + w2) / 32767) >> 19;
    return (int16_t)((a0 + w2) >> 4);
  }

  int32_t pressx(int32_t dp, int16_t tx) {         // pressure x 16, in Pa
    int64_t w1 = bt1 * tx + ((bp1 * dp) >> 5);
    int64_t w3 = (((bt2 * tx) >> 1) * tx) >> 8;
    w3 += (((b11 * tx) >> 4) * dp) >> 1;
    w3 += (((bp2 * dp) >> 13) * dp) >> 1;
    w1 += w3 >> 14;
    w3  = ((((b12 * tx) * tx) >> 22) * dp) >> 1;
    w3 += (((((b21 * tx) >> 6) * dp) >> 23) * dp) >> 1;
    w3 += (((bp3 * dp) >> 12) * dp >> 23) * dp;
    w1 += w3 >> 15;
    w1 /= 32767;
    w1 >>= 11;
    return (int32_t)(w1 + b00);
  }
};

} // namespace m5env
