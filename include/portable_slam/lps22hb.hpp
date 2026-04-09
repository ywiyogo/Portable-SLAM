/**
 * @brief Implementation of LPS22HB barometric pressure sensor for the
 *        Waveshare Sense HAT B.
 *        Datasheet:
 * https://www.st.com/resource/en/datasheet/lps22hb.pdf
 * @author Yongkie Wiyogo
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <stdexcept>

static constexpr uint8_t LPS22HB_ADDRESS = 0x5C;

// LPS22HB Register Map
static constexpr uint8_t LPS22HB_WHO_AM_I = 0x0F;
static constexpr uint8_t LPS22HB_DEVICE_ID = 0xB1;

static constexpr uint8_t LPS22HB_CTRL_REG1 = 0x10;
static constexpr uint8_t LPS22HB_CTRL_REG2 = 0x11;
static constexpr uint8_t LPS22HB_CTRL_REG3 = 0x12;

static constexpr uint8_t LPS22HB_STATUS_REG = 0x27;

static constexpr uint8_t LPS22HB_PRESS_OUT_XL = 0x28;
static constexpr uint8_t LPS22HB_PRESS_OUT_L = 0x29;
static constexpr uint8_t LPS22HB_PRESS_OUT_H = 0x2A;

static constexpr uint8_t LPS22HB_TEMP_OUT_L = 0x2B;
static constexpr uint8_t LPS22HB_TEMP_OUT_H = 0x2C;

static constexpr uint8_t LPS22HB_LPFP_RES = 0x33;

// Output Data Rate selections (CTRL_REG1 bits 7:4)
static constexpr uint8_t LPS22HB_ODR_ONE_SHOT = 0x00;
static constexpr uint8_t LPS22HB_ODR_1HZ = 0x10;
static constexpr uint8_t LPS22HB_ODR_10HZ = 0x20;
static constexpr uint8_t LPS22HB_ODR_25HZ = 0x30;
static constexpr uint8_t LPS22HB_ODR_50HZ = 0x40;
static constexpr uint8_t LPS22HB_ODR_75HZ = 0x50;

// CTRL_REG1 bit masks
static constexpr uint8_t LPS22HB_EN_LPFP = 0x08;
static constexpr uint8_t LPS22HB_BDU = 0x02;

// Pressure sensitivity: 4096 LSB/hPa (= 0.0244140625 hPa/LSB)
static constexpr double LPS22HB_PRESS_SENSITIVITY = 1.0 / 4096.0;

// Temperature sensitivity: 100 LSB/degC
static constexpr double LPS22HB_TEMP_SENSITIVITY = 1.0 / 100.0;

// Physical constants
static constexpr double SEA_LEVEL_PRESSURE_HPA = 1013.25;
static constexpr double PRESSURE_VARIANCE = 0.01;

struct PressureData {
  double pressure_hpa;
  double temperature_c;
  double altitude_m;
};

class LPS22HB {
public:
  LPS22HB(int bus = 5);
  ~LPS22HB();

  LPS22HB(const LPS22HB &) = delete;
  LPS22HB &operator=(const LPS22HB &) = delete;

  void initializeSensor();
  PressureData readData();

  double getAltitude(double pressure_hpa,
                     double reference_hpa = SEA_LEVEL_PRESSURE_HPA) const;

private:
  int i2c_file;

  void writeRegister(uint8_t reg, uint8_t value);
  uint8_t readRegister(uint8_t reg);
  int16_t readRegister16(uint8_t reg_low);

  bool isPressureReady();
};