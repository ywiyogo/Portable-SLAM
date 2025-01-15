/**
 * @brief Implementation of Sense Hat B ICM20948 for OrangePi 5
 * @author Yongkie Wiyogo
 */
 
#pragma once

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <tuple>

static constexpr uint8_t PWR_MGMT_1 = 0x06;
static constexpr uint8_t PWR_MGMT_2 = 0x07;
static constexpr uint8_t USER_CTRL = 0x0E;
static constexpr uint8_t ACCEL_CONFIG = 0x14;
static constexpr uint8_t GYRO_CONFIG = 0x15;
static constexpr double GRAVITY = 9.81; // m/s^2
static constexpr double DEG_TO_RAD = M_PI / 180.0;

class ICM20948 {
private:
  int i2c_file;
  const uint8_t ICM_ADDRESS = 0x68;

  // Register addresses (typical for ICM-20948)
  static constexpr uint8_t ACCEL_XOUT_H = 0x2D;
  static constexpr uint8_t ACCEL_XOUT_L = 0x2E;
  static constexpr uint8_t ACCEL_YOUT_H = 0x2F;
  static constexpr uint8_t ACCEL_YOUT_L = 0x30;
  static constexpr uint8_t ACCEL_ZOUT_H = 0x31;
  static constexpr uint8_t ACCEL_ZOUT_L = 0x32;
  static constexpr uint8_t GYRO_XOUT_H = 0x33;
  static constexpr uint8_t GYRO_XOUT_L = 0x34;
  static constexpr uint8_t GYRO_YOUT_H = 0x35;
  static constexpr uint8_t GYRO_YOUT_L = 0x36;
  static constexpr uint8_t GYRO_ZOUT_H = 0x37;
  static constexpr uint8_t GYRO_ZOUT_L = 0x38;

  // Scaling factor for conversion functions
  static constexpr float ACCEL_SCALE = 16.0f * 9.81f / 32768.0f;
  static constexpr float GYRO_SCALE = 2000.0f / 32768.0f;

  // Counter for displaying headers periodically
  int print_header_interval;
  int counter = 0;

  inline void setPrintHeaderInterval(int interval) {
    if (interval <= 0) {
      throw std::invalid_argument("Header interval must be positive");
    }
    print_header_interval = interval;
  }

protected:
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t value);

public:
  //  Standard deviation for ovariance matrices
  //   Linear Acceleration (Accelerometer):
  // Full Scale Range: ±16g
  // Initial Calibration Tolerance: ±50mg
  // Zero-G Initial Calibration Tolerance: ±50mg
  // 50mg = 0.05g = 0.05 * 9.81 = 0.4905 m/s^2
  static constexpr double ACCEL_TOLERANCE_G = 0.05; // 50mg
  static constexpr double ACCEL_STDEV = ACCEL_TOLERANCE_G * GRAVITY;
  static constexpr double ACCEL_VARIANCE = ACCEL_STDEV * ACCEL_STDEV;

  // Angular Rate (Gyroscope):
  // Full Scale Range: ±2000°/s
  // Initial Zero Rate Output (ZRO) Tolerance: ±1°/s
  // 1°/s = 0.017453293 rad/s

  static constexpr double GYRO_TOLERANCE_DEG = 1.0; // 1 degree/s
  static constexpr double GYRO_STDEV = GYRO_TOLERANCE_DEG * DEG_TO_RAD;
  static constexpr double GYRO_VARIANCE = GYRO_STDEV * GYRO_STDEV;

  ICM20948(int bus = 5);
  virtual ~ICM20948();

  ICM20948(const ICM20948 &) = delete;
  ICM20948 &operator=(const ICM20948 &) = delete;

  void initializeSensor();

  /**
   The ICM-20948's ADC (Analog-to-Digital Converter) is 16-bit. Each sensor
   reading (accelerometer and gyroscope) outputs a 16-bit value. The data
   registers are organized as high byte (8-bit) and low byte (8-bit) pairs
   */
  std::tuple<int16_t, int16_t, int16_t, int16_t, int16_t, int16_t>
  readSensorData();

  float convertAcceleration(int16_t rawValue);
  float convertGyro(int16_t rawValue);

  void printSensorData();
};