/**
 * @brief Implementation of LPS22HB barometric pressure sensor driver
 * @author Yongkie Wiyogo
 */

#include <chrono>
#include <cstdio>
#include <iostream>
#include <string>
#include <thread>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "portable_slam/lps22hb.hpp"

LPS22HB::LPS22HB(int bus) : i2c_file(-1) {
  if (bus < 0) {
    throw std::runtime_error("Invalid I2C bus number");
  }

  std::string filename = "/dev/i2c-" + std::to_string(bus);
  i2c_file = open(filename.c_str(), O_RDWR);

  if (i2c_file < 0) {
    char err_str[256];
    snprintf(err_str, sizeof(err_str), "Could not open I2C bus %s",
             filename.c_str());
    throw std::runtime_error(err_str);
  }

  if (ioctl(i2c_file, I2C_SLAVE, LPS22HB_ADDRESS) < 0) {
    close(i2c_file);
    i2c_file = -1;
    throw std::runtime_error("Could not set I2C slave address for LPS22HB");
  }

  initializeSensor();
}

LPS22HB::~LPS22HB() {
  if (i2c_file >= 0) {
    close(i2c_file);
  }
}

void LPS22HB::writeRegister(uint8_t reg, uint8_t value) {
  uint8_t buffer[2] = {reg, value};
  if (write(i2c_file, buffer, 2) != 2) {
    throw std::runtime_error("LPS22HB: Failed to write register");
  }
}

uint8_t LPS22HB::readRegister(uint8_t reg) {
  if (write(i2c_file, &reg, 1) != 1) {
    throw std::runtime_error("LPS22HB: Error setting register pointer");
  }

  uint8_t data;
  if (read(i2c_file, &data, 1) != 1) {
    throw std::runtime_error("LPS22HB: Error reading from register");
  }
  return data;
}

int16_t LPS22HB::readRegister16(uint8_t reg_low) {
  uint8_t low = readRegister(reg_low);
  uint8_t high = readRegister(reg_low + 1);
  return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}

bool LPS22HB::isPressureReady() {
  uint8_t status = readRegister(LPS22HB_STATUS_REG);
  return (status & 0x02) != 0;
}

void LPS22HB::initializeSensor() {
  uint8_t who_am_i = readRegister(LPS22HB_WHO_AM_I);
  if (who_am_i != LPS22HB_DEVICE_ID) {
    char err[128];
    snprintf(err, sizeof(err),
             "LPS22HB WHO_AM_I mismatch: expected 0x%02X, got 0x%02X",
             LPS22HB_DEVICE_ID, who_am_i);
    throw std::runtime_error(err);
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  writeRegister(LPS22HB_CTRL_REG1, LPS22HB_ODR_25HZ | LPS22HB_BDU);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  uint8_t ctrl2 = readRegister(LPS22HB_CTRL_REG2);
  writeRegister(LPS22HB_CTRL_REG2, ctrl2 | 0x10);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  writeRegister(LPS22HB_CTRL_REG2, ctrl2 & ~0x10);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  std::cout << "LPS22HB initialized (ODR=25Hz, BDU=enabled)" << std::endl;
}

PressureData LPS22HB::readData() {
  PressureData data{};

  if (!isPressureReady()) {
    data.pressure_hpa = 0.0;
    data.temperature_c = 0.0;
    data.altitude_m = 0.0;
    return data;
  }

  uint8_t pxl = readRegister(LPS22HB_PRESS_OUT_XL);
  uint8_t pl = readRegister(LPS22HB_PRESS_OUT_L);
  uint8_t ph = readRegister(LPS22HB_PRESS_OUT_H);
  uint32_t raw_pressure =
      (static_cast<uint32_t>(ph) << 16) | (static_cast<uint32_t>(pl) << 8) |
      static_cast<uint32_t>(pxl);

  int16_t raw_temp = readRegister16(LPS22HB_TEMP_OUT_L);

  data.pressure_hpa = static_cast<double>(raw_pressure) *
                       LPS22HB_PRESS_SENSITIVITY;
  data.temperature_c =
      static_cast<double>(raw_temp) * LPS22HB_TEMP_SENSITIVITY;
  data.altitude_m = getAltitude(data.pressure_hpa);

  return data;
}

double LPS22HB::getAltitude(double pressure_hpa,
                            double reference_hpa) const {
  if (pressure_hpa <= 0.0 || reference_hpa <= 0.0) {
    return 0.0;
  }
  return 44330.0 * (1.0 - std::pow(pressure_hpa / reference_hpa, 0.1903));
}