/**
 * @brief Implementation of Sense Hat B ICM20948 for OrangePi 5
 * @author Yongkie Wiyogo
 */

// C++ Standard Library
#include <chrono>
#include <cstdio>
#include <thread>
#include <string>
#include <iostream>

// System headers
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

// Project headers
#include "portable_slam/icm20948.hpp"

ICM20948::ICM20948(int bus) {
  if (bus < 0) {
    throw std::runtime_error("Invalid I2C bus number");
  }
  std::string filename = "/dev/i2c-" + std::to_string(bus);
  i2c_file = open(filename.c_str(), O_RDWR);

  if (i2c_file < 0) {
    throw std::runtime_error("Could not open I2C bus");
  }

  if (ioctl(i2c_file, I2C_SLAVE, ICM_ADDRESS) < 0) {
    close(i2c_file);
    throw std::runtime_error("Could not set I2C slave address");
  }
  setPrintHeaderInterval(30);
  // Call initialization method
  initializeSensor();
}

ICM20948::~ICM20948() {
  if (i2c_file >= 0) {
    close(i2c_file);
  }
}

void ICM20948::writeRegister(uint8_t reg, uint8_t value) {
  uint8_t buffer[2] = {reg, value};
  if (write(i2c_file, buffer, 2) != 2) {
    throw std::runtime_error("Failed to write register");
  }
}

uint8_t ICM20948::readRegister(uint8_t reg) {
  // Set register pointer
  if (write(i2c_file, &reg, 1) != 1) {
    throw std::runtime_error("Error setting register pointer");
  }

  uint8_t data;
  if (read(i2c_file, &data, 1) != 1) {
    throw std::runtime_error("Error reading from register");
  }

  return data;
}

void ICM20948::initializeSensor() {
  try {
    // Software reset
    writeRegister(PWR_MGMT_1, 0x80); // PWR_MGMT_1 reset bit
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Wake up and select best available clock
    writeRegister(PWR_MGMT_1, 0x01); // Auto select best available clock

    // Enable accelerometer and gyroscope
    writeRegister(PWR_MGMT_2, 0x00); // Disable all interrupts
    writeRegister(USER_CTRL, 0x07);  // Enable accel and gyro

    // Configure accelerometer (±16g range)
    writeRegister(ACCEL_CONFIG, 0x02); // Accel config

    // Configure gyroscope (±2000 dps range)
    writeRegister(GYRO_CONFIG, 0x01); // Gyro config

    // Optional: Print registers to verify
    // printRegisters();
  } catch (const std::exception &e) {
    std::cerr << "Initialization error: " << e.what() << std::endl;
    throw;
  }
}

std::tuple<int16_t, int16_t, int16_t, int16_t, int16_t, int16_t> ICM20948::readSensorData() {
    int16_t accel_x = (static_cast<int16_t>(readRegister(ACCEL_XOUT_H)) << 8) |
                      readRegister(ACCEL_XOUT_L);
    int16_t accel_y = (static_cast<int16_t>(readRegister(ACCEL_YOUT_H)) << 8) |
                      readRegister(ACCEL_YOUT_L);
    int16_t accel_z = (static_cast<int16_t>(readRegister(ACCEL_ZOUT_H)) << 8) |
                      readRegister(ACCEL_ZOUT_L);
    
    int16_t gyro_x = (static_cast<int16_t>(readRegister(GYRO_XOUT_H)) << 8) |
                     readRegister(GYRO_XOUT_L);
    int16_t gyro_y = (static_cast<int16_t>(readRegister(GYRO_YOUT_H)) << 8) |
                     readRegister(GYRO_YOUT_L);
    int16_t gyro_z = (static_cast<int16_t>(readRegister(GYRO_ZOUT_H)) << 8) |
                     readRegister(GYRO_ZOUT_L);

    return std::make_tuple(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
}

void ICM20948::printSensorData() {
  auto [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z] = readSensorData();

  // Print values
  std::cout << accel_x << " (" << convertAcceleration(accel_x) << " m/s²)"
            << "\t" << accel_y << " (" << convertAcceleration(accel_y)
            << " m/s²)" << "\t" << accel_z << " ("
            << convertAcceleration(accel_z) << " m/s²)" << "\t" << gyro_x
            << " (" << convertGyro(gyro_x) << " deg/s)" << std::endl;

  // Optional: Periodic header refresh
  if (++counter >= print_header_interval) {
    counter = 0;
    std::cout << "  Accel X\t\t Accel Y\t\t Accel Z\t\t Gyro X" << std::endl;
  }
}

float ICM20948::convertAcceleration(int16_t rawValue) {
  const float ACCEL_SCALE = 16.0f * 9.81f / 32768.0f;
  return rawValue * ACCEL_SCALE;
}
float ICM20948::convertGyro(int16_t rawValue) {
  const float GYRO_SCALE = 2000.0f / 32768.0f;
  return rawValue * GYRO_SCALE;
}