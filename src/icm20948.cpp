/**
 * @brief Implementation of Sense Hat B ICM20948 for OrangePi 5
 * @author Yongkie Wiyogo
 */

// C++ Standard Library
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

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
    char err_str[256];
    snprintf(err_str, sizeof(err_str), "Could not open I2C bus %s",
             filename.c_str());
    throw std::runtime_error(err_str);
  }

  if (ioctl(i2c_file, I2C_SLAVE, ICM_ADDRESS) < 0) {
    close(i2c_file);
    perror("ioctl I2C_SLAVE failed");
    throw std::runtime_error("Could not set I2C slave address");
  }

  // Additional check for valid file descriptor
  if (fcntl(i2c_file, F_GETFL) == -1) {
    throw std::runtime_error("Invalid I2C file descriptor");
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

void ICM20948::selectBank(uint8_t bank) {
  if (current_bank != bank) {
    uint8_t buffer[2] = {REG_BANK_SEL, static_cast<uint8_t>(bank << 4)};
    if (write(i2c_file, buffer, 2) != 2) {
      throw std::runtime_error("Failed to select bank");
    }
    current_bank = bank;
  }
}

void ICM20948::writeRegister(uint8_t bank, uint8_t reg, uint8_t value) {
  selectBank(bank);
  uint8_t buffer[2] = {reg, value};
  if (write(i2c_file, buffer, 2) != 2) {
    std::string error =
        std::format("Failed to write register {:#x} value {:#x}", reg, value);
    throw std::runtime_error(error.c_str());
  }
}

uint8_t ICM20948::readRegister(uint8_t bank, uint8_t reg) {
  selectBank(bank);
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
    // Software reset and wait
    writeRegister(0, PWR_MGMT_1, 0x80);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Wake up device and select auto clock source
    writeRegister(0, PWR_MGMT_1, 0x01);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Enable all sensors
    writeRegister(0, PWR_MGMT_2, 0x00);

    // Reset all modules
    writeRegister(0, USER_CTRL, 0x00);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Configure gyroscope in bank 2
    selectBank(2);
    writeRegister(2, GYRO_CONFIG_1, GYRO_FS_SEL_0); // ±250 dps
    writeRegister(2, GYRO_SMPLRT_DIV, 0x00);        // 1.1 kHz sample rate

    // Configure accelerometer in bank 2
    writeRegister(2, ACCEL_CONFIG, ACCEL_SCALE.at(0)); // set to ±2g
    writeRegister(2, ACCEL_SMPLRT_DIV_1, 0x00);        // 1.125 kHz sample rate
    writeRegister(2, ACCEL_SMPLRT_DIV_2, 0x00);

    // Return to bank 0 for normal operation
    selectBank(0);

    // Enable I2C master mode
    writeRegister(0, USER_CTRL, I2C_MST_EN);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Initialize magnetometer
    initializeMagnetometer();

    // Load calibration if available
    if (!loadCalibration(
            "/home/ubuntu/.config/portable_slam/imu_calibration.yaml")) {
      std::cout << "No calibration data found. Please run calibration."
                << std::endl;
    }

  } catch (const std::exception &e) {
    std::cerr << "Initialization error: " << e.what() << std::endl;
    throw;
  }
}

Vector3 ICM20948::collectSamples(int numSamples) {
  Vector3 sum;
  for (int i = 0; i < numSamples; ++i) {
    auto data = readSensorData();
    sum.x += data.gyro.x;
    sum.y += data.gyro.y;
    sum.z += data.gyro.z;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return Vector3(sum.x / numSamples, sum.y / numSamples, sum.z / numSamples);
}

void ICM20948::calibrateGyro() {
  std::cout << "Calibrating gyroscope. Keep the device still..." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(2));

  Vector3 gyroBias = collectSamples(CALIBRATION_SAMPLES);

  calibration_.gyroBias.x = convertGyro(gyroBias.x);
  calibration_.gyroBias.y = convertGyro(gyroBias.y);
  calibration_.gyroBias.z = convertGyro(gyroBias.z);

  std::cout << "Gyroscope calibration complete." << std::endl;
  std::cout << "Bias: X=" << calibration_.gyroBias.x
            << " Y=" << calibration_.gyroBias.y
            << " Z=" << calibration_.gyroBias.z << std::endl;
}

void ICM20948::calibrateAccel() {
  std::cout << "Accelerometer calibration - 6 position calibration"
            << std::endl;
  std::cout << "Place the device in 6 different positions..." << std::endl;

  Vector3 minVals(1e6, 1e6, 1e6);
  Vector3 maxVals(-1e6, -1e6, -1e6);

  for (int pos = 0; pos < 6; ++pos) {
    std::cout << "Position " << pos + 1 << "/6. Waiting for 5 secs" << std::endl
              << std::flush;
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    std::cout << "Calibrating ..." << std::endl << std::flush;
    Vector3 samples = collectSamples(CALIBRATION_SAMPLES);
    std::cout << "Finished" << std::endl << std::flush;
    minVals.x = std::min(minVals.x, samples.x);
    minVals.y = std::min(minVals.y, samples.y);
    minVals.z = std::min(minVals.z, samples.z);

    maxVals.x = std::max(maxVals.x, samples.x);
    maxVals.y = std::max(maxVals.y, samples.y);
    maxVals.z = std::max(maxVals.z, samples.z);
  }

  calibration_.accelBias.x = (minVals.x + maxVals.x);
  calibration_.accelBias.y = (minVals.y + maxVals.y);
  calibration_.accelBias.z = (minVals.z + maxVals.z);

  calibration_.accelScale.x = (maxVals.x - minVals.x);
  calibration_.accelScale.y = (maxVals.y - minVals.y);
  calibration_.accelScale.z = (maxVals.z - minVals.z);

  std::cout << "Accelerometer calibration complete." << std::endl;
}

void ICM20948::calibrateMag() {
  std::cout << "Magnetometer calibration" << std::endl;
  std::cout << "Wave the device in a figure-8 pattern for 30 seconds..."
            << std::endl;

  Vector3 minVals(1e6, 1e6, 1e6);
  Vector3 maxVals(-1e6, -1e6, -1e6);

  auto startTime = std::chrono::steady_clock::now();
  while (std::chrono::duration_cast<std::chrono::seconds>(
             std::chrono::steady_clock::now() - startTime)
             .count() < 30) {

    auto data = readSensorData();

    minVals.x = std::min(minVals.x, static_cast<double>(data.mag.x));
    minVals.y = std::min(minVals.y, static_cast<double>(data.mag.y));
    minVals.z = std::min(minVals.z, static_cast<double>(data.mag.z));

    maxVals.x = std::max(maxVals.x, static_cast<double>(data.mag.x));
    maxVals.y = std::max(maxVals.y, static_cast<double>(data.mag.y));
    maxVals.z = std::max(maxVals.z, static_cast<double>(data.mag.z));

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  calibration_.magBias.x = (minVals.x + maxVals.x) / 2.0;
  calibration_.magBias.y = (minVals.y + maxVals.y) / 2.0;
  calibration_.magBias.z = (minVals.z + maxVals.z) / 2.0;

  calibration_.magScale.x = 2.0 / (maxVals.x - minVals.x);
  calibration_.magScale.y = 2.0 / (maxVals.y - minVals.y);
  calibration_.magScale.z = 2.0 / (maxVals.z - minVals.z);

  std::cout << "Magnetometer calibration complete." << std::endl;
}

void ICM20948::performCalibration() {
  calibrateGyro();
  calibrateAccel();
  calibrateMag();
  calibration_.isCalibrated = true;
  saveCalibration("/home/ubuntu/.config/portable_slam/imu_calibration.yaml");
}

bool ICM20948::loadCalibration(const std::string &filename) {
  try {
    std::ifstream file(filename);
    if (!file.is_open()) {
      return false;
    }

    // Simple YAML parsing
    std::string line;
    while (std::getline(file, line)) {
      std::istringstream iss(line);
      std::string key;
      double value;

      if (std::getline(iss, key, ':') && iss >> value) {
        if (key == "gyro_bias_x")
          calibration_.gyroBias.x = value;
        else if (key == "gyro_bias_y")
          calibration_.gyroBias.y = value;
        else if (key == "gyro_bias_z")
          calibration_.gyroBias.z = value;
        // Add other calibration parameters...
      }
    }

    calibration_.isCalibrated = true;
    return true;
  } catch (const std::exception &e) {
    std::cerr << "Error loading calibration: " << e.what() << std::endl;
    return false;
  }
}

bool ICM20948::saveCalibration(const std::string &filename) const {
  try {
    // Create directory if it doesn't exist
    std::filesystem::create_directories(
        std::filesystem::path(filename).parent_path());

    std::ofstream file(filename);
    if (!file.is_open()) {
      return false;
    }

    // Write calibration data in YAML format
    file << "gyro_bias_x: " << calibration_.gyroBias.x << "\n";
    file << "gyro_bias_y: " << calibration_.gyroBias.y << "\n";
    file << "gyro_bias_z: " << calibration_.gyroBias.z << "\n";
    // Add other calibration parameters...

    return true;
  } catch (const std::exception &e) {
    std::cerr << "Error saving calibration: " << e.what() << std::endl;
    return false;
  }
}

void ICM20948::initializeMagnetometer() {
  const int MAX_RETRIES = 3;
  const int INIT_DELAY = 200; // Increased initial delay

  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    try {
      // Initial delay to ensure I2C bus is ready
      std::this_thread::sleep_for(std::chrono::milliseconds(INIT_DELAY));

      // First disable I2C master mode
      writeRegister(0, USER_CTRL, readRegister(0, USER_CTRL) & ~I2C_MST_EN);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));

      // Reset I2C master module
      writeRegister(0, USER_CTRL, readRegister(0, USER_CTRL) | I2C_MST_RST);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));

      // Configure I2C master clock frequency and settings
      writeRegister(3, I2C_MST_CTRL,
                    0x07); // 345.6 kHz I2C clock (instead of 400kHz)
      std::this_thread::sleep_for(std::chrono::milliseconds(50));

      // Enable I2C master mode
      writeRegister(0, USER_CTRL, readRegister(0, USER_CTRL) | I2C_MST_EN);
      std::this_thread::sleep_for(
          std::chrono::milliseconds(100)); // Longer delay after enabling

      // Configure I2C master delays
      writeRegister(3, I2C_MST_DELAY_CTRL,
                    0x01); // Enable delay for external sensor
      std::this_thread::sleep_for(std::chrono::milliseconds(50));

      // Reset magnetometer with proper delays
      writeRegister(3, I2C_SLV0_ADDR,
                    MAG_I2C_ADDR); // Write operation to magnetometer
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      writeRegister(3, I2C_SLV0_REG, MAG_CNTL3); // Control 3 register
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      writeRegister(3, I2C_SLV0_DO, 0x01); // Soft reset
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      writeRegister(3, I2C_SLV0_CTRL, 0x81); // Enable writing 1 byte
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      // Verify magnetometer ID
      writeRegister(3, I2C_SLV0_ADDR, MAG_I2C_ADDR | 0x80); // Read operation
      writeRegister(3, I2C_SLV0_REG, MAG_WIA2); // Device ID register
      writeRegister(3, I2C_SLV0_CTRL, 0x81);    // Enable reading 1 byte
      std::this_thread::sleep_for(
          std::chrono::milliseconds(200)); // Increased delay

      uint8_t mag_id = readRegister(0, EXT_SLV_SENS_DATA_00);
      std::cout << "Magnetometer ID read: 0x" << std::hex
                << static_cast<int>(mag_id) << std::dec << std::endl;

      if (mag_id != MAG_DEVICE_ID) {
        if (retry == MAX_RETRIES - 1) {
          std::stringstream ss;
          ss << "Magnetometer ID mismatch (expected 0x09, got 0x" << std::hex
             << static_cast<int>(mag_id) << ")";
          throw std::runtime_error(ss.str());
        }
        std::cout << "Retrying magnetometer initialization (attempt "
                  << retry + 1 << " of " << MAX_RETRIES << ")" << std::endl;
        continue;
      }

      // If we get here, initialization was successful
      break;
    } catch (const std::exception &e) {
      if (retry == MAX_RETRIES - 1) {
        throw; // Re-throw on last retry
      }
      std::cout << "Retrying due to error: " << e.what() << std::endl;
    }
  }

  // Set magnetometer to continuous measurement mode 4 (100Hz)
  writeRegister(3, I2C_SLV0_ADDR, MAG_I2C_ADDR); // Write operation
  writeRegister(3, I2C_SLV0_REG, MAG_CNTL2);     // Control 2 register
  writeRegister(3, I2C_SLV0_DO, 0x08);           // Continuous mode 4 (100Hz)
  writeRegister(3, I2C_SLV0_CTRL, 0x81);         // Enable writing 1 byte
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Set up automatic reading of magnetometer status and data
  // First slave reads ST1 (status 1), ST2 (status 2), and six data bytes
  writeRegister(3, I2C_SLV0_ADDR, MAG_I2C_ADDR | 0x80); // Read operation
  writeRegister(3, I2C_SLV0_REG, MAG_ST1); // Start reading from Status 1
  writeRegister(3, I2C_SLV0_CTRL, 0x88);   // Enable reading 8 bytes

  // Wait for first measurement to complete (100Hz = 10ms)
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

IMUSensorData ICM20948::readSensorData() {
  IMUSensorData data;

  // Read accelerometer data (Bank 0)
  data.accel.x = (static_cast<int16_t>(readRegister(0, ACCEL_XOUT_H)) << 8) |
                 readRegister(0, ACCEL_XOUT_L);
  data.accel.y = (static_cast<int16_t>(readRegister(0, ACCEL_YOUT_H)) << 8) |
                 readRegister(0, ACCEL_YOUT_L);
  data.accel.z = (static_cast<int16_t>(readRegister(0, ACCEL_ZOUT_H)) << 8) |
                 readRegister(0, ACCEL_ZOUT_L);

  // Read gyroscope data (Bank 0)
  data.gyro.x = (static_cast<int16_t>(readRegister(0, GYRO_XOUT_H)) << 8) |
                readRegister(0, GYRO_XOUT_L);
  data.gyro.y = (static_cast<int16_t>(readRegister(0, GYRO_YOUT_H)) << 8) |
                readRegister(0, GYRO_YOUT_L);
  data.gyro.z = (static_cast<int16_t>(readRegister(0, GYRO_ZOUT_H)) << 8) |
                readRegister(0, GYRO_ZOUT_L);

  // Read magnetometer status and data from external sensor data registers
  uint8_t mag_status1 = readRegister(0, EXT_SLV_SENS_DATA_00);

  // Check if new data is ready
  if (!(mag_status1 & 0x01)) {
    // If data is not ready, return last known good values
    // This is better than throwing an error since the magnetometer updates more
    // slowly
    return data;
  }

  // Read magnetometer data (data registers follow status register)
  data.mag.x =
      (static_cast<int16_t>(readRegister(0, EXT_SLV_SENS_DATA_02)) << 8) |
      readRegister(0, EXT_SLV_SENS_DATA_01);
  data.mag.y =
      (static_cast<int16_t>(readRegister(0, EXT_SLV_SENS_DATA_04)) << 8) |
      readRegister(0, EXT_SLV_SENS_DATA_03);
  data.mag.z =
      (static_cast<int16_t>(readRegister(0, EXT_SLV_SENS_DATA_06)) << 8) |
      readRegister(0, EXT_SLV_SENS_DATA_05);

  // Check overflow flag in status 2 register (follows the 6 data bytes)
  uint8_t mag_status2 = readRegister(0, EXT_SLV_SENS_DATA_07);
  if (mag_status2 & 0x08) {
    // Log overflow but don't throw - just mark the data as invalid by zeroing
    // it
    std::cerr << "Magnetometer data overflow detected" << std::endl;
    data.mag.x = 0;
    data.mag.y = 0;
    data.mag.z = 0;
  }

  return data;
}

void ICM20948::printSensorData() {
  try {
    IMUSensorData data = readSensorData();

    // Print values with proper units
    Vector3 accel = convertAcceleration(data.accel);
    std::cout << "Accelerometer (m/s2):" << " X: " << accel.x
              << " Y: " << accel.y << " Z: " << accel.z << "\n";

    std::cout << "Gyroscope (deg/s):" << " X: " << convertGyro(data.gyro.x)
              << " Y: " << convertGyro(data.gyro.y)
              << " Z: " << convertGyro(data.gyro.z) << "\n";

    std::cout << "Magnetometer (μT):" << " X: "
              << convertMagneticField(data.mag.x)
              << " Y: " << convertMagneticField(data.mag.y)
              << " Z: " << convertMagneticField(data.mag.z) << std::endl;

    // Optional: Periodic header refresh
    if (++counter >= print_header_interval) {
      counter = 0;
      std::cout << "\n=== Sensor Readings ===" << std::endl;
    }
  } catch (const std::exception &e) {
    std::cerr << "Error reading sensor data: " << e.what() << std::endl;
  }
}

Vector3 ICM20948::convertAcceleration(const AccelData &rawValue) {
  Vector3 result;

  // Convert raw values to g's first
  float x = rawValue.x / ACCEL_SCALE.at(0);
  float y = rawValue.y / ACCEL_SCALE.at(0);
  float z = rawValue.z / ACCEL_SCALE.at(0);

  if (calibration_.isCalibrated) {
    // Apply calibration and axis inversion
    x = (x - calibration_.accelBias.x) * calibration_.accelScale.x;
    y = (y - calibration_.accelBias.y) * calibration_.accelScale.y;
    z = (z - calibration_.accelBias.z) * calibration_.accelScale.z;
  }

  // Convert from g's to m/s2
  result.x = x * GRAVITY;
  result.y = y * GRAVITY;
  result.z = z * GRAVITY;

  return result;
}

float ICM20948::convertGyro(int16_t rawValue) {
  if (!calibration_.isCalibrated) {
    return rawValue / GYRO_SCALE;
  }
  float value = rawValue / GYRO_SCALE;
  value -= calibration_.gyroBias.x;
  return value;
}

float ICM20948::convertMagneticField(int16_t rawValue) {
  if (!calibration_.isCalibrated) {
    return rawValue * MAG_SCALE;
  }
  float value = rawValue * MAG_SCALE;
  value = (value - calibration_.magBias.x) * calibration_.magScale.x;
  return value;
}
