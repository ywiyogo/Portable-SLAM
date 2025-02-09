/**
 * @brief Implementation of Sense Hat B ICM20948 for OrangePi 5
 * @author Yongkie Wiyogo
 */
 
#pragma once

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <tuple>

// ICM20948 Constants
static constexpr uint8_t ICM_ADDRESS = 0x68;    // I2C address
static constexpr uint8_t REG_BANK_SEL = 0x7F;   // Bank selection register

// Bank 0 Registers (Power Management, Interrupt Configuration)
static constexpr uint8_t WHO_AM_I = 0x00;       // Device ID register
static constexpr uint8_t USER_CTRL = 0x03;      // User control
static constexpr uint8_t LP_CONFIG = 0x05;      // Low power configuration
static constexpr uint8_t PWR_MGMT_1 = 0x06;     // Power management 1
static constexpr uint8_t PWR_MGMT_2 = 0x07;     // Power management 2
static constexpr uint8_t INT_PIN_CFG = 0x0F;    // Interrupt pin configuration
static constexpr uint8_t INT_ENABLE = 0x10;     // Interrupt enable
static constexpr uint8_t INT_ENABLE_1 = 0x11;   // Interrupt enable 1
static constexpr uint8_t INT_ENABLE_2 = 0x12;   // Interrupt enable 2
static constexpr uint8_t INT_ENABLE_3 = 0x13;   // Interrupt enable 3

// Bank 0 Sensor Data Registers
static constexpr uint8_t ACCEL_XOUT_H = 0x2D;   // Accelerometer X-axis high byte
static constexpr uint8_t ACCEL_XOUT_L = 0x2E;   // Accelerometer X-axis low byte
static constexpr uint8_t ACCEL_YOUT_H = 0x2F;   // Accelerometer Y-axis high byte
static constexpr uint8_t ACCEL_YOUT_L = 0x30;   // Accelerometer Y-axis low byte
static constexpr uint8_t ACCEL_ZOUT_H = 0x31;   // Accelerometer Z-axis high byte
static constexpr uint8_t ACCEL_ZOUT_L = 0x32;   // Accelerometer Z-axis low byte
static constexpr uint8_t GYRO_XOUT_H = 0x33;    // Gyroscope X-axis high byte
static constexpr uint8_t GYRO_XOUT_L = 0x34;    // Gyroscope X-axis low byte
static constexpr uint8_t GYRO_YOUT_H = 0x35;    // Gyroscope Y-axis high byte
static constexpr uint8_t GYRO_YOUT_L = 0x36;    // Gyroscope Y-axis low byte
static constexpr uint8_t GYRO_ZOUT_H = 0x37;    // Gyroscope Z-axis high byte
static constexpr uint8_t GYRO_ZOUT_L = 0x38;    // Gyroscope Z-axis low byte

// Bank 0 External Sensor Data Registers
static constexpr uint8_t EXT_SENS_DATA_00 = 0x3B;  // External sensor data 00
static constexpr uint8_t EXT_SENS_DATA_01 = 0x3C;  // External sensor data 01
static constexpr uint8_t EXT_SENS_DATA_02 = 0x3D;  // External sensor data 02
static constexpr uint8_t EXT_SENS_DATA_03 = 0x3E;  // External sensor data 03
static constexpr uint8_t EXT_SENS_DATA_04 = 0x3F;  // External sensor data 04
static constexpr uint8_t EXT_SENS_DATA_05 = 0x40;  // External sensor data 05
static constexpr uint8_t EXT_SENS_DATA_06 = 0x41;  // External sensor data 06
static constexpr uint8_t EXT_SENS_DATA_07 = 0x42;  // External sensor data 07

// Bank 2 Registers (Sensor Configuration)
static constexpr uint8_t GYRO_SMPLRT_DIV = 0x00;   // Gyroscope sample rate divider
static constexpr uint8_t GYRO_CONFIG_1 = 0x01;     // Gyroscope configuration 1
static constexpr uint8_t ACCEL_SMPLRT_DIV_1 = 0x10;// Accelerometer sample rate divider 1
static constexpr uint8_t ACCEL_SMPLRT_DIV_2 = 0x11;// Accelerometer sample rate divider 2
static constexpr uint8_t ACCEL_CONFIG = 0x14;      // Accelerometer configuration
static constexpr uint8_t ACCEL_CONFIG_2 = 0x15;    // Accelerometer configuration 2

// Bank 3 I2C Master Registers
static constexpr uint8_t I2C_MST_CTRL = 0x01;      // I2C master control
static constexpr uint8_t I2C_SLV0_ADDR = 0x03;     // I2C slave 0 address
static constexpr uint8_t I2C_SLV0_REG = 0x04;      // I2C slave 0 register
static constexpr uint8_t I2C_SLV0_CTRL = 0x05;     // I2C slave 0 control
static constexpr uint8_t I2C_SLV0_DO = 0x06;       // I2C slave 0 data out
static constexpr uint8_t I2C_MST_DELAY_CTRL = 0x02;// I2C master delay control

// AK09916 Magnetometer Constants and Registers
static constexpr uint8_t MAG_I2C_ADDR = 0x0C;      // Magnetometer I2C address
static constexpr uint8_t MAG_DEVICE_ID = 0x09;     // Expected device ID
static constexpr uint8_t MAG_WIA1 = 0x00;          // Who I Am 1
static constexpr uint8_t MAG_WIA2 = 0x01;          // Who I Am 2
static constexpr uint8_t MAG_ST1 = 0x10;           // Status 1
static constexpr uint8_t MAG_HXL = 0x11;           // X-axis data low byte
static constexpr uint8_t MAG_HXH = 0x12;           // X-axis data high byte
static constexpr uint8_t MAG_HYL = 0x13;           // Y-axis data low byte
static constexpr uint8_t MAG_HYH = 0x14;           // Y-axis data high byte
static constexpr uint8_t MAG_HZL = 0x15;           // Z-axis data low byte
static constexpr uint8_t MAG_HZH = 0x16;           // Z-axis data high byte
static constexpr uint8_t MAG_ST2 = 0x18;           // Status 2
static constexpr uint8_t MAG_CNTL2 = 0x31;         // Control 2
static constexpr uint8_t MAG_CNTL3 = 0x32;         // Control 3

// Physical Constants
static constexpr double GRAVITY = 9.81; // m/s^2
static constexpr double DEG_TO_RAD = M_PI / 180.0;

// Calibration data structures
struct CalibrationData {
    struct BiasData {
        double x{0.0}, y{0.0}, z{0.0};
    };
    
    struct ScaleData {
        double x{1.0}, y{1.0}, z{1.0};
    };

    BiasData gyroBias;
    BiasData accelBias;
    BiasData magBias;
    ScaleData accelScale;
    ScaleData magScale;
    bool isCalibrated{false};
};

struct Vector3 {
    double x, y, z;
    
    Vector3(double x_ = 0.0, double y_ = 0.0, double z_ = 0.0) 
        : x(x_), y(y_), z(z_) {}
};

struct AccelData {
    int16_t x, y, z;
};

struct GyroData {
    int16_t x, y, z;
};

struct MagData {
    int16_t x, y, z;
};

struct IMUSensorData {
    AccelData accel;
    GyroData gyro;
    MagData mag;
};

class ICM20948 {
private:
  // I2C communication
  int i2c_file;
  uint8_t current_bank = 0;

  // Scaling factors for sensor data conversion
  static constexpr float ACCEL_SCALE = 4.0f * 9.81f / 32768.0f;   // ±4g range
  static constexpr float GYRO_SCALE = 500.0f / 32768.0f;          // ±500 dps range
  static constexpr float MAG_SCALE = 0.15f;                       // 0.15 μT/LSB

  // Calibration data
  CalibrationData calibration_;
  static constexpr int CALIBRATION_SAMPLES = 1000;  // Number of samples for calibration

  // Print formatting
  int print_header_interval;
  int counter = 0;

  inline void setPrintHeaderInterval(int interval) {
    if (interval <= 0) {
      throw std::invalid_argument("Header interval must be positive");
    }
    print_header_interval = interval;
  }

protected:
  void selectBank(uint8_t bank);
  uint8_t readRegister(uint8_t bank, uint8_t reg);
  void writeRegister(uint8_t bank, uint8_t reg, uint8_t value);
  bool isDataReady();
  
  // Calibration methods
  Vector3 collectSamples(int numSamples);
  void calibrateGyro();
  void calibrateAccel();
  void calibrateMag();

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

  // Magnetometer:
  // Full Scale Range: ±4900 μT
  // Initial Sensitivity Tolerance: ±0.3 μT
  static constexpr double MAG_TOLERANCE_UT = 0.3; // 0.3 microtesla
  static constexpr double MAG_STDEV = MAG_TOLERANCE_UT;
  static constexpr double MAG_VARIANCE = MAG_STDEV * MAG_STDEV;

  ICM20948(int bus = 5);
  virtual ~ICM20948();

  ICM20948(const ICM20948 &) = delete;
  ICM20948 &operator=(const ICM20948 &) = delete;

  void initializeMagnetometer();
  void initializeSensor();
  
  // Calibration methods
  void performCalibration();
  bool loadCalibration(const std::string& filename);
  bool saveCalibration(const std::string& filename) const;
  bool isCalibrated() const { return calibration_.isCalibrated; }

  /**
   The ICM-20948's ADC (Analog-to-Digital Converter) is 16-bit. Each sensor
   reading (accelerometer and gyroscope) outputs a 16-bit value. The data
   registers are organized as high byte (8-bit) and low byte (8-bit) pairs
   */
  IMUSensorData readSensorData();

  float convertAcceleration(int16_t rawValue);
  float convertGyro(int16_t rawValue);
  float convertMagneticField(int16_t rawValue);

  void printSensorData();
};
