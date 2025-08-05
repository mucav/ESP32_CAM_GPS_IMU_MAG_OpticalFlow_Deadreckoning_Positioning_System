/*
 * MPU9250 IMU Library for ESP32
 * 
 * Provides interface for MPU9250 9-axis motion tracking device
 * Includes accelerometer, gyroscope, and magnetometer
 */

#ifndef MPU9250_H
#define MPU9250_H

#include <Arduino.h>
#include <Wire.h>

// MPU9250 Register Addresses
#define MPU9250_ADDR                   0x68
#define MPU9250_WHO_AM_I               0x75
#define MPU9250_PWR_MGMT_1             0x6B
#define MPU9250_PWR_MGMT_2             0x6C
#define MPU9250_CONFIG                 0x1A
#define MPU9250_GYRO_CONFIG            0x1B
#define MPU9250_ACCEL_CONFIG           0x1C
#define MPU9250_ACCEL_CONFIG2          0x1D
#define MPU9250_ACCEL_XOUT_H           0x3B
#define MPU9250_GYRO_XOUT_H            0x43
#define MPU9250_TEMP_OUT_H             0x41
#define MPU9250_MAG_XOUT_L             0x03
#define MPU9250_MAG_CNTL1              0x0A
#define MPU9250_MAG_CNTL2              0x0B
#define MPU9250_MAG_ST1                0x02
#define MPU9250_MAG_ST2                0x09

// AK8963 Magnetometer Address
#define AK8963_ADDR                    0x0C
#define AK8963_WHO_AM_I                0x00
#define AK8963_ST1                     0x02
#define AK8963_XOUT_L                  0x03
#define AK8963_CNTL1                   0x0A
#define AK8963_ASAX                    0x10

// Accelerometer ranges
#define ACCEL_RANGE_2G                 0x00
#define ACCEL_RANGE_4G                 0x08
#define ACCEL_RANGE_8G                 0x10
#define ACCEL_RANGE_16G                0x18

// Gyroscope ranges
#define GYRO_RANGE_250DPS              0x00
#define GYRO_RANGE_500DPS              0x08
#define GYRO_RANGE_1000DPS             0x10
#define GYRO_RANGE_2000DPS             0x18

// Magnetometer modes
#define MAG_MODE_POWER_DOWN            0x00
#define MAG_MODE_SINGLE_MEASURE        0x01
#define MAG_MODE_CONTINUOUS_MEASURE_1  0x02
#define MAG_MODE_CONTINUOUS_MEASURE_2  0x06
#define MAG_MODE_EXTERNAL_TRIGGER      0x04
#define MAG_MODE_SELF_TEST             0x08
#define MAG_MODE_FUSE_ROM_ACCESS       0x0F

// Data structures
struct AccelerometerData {
    float x, y, z;           // m/s²
    float temperature;       // °C
    uint32_t timestamp;
};

struct GyroscopeData {
    float x, y, z;           // rad/s
    uint32_t timestamp;
};

struct MagnetometerData {
    float x, y, z;           // μT
    uint32_t timestamp;
};

struct IMUData {
    AccelerometerData accel;
    GyroscopeData gyro;
    MagnetometerData mag;
    uint32_t timestamp;
};

class MPU9250 {
private:
    uint8_t deviceAddress;
    TwoWire* wire;
    bool initialized;
    
    // Calibration data
    float accelBias[3];
    float gyroBias[3];
    float magBias[3];
    float magScale[3];
    
    // Configuration
    uint8_t accelRange;
    uint8_t gyroRange;
    float accelScale;
    float gyroScale;
    
    // Magnetometer calibration
    float magCalibration[3];
    bool magCalibrated;
    
    // Private methods
    bool writeRegister(uint8_t reg, uint8_t data);
    uint8_t readRegister(uint8_t reg);
    bool readRegisters(uint8_t reg, uint8_t* data, uint8_t length);
    bool initializeMagnetometer();
    void calculateAccelScale();
    void calculateGyroScale();

public:
    MPU9250(uint8_t address = MPU9250_ADDR);
    
    // Initialization
    bool begin(TwoWire& wire = Wire);
    bool reset();
    
    // Configuration
    bool setAccelRange(uint8_t range);
    bool setGyroRange(uint8_t range);
    bool setDLPFConfig(uint8_t config);
    bool setSampleRate(uint8_t rate);
    
    // Data reading
    bool readAccelerometer(AccelerometerData& data);
    bool readGyroscope(GyroscopeData& data);
    bool readMagnetometer(MagnetometerData& data);
    bool readAll(IMUData& data);
    
    // Calibration
    bool calibrateAccelerometer(int samples = 100);
    bool calibrateGyroscope(int samples = 100);
    bool calibrateMagnetometer(int samples = 100);
    
    // Utility methods
    bool isConnected();
    void setAccelBias(float x, float y, float z);
    void setGyroBias(float x, float y, float z);
    void setMagBias(float x, float y, float z);
    void setMagScale(float x, float y, float z);
    
    // Self-test
    bool selfTest();
};

#endif // MPU9250_H 