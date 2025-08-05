/*
 * MPU9250 IMU Library Implementation
 */

#include "MPU9250.h"
#include <math.h>

MPU9250::MPU9250(uint8_t address) {
    deviceAddress = address;
    wire = nullptr;
    initialized = false;
    
    // Initialize calibration data
    for (int i = 0; i < 3; i++) {
        accelBias[i] = 0.0f;
        gyroBias[i] = 0.0f;
        magBias[i] = 0.0f;
        magScale[i] = 1.0f;
        magCalibration[i] = 0.0f;
    }
    
    accelRange = ACCEL_RANGE_2G;
    gyroRange = GYRO_RANGE_250DPS;
    accelScale = 16384.0f; // 2G default
    gyroScale = 131.0f;    // 250DPS default
    magCalibrated = false;
}

bool MPU9250::begin(TwoWire& wireInstance) {
    wire = &wireInstance;
    
    // Reset device
    if (!reset()) {
        // Don't print error message to avoid spam
        return false;
    }
    delay(100);
    
    // Check device ID
    uint8_t whoAmI = readRegister(MPU9250_WHO_AM_I);
    if (whoAmI != 0x71) {
        // Don't print error message to avoid spam
        return false;
    }
    
    // Wake up device
    if (!writeRegister(MPU9250_PWR_MGMT_1, 0x00)) {
        // Don't print error message to avoid spam
        return false;
    }
    
    // Configure accelerometer and gyroscope
    if (!setAccelRange(ACCEL_RANGE_16G)) return false;
    if (!setGyroRange(GYRO_RANGE_2000DPS)) return false;
    if (!setDLPFConfig(0x03)) return false; // 44Hz DLPF
    
    // Initialize magnetometer
    if (!initializeMagnetometer()) {
        // Don't print error message to avoid spam
        return false;
    }
    
    initialized = true;
    Serial.println("MPU9250 initialized successfully");
    return true;
}

bool MPU9250::reset() {
    return writeRegister(MPU9250_PWR_MGMT_1, 0x80);
}

bool MPU9250::setAccelRange(uint8_t range) {
    accelRange = range;
    calculateAccelScale();
    return writeRegister(MPU9250_ACCEL_CONFIG, range);
}

bool MPU9250::setGyroRange(uint8_t range) {
    gyroRange = range;
    calculateGyroScale();
    return writeRegister(MPU9250_GYRO_CONFIG, range);
}

bool MPU9250::setDLPFConfig(uint8_t config) {
    return writeRegister(MPU9250_CONFIG, config);
}

bool MPU9250::setSampleRate(uint8_t rate) {
    return writeRegister(0x19, rate);
}

bool MPU9250::readAccelerometer(AccelerometerData& data) {
    if (!initialized) return false;
    
    uint8_t buffer[6];
    if (!readRegisters(MPU9250_ACCEL_XOUT_H, buffer, 6)) {
        return false;
    }
    
    // Convert raw data to m/s²
    int16_t rawX = (buffer[0] << 8) | buffer[1];
    int16_t rawY = (buffer[2] << 8) | buffer[3];
    int16_t rawZ = (buffer[4] << 8) | buffer[5];
    
    data.x = (rawX / accelScale) * 9.81f - accelBias[0];
    data.y = (rawY / accelScale) * 9.81f - accelBias[1];
    data.z = (rawZ / accelScale) * 9.81f - accelBias[2];
    
    data.timestamp = millis();
    return true;
}

bool MPU9250::readGyroscope(GyroscopeData& data) {
    if (!initialized) return false;
    
    uint8_t buffer[6];
    if (!readRegisters(MPU9250_GYRO_XOUT_H, buffer, 6)) {
        return false;
    }
    
    // Convert raw data to rad/s
    int16_t rawX = (buffer[0] << 8) | buffer[1];
    int16_t rawY = (buffer[2] << 8) | buffer[3];
    int16_t rawZ = (buffer[4] << 8) | buffer[5];
    
    data.x = (rawX / gyroScale) * (PI / 180.0f) - gyroBias[0];
    data.y = (rawY / gyroScale) * (PI / 180.0f) - gyroBias[1];
    data.z = (rawZ / gyroScale) * (PI / 180.0f) - gyroBias[2];
    
    data.timestamp = millis();
    return true;
}

bool MPU9250::readMagnetometer(MagnetometerData& data) {
    if (!initialized) return false;
    
    // Check if magnetometer data is ready
    uint8_t st1 = readRegister(MPU9250_MAG_ST1);
    if (!(st1 & 0x01)) {
        return false; // Data not ready
    }
    
    uint8_t buffer[7];
    if (!readRegisters(MPU9250_MAG_XOUT_L, buffer, 7)) {
        return false;
    }
    
    // Check for overflow
    if (buffer[6] & 0x08) {
        return false; // Magnetic sensor overflow
    }
    
    // Convert raw data to μT
    int16_t rawX = (buffer[1] << 8) | buffer[0];
    int16_t rawY = (buffer[3] << 8) | buffer[2];
    int16_t rawZ = (buffer[5] << 8) | buffer[4];
    
    data.x = (rawX * magCalibration[0] - magBias[0]) * magScale[0];
    data.y = (rawY * magCalibration[1] - magBias[1]) * magScale[1];
    data.z = (rawZ * magCalibration[2] - magBias[2]) * magScale[2];
    
    data.timestamp = millis();
    return true;
}

bool MPU9250::readAll(IMUData& data) {
    bool success = true;
    success &= readAccelerometer(data.accel);
    success &= readGyroscope(data.gyro);
    success &= readMagnetometer(data.mag);
    
    data.timestamp = millis();
    return success;
}

bool MPU9250::calibrateAccelerometer(int samples) {
    Serial.println("Calibrating accelerometer...");
    
    float sumX = 0, sumY = 0, sumZ = 0;
    AccelerometerData tempData;
    
    for (int i = 0; i < samples; i++) {
        if (readAccelerometer(tempData)) {
            sumX += tempData.x;
            sumY += tempData.y;
            sumZ += tempData.z;
        }
        delay(10);
    }
    
    accelBias[0] = sumX / samples;
    accelBias[1] = sumY / samples;
    accelBias[2] = (sumZ / samples) - 9.81f; // Remove gravity
    
    Serial.printf("Accelerometer bias: %.3f, %.3f, %.3f\n", 
                  accelBias[0], accelBias[1], accelBias[2]);
    return true;
}

bool MPU9250::calibrateGyroscope(int samples) {
    Serial.println("Calibrating gyroscope...");
    
    float sumX = 0, sumY = 0, sumZ = 0;
    GyroscopeData tempData;
    
    for (int i = 0; i < samples; i++) {
        if (readGyroscope(tempData)) {
            sumX += tempData.x;
            sumY += tempData.y;
            sumZ += tempData.z;
        }
        delay(10);
    }
    
    gyroBias[0] = sumX / samples;
    gyroBias[1] = sumY / samples;
    gyroBias[2] = sumZ / samples;
    
    Serial.printf("Gyroscope bias: %.6f, %.6f, %.6f\n", 
                  gyroBias[0], gyroBias[1], gyroBias[2]);
    return true;
}

bool MPU9250::calibrateMagnetometer(int samples) {
    Serial.println("Calibrating magnetometer...");
    
    float minX = 10000, maxX = -10000;
    float minY = 10000, maxY = -10000;
    float minZ = 10000, maxZ = -10000;
    
    MagnetometerData tempData;
    
    Serial.println("Please rotate the device in all directions...");
    
    for (int i = 0; i < samples; i++) {
        if (readMagnetometer(tempData)) {
            if (tempData.x < minX) minX = tempData.x;
            if (tempData.x > maxX) maxX = tempData.x;
            if (tempData.y < minY) minY = tempData.y;
            if (tempData.y > maxY) maxY = tempData.y;
            if (tempData.z < minZ) minZ = tempData.z;
            if (tempData.z > maxZ) maxZ = tempData.z;
        }
        delay(50);
    }
    
    // Calculate bias and scale
    magBias[0] = (maxX + minX) / 2.0f;
    magBias[1] = (maxY + minY) / 2.0f;
    magBias[2] = (maxZ + minZ) / 2.0f;
    
    float avgDelta = ((maxX - minX) + (maxY - minY) + (maxZ - minZ)) / 3.0f;
    magScale[0] = avgDelta / (maxX - minX);
    magScale[1] = avgDelta / (maxY - minY);
    magScale[2] = avgDelta / (maxZ - minZ);
    
    magCalibrated = true;
    
    Serial.printf("Magnetometer bias: %.2f, %.2f, %.2f\n", 
                  magBias[0], magBias[1], magBias[2]);
    Serial.printf("Magnetometer scale: %.2f, %.2f, %.2f\n", 
                  magScale[0], magScale[1], magScale[2]);
    return true;
}

bool MPU9250::isConnected() {
    uint8_t whoAmI = readRegister(MPU9250_WHO_AM_I);
    return (whoAmI == 0x71);
}

void MPU9250::setAccelBias(float x, float y, float z) {
    accelBias[0] = x;
    accelBias[1] = y;
    accelBias[2] = z;
}

void MPU9250::setGyroBias(float x, float y, float z) {
    gyroBias[0] = x;
    gyroBias[1] = y;
    gyroBias[2] = z;
}

void MPU9250::setMagBias(float x, float y, float z) {
    magBias[0] = x;
    magBias[1] = y;
    magBias[2] = z;
}

void MPU9250::setMagScale(float x, float y, float z) {
    magScale[0] = x;
    magScale[1] = y;
    magScale[2] = z;
}

bool MPU9250::selfTest() {
    // Implement self-test functionality
    return true;
}

// Private methods
bool MPU9250::writeRegister(uint8_t reg, uint8_t data) {
    if (!wire) return false;
    
    wire->beginTransmission(deviceAddress);
    wire->write(reg);
    wire->write(data);
    byte error = wire->endTransmission();
    
    if (error != 0) {
        // Don't print error messages to avoid spam
        return false;
    }
    return true;
}

uint8_t MPU9250::readRegister(uint8_t reg) {
    if (!wire) return 0;
    
    wire->beginTransmission(deviceAddress);
    wire->write(reg);
    byte error = wire->endTransmission(false);
    
    if (error != 0) {
        return 0;
    }
    
    if (wire->requestFrom(deviceAddress, (uint8_t)1) != 1) {
        return 0;
    }
    
    if (wire->available()) {
        return wire->read();
    }
    return 0;
}

bool MPU9250::readRegisters(uint8_t reg, uint8_t* data, uint8_t length) {
    if (!wire) return false;
    
    wire->beginTransmission(deviceAddress);
    wire->write(reg);
    byte error = wire->endTransmission(false);
    
    if (error != 0) {
        return false;
    }
    
    if (wire->requestFrom(deviceAddress, length) != length) {
        return false;
    }
    
    if (wire->available() >= length) {
        for (uint8_t i = 0; i < length; i++) {
            data[i] = wire->read();
        }
        return true;
    }
    return false;
}

bool MPU9250::initializeMagnetometer() {
    // Enable I2C bypass to access magnetometer
    if (!writeRegister(0x37, 0x02)) return false;
    if (!writeRegister(0x6A, 0x00)) return false;
    
    delay(10);
    
    // Check magnetometer WHO_AM_I
    wire->beginTransmission(AK8963_ADDR);
    wire->write(AK8963_WHO_AM_I);
    wire->endTransmission(false);
    wire->requestFrom(AK8963_ADDR, (uint8_t)1);
    
    if (wire->available()) {
        uint8_t whoAmI = wire->read();
        if (whoAmI != 0x48) {
            Serial.printf("Magnetometer WHO_AM_I failed: 0x%02X\n", whoAmI);
            return false;
        }
    }
    
    // Read magnetometer calibration data
    uint8_t asa[3];
    wire->beginTransmission(AK8963_ADDR);
    wire->write(AK8963_ASAX);
    wire->endTransmission(false);
    wire->requestFrom(AK8963_ADDR, (uint8_t)3);
    
    if (wire->available() >= 3) {
        for (int i = 0; i < 3; i++) {
            asa[i] = wire->read();
        }
        
        // Calculate calibration values
        magCalibration[0] = (asa[0] - 128) / 256.0f + 1.0f;
        magCalibration[1] = (asa[1] - 128) / 256.0f + 1.0f;
        magCalibration[2] = (asa[2] - 128) / 256.0f + 1.0f;
    }
    
    // Set magnetometer to continuous measurement mode
    wire->beginTransmission(AK8963_ADDR);
    wire->write(AK8963_CNTL1);
    wire->write(MAG_MODE_CONTINUOUS_MEASURE_2);
    wire->endTransmission();
    
    delay(10);
    
    return true;
}

void MPU9250::calculateAccelScale() {
    switch (accelRange) {
        case ACCEL_RANGE_2G:
            accelScale = 16384.0f;
            break;
        case ACCEL_RANGE_4G:
            accelScale = 8192.0f;
            break;
        case ACCEL_RANGE_8G:
            accelScale = 4096.0f;
            break;
        case ACCEL_RANGE_16G:
            accelScale = 2048.0f;
            break;
    }
}

void MPU9250::calculateGyroScale() {
    switch (gyroRange) {
        case GYRO_RANGE_250DPS:
            gyroScale = 131.0f;
            break;
        case GYRO_RANGE_500DPS:
            gyroScale = 65.5f;
            break;
        case GYRO_RANGE_1000DPS:
            gyroScale = 32.8f;
            break;
        case GYRO_RANGE_2000DPS:
            gyroScale = 16.4f;
            break;
    }
} 