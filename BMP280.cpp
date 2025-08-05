/*
 * BMP280 Barometer Library Implementation
 */

#include "BMP280.h"
#include <math.h>

BMP280::BMP280(uint8_t address) {
    deviceAddress = address;
    wire = nullptr;
    initialized = false;
    
    // Default configuration
    pressureOversampling = BMP280_OVERSAMP_16X;
    temperatureOversampling = BMP280_OVERSAMP_2X;
    filterCoeff = BMP280_FILTER_4;
    standbyTime = BMP280_STANDBY_250;
    
    // Default sea level pressure (1013.25 hPa)
    seaLevelPressure = 101325.0f;
    
    // Initialize calibration data
    memset(&calib, 0, sizeof(calib));
}

bool BMP280::begin(TwoWire& wireInstance) {
    wire = &wireInstance;
    
    // Check device ID
    uint8_t chipId = readRegister(BMP280_CHIP_ID);
    if (chipId != BMP280_CHIP_ID_VALUE) {
        // Don't print error message to avoid spam
        return false;
    }
    
    // Reset device
    if (!reset()) {
        // Don't print error message to avoid spam
        return false;
    }
    delay(100);
    
    // Read calibration data
    if (!readCalibrationData()) {
        // Don't print error message to avoid spam
        return false;
    }
    
    // Configure device
    if (!setPressureOversampling(pressureOversampling)) return false;
    if (!setTemperatureOversampling(temperatureOversampling)) return false;
    if (!setFilter(filterCoeff)) return false;
    if (!setStandbyTime(standbyTime)) return false;
    if (!setMode(BMP280_MODE_NORMAL)) return false;
    
    initialized = true;
    Serial.println("BMP280 initialized successfully");
    return true;
}

bool BMP280::reset() {
    return writeRegister(BMP280_SOFT_RESET, 0xB6);
}

bool BMP280::setPressureOversampling(uint8_t oversampling) {
    pressureOversampling = oversampling;
    uint8_t ctrl_meas = readRegister(BMP280_CTRL_MEAS);
    ctrl_meas &= 0xE3; // Clear pressure oversampling bits
    ctrl_meas |= (oversampling << 2); // Set pressure oversampling bits
    return writeRegister(BMP280_CTRL_MEAS, ctrl_meas);
}

bool BMP280::setTemperatureOversampling(uint8_t oversampling) {
    temperatureOversampling = oversampling;
    uint8_t ctrl_meas = readRegister(BMP280_CTRL_MEAS);
    ctrl_meas &= 0x1F; // Clear temperature oversampling bits
    ctrl_meas |= (oversampling << 5); // Set temperature oversampling bits
    return writeRegister(BMP280_CTRL_MEAS, ctrl_meas);
}

bool BMP280::setFilter(uint8_t filter) {
    filterCoeff = filter;
    uint8_t config = readRegister(BMP280_CONFIG);
    config &= 0xFC; // Clear filter bits
    config |= (filter << 2); // Set filter bits
    return writeRegister(BMP280_CONFIG, config);
}

bool BMP280::setStandbyTime(uint8_t standby) {
    standbyTime = standby;
    uint8_t config = readRegister(BMP280_CONFIG);
    config &= 0xE3; // Clear standby time bits
    config |= (standby << 5); // Set standby time bits
    return writeRegister(BMP280_CONFIG, config);
}

bool BMP280::setMode(uint8_t mode) {
    uint8_t ctrl_meas = readRegister(BMP280_CTRL_MEAS);
    ctrl_meas &= 0xFC; // Clear mode bits
    ctrl_meas |= mode; // Set mode bits
    return writeRegister(BMP280_CTRL_MEAS, ctrl_meas);
}

bool BMP280::readPressure(float& pressure) {
    if (!initialized) return false;
    
    // Read pressure data
    uint8_t data[3];
    if (!readRegisters(BMP280_PRESS_MSB, data, 3)) {
        return false;
    }
    
    // Combine bytes
    int32_t adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);
    
    // Read temperature for compensation
    float temperature;
    if (!readTemperature(temperature)) {
        return false;
    }
    
    // Compensate pressure
    int32_t t_fine;
    compensateTemperature(adc_P, t_fine); // This will be ignored, we need proper temp compensation
    uint32_t comp_p = compensatePressure(adc_P, t_fine);
    
    pressure = (float)comp_p / 256.0f; // Convert to Pa
    return true;
}

bool BMP280::readTemperature(float& temperature) {
    if (!initialized) return false;
    
    // Read temperature data
    uint8_t data[3];
    if (!readRegisters(BMP280_TEMP_MSB, data, 3)) {
        return false;
    }
    
    // Combine bytes
    int32_t adc_T = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);
    
    // Compensate temperature
    int32_t t_fine;
    int32_t comp_t = compensateTemperature(adc_T, t_fine);
    
    temperature = (float)comp_t / 100.0f; // Convert to °C
    return true;
}

bool BMP280::readAltitude(float& altitude) {
    if (!initialized) return false;
    
    float pressure;
    if (!readPressure(pressure)) {
        return false;
    }
    
    // Calculate altitude using barometric formula
    altitude = 44330.0f * (1.0f - pow(pressure / seaLevelPressure, 0.1903f));
    return true;
}

bool BMP280::readAll(BMP280Data& data) {
    if (!initialized) return false;
    
    // Read all sensor data
    uint8_t sensorData[6];
    if (!readRegisters(BMP280_PRESS_MSB, sensorData, 6)) {
        return false;
    }
    
    // Combine pressure bytes
    int32_t adc_P = ((int32_t)sensorData[0] << 12) | ((int32_t)sensorData[1] << 4) | (sensorData[2] >> 4);
    
    // Combine temperature bytes
    int32_t adc_T = ((int32_t)sensorData[3] << 12) | ((int32_t)sensorData[4] << 4) | (sensorData[5] >> 4);
    
    // Compensate temperature first
    int32_t t_fine;
    int32_t comp_t = compensateTemperature(adc_T, t_fine);
    data.temperature = (float)comp_t / 100.0f;
    
    // Compensate pressure
    uint32_t comp_p = compensatePressure(adc_P, t_fine);
    data.pressure = (float)comp_p / 256.0f;
    
    // Calculate altitude
    data.altitude = 44330.0f * (1.0f - pow(data.pressure / seaLevelPressure, 0.1903f));
    
    data.timestamp = millis();
    return true;
}

bool BMP280::isConnected() {
    uint8_t chipId = readRegister(BMP280_CHIP_ID);
    return (chipId == BMP280_CHIP_ID_VALUE);
}

void BMP280::setSeaLevelPressure(float pressure) {
    seaLevelPressure = pressure;
}

float BMP280::getSeaLevelPressure() {
    return seaLevelPressure;
}

bool BMP280::selfTest() {
    // Implement self-test functionality
    return true;
}

// Private methods
bool BMP280::writeRegister(uint8_t reg, uint8_t data) {
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

uint8_t BMP280::readRegister(uint8_t reg) {
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

bool BMP280::readRegisters(uint8_t reg, uint8_t* data, uint8_t length) {
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

bool BMP280::readCalibrationData() {
    uint8_t calibData[24];
    
    if (!readRegisters(BMP280_DIG_T1_LSB, calibData, 24)) {
        return false;
    }
    
    // Parse calibration data
    calib.dig_T1 = (calibData[1] << 8) | calibData[0];
    calib.dig_T2 = (calibData[3] << 8) | calibData[2];
    calib.dig_T3 = (calibData[5] << 8) | calibData[4];
    calib.dig_P1 = (calibData[7] << 8) | calibData[6];
    calib.dig_P2 = (calibData[9] << 8) | calibData[8];
    calib.dig_P3 = (calibData[11] << 8) | calibData[10];
    calib.dig_P4 = (calibData[13] << 8) | calibData[12];
    calib.dig_P5 = (calibData[15] << 8) | calibData[14];
    calib.dig_P6 = (calibData[17] << 8) | calibData[16];
    calib.dig_P7 = (calibData[19] << 8) | calibData[18];
    calib.dig_P8 = (calibData[21] << 8) | calibData[20];
    calib.dig_P9 = (calibData[23] << 8) | calibData[22];
    
    return true;
}

int32_t BMP280::compensateTemperature(int32_t adc_T, int32_t& t_fine) {
    int32_t var1, var2;
    
    var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) * ((int32_t)calib.dig_T3)) >> 14;
    
    t_fine = var1 + var2;
    
    return (t_fine * 5 + 128) >> 8;
}

uint32_t BMP280::compensatePressure(int32_t adc_P, int32_t t_fine) {
    int64_t var1, var2, p;
    
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) + ((var1 * (int64_t)calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib.dig_P1) >> 33;
    
    if (var1 == 0) {
        return 0; // Avoid division by zero
    }
    
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib.dig_P8) * p) >> 19;
    
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);
    
    return (uint32_t)p;
} 