/*
 * BMP280 Barometer Library for ESP32
 * 
 * Provides interface for BMP280 pressure sensor
 * Includes pressure, temperature, and altitude calculation
 */

#ifndef BMP280_H
#define BMP280_H

#include <Arduino.h>
#include <Wire.h>

// BMP280 Register Addresses
#define BMP280_ADDR                    0x76
#define BMP280_ADDR_ALT                0x77
#define BMP280_CHIP_ID                 0xD0
#define BMP280_VERSION                 0xD1
#define BMP280_SOFT_RESET              0xE0
#define BMP280_STATUS                  0xF3
#define BMP280_CTRL_MEAS               0xF4
#define BMP280_CONFIG                  0xF5
#define BMP280_PRESS_MSB               0xF7
#define BMP280_PRESS_LSB               0xF8
#define BMP280_PRESS_XLSB              0xF9
#define BMP280_TEMP_MSB                0xFA
#define BMP280_TEMP_LSB                0xFB
#define BMP280_TEMP_XLSB               0xFC
#define BMP280_DIG_T1_LSB              0x88
#define BMP280_DIG_T1_MSB              0x89
#define BMP280_DIG_T2_LSB              0x8A
#define BMP280_DIG_T2_MSB              0x8B
#define BMP280_DIG_T3_LSB              0x8C
#define BMP280_DIG_T3_MSB              0x8D
#define BMP280_DIG_P1_LSB              0x8E
#define BMP280_DIG_P1_MSB              0x8F
#define BMP280_DIG_P2_LSB              0x90
#define BMP280_DIG_P2_MSB              0x91
#define BMP280_DIG_P3_LSB              0x92
#define BMP280_DIG_P3_MSB              0x93
#define BMP280_DIG_P4_LSB              0x94
#define BMP280_DIG_P4_MSB              0x95
#define BMP280_DIG_P5_LSB              0x96
#define BMP280_DIG_P5_MSB              0x97
#define BMP280_DIG_P6_LSB              0x98
#define BMP280_DIG_P6_MSB              0x99
#define BMP280_DIG_P7_LSB              0x9A
#define BMP280_DIG_P7_MSB              0x9B
#define BMP280_DIG_P8_LSB              0x9C
#define BMP280_DIG_P8_MSB              0x9D
#define BMP280_DIG_P9_LSB              0x9E
#define BMP280_DIG_P9_MSB              0x9F

// BMP280 Chip ID
#define BMP280_CHIP_ID_VALUE           0x58

// Operating modes
#define BMP280_MODE_SLEEP              0x00
#define BMP280_MODE_FORCED             0x01
#define BMP280_MODE_NORMAL             0x03

// Oversampling settings
#define BMP280_OVERSAMP_SKIPPED        0x00
#define BMP280_OVERSAMP_1X             0x01
#define BMP280_OVERSAMP_2X             0x02
#define BMP280_OVERSAMP_4X             0x03
#define BMP280_OVERSAMP_8X             0x04
#define BMP280_OVERSAMP_16X            0x05

// Filter settings
#define BMP280_FILTER_OFF              0x00
#define BMP280_FILTER_2                0x01
#define BMP280_FILTER_4                0x02
#define BMP280_FILTER_8                0x03
#define BMP280_FILTER_16               0x04

// Standby time settings
#define BMP280_STANDBY_0_5             0x00
#define BMP280_STANDBY_62_5            0x01
#define BMP280_STANDBY_125             0x02
#define BMP280_STANDBY_250             0x03
#define BMP280_STANDBY_500             0x04
#define BMP280_STANDBY_1000            0x05
#define BMP280_STANDBY_2000            0x06
#define BMP280_STANDBY_4000            0x07

// Data structures
struct BMP280Data {
    float pressure;        // Pa
    float temperature;     // °C
    float altitude;        // m
    uint32_t timestamp;
};

struct BMP280Calibration {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
};

class BMP280 {
private:
    uint8_t deviceAddress;
    TwoWire* wire;
    BMP280Calibration calib;
    bool initialized;
    
    // Configuration
    uint8_t pressureOversampling;
    uint8_t temperatureOversampling;
    uint8_t filterCoeff;
    uint8_t standbyTime;
    
    // Sea level pressure for altitude calculation
    float seaLevelPressure;
    
    // Private methods
    bool writeRegister(uint8_t reg, uint8_t data);
    uint8_t readRegister(uint8_t reg);
    bool readRegisters(uint8_t reg, uint8_t* data, uint8_t length);
    bool readCalibrationData();
    int32_t compensateTemperature(int32_t adc_T, int32_t& t_fine);
    uint32_t compensatePressure(int32_t adc_P, int32_t t_fine);

public:
    BMP280(uint8_t address = BMP280_ADDR);
    
    // Initialization
    bool begin(TwoWire& wire = Wire);
    bool reset();
    
    // Configuration
    bool setPressureOversampling(uint8_t oversampling);
    bool setTemperatureOversampling(uint8_t oversampling);
    bool setFilter(uint8_t filter);
    bool setStandbyTime(uint8_t standby);
    bool setMode(uint8_t mode);
    
    // Data reading
    bool readPressure(float& pressure);
    bool readTemperature(float& temperature);
    bool readAltitude(float& altitude);
    bool readAll(BMP280Data& data);
    
    // Utility methods
    bool isConnected();
    void setSeaLevelPressure(float pressure);
    float getSeaLevelPressure();
    
    // Self-test
    bool selfTest();
};

#endif // BMP280_H 