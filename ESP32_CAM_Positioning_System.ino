/*
 * ESP32-CAM Positioning System
 * 
 * Hardware:
 * - ESP32-CAM (AI Thinker)
 * - MPU9250 IMU (accelerometer + gyroscope + magnetometer)
 * - BMP280 barometer
 * - u-blox GPS module (Neo-M8N, 9600 baud)
 * - Downward-facing camera for optical flow
 * 
 * Core 1: Camera capture and optical flow processing
 * Core 0: IMU, barometer, GPS, EKF, and NMEA output
 * 
 * Features:
 * - Extended Kalman Filter for sensor fusion
 * - GPS jamming/spoofing detection
 * - NMEA output (GGA and RMC sentences)
 * - Optical flow-based motion estimation
 */

#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_http_server.h"
#include "SPI.h"
#include "Wire.h"
#include "EEPROM.h"
#include <math.h>
#include <HardwareSerial.h>

// Include custom libraries
#include "OpticalFlow.h"
#include "MPU9250.h"
#include "BMP280.h"
#include "GPS.h"
#include "EKF.h"

// Pin definitions for ESP32-CAM AI Thinker
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM     -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// I2C pins for sensors
#define I2C_SDA_PIN       14
#define I2C_SCL_PIN       15

// GPS UART pins
#define GPS_RX_PIN        13
#define GPS_TX_PIN        12

// NMEA output UART pins
#define NMEA_RX_PIN       16
#define NMEA_TX_PIN       17

// Task handles
TaskHandle_t cameraTaskHandle = NULL;
TaskHandle_t sensorTaskHandle = NULL;

// Global variables
volatile bool cameraReady = false;
volatile bool sensorsReady = false;
volatile bool imuConnected = false;
volatile bool barometerConnected = false;
volatile bool gpsConnected = false;

// Camera frame buffer
camera_fb_t *fb = NULL;

// Sensor objects
OpticalFlow opticalFlow;
MPU9250 imu;
BMP280 barometer;
GPS gps;
EKF ekf;

// Data structures
FlowResult opticalFlowData;
IMUData imuData;
BMP280Data barometerData;
GPSData gpsData;
EKFState ekfState;
EKFMeasurement ekfMeasurement;

// GPS jamming detection
struct GPSJammingDetector {
    float lastLat, lastLon;
    float maxJumpDistance;
    uint32_t lastUpdateTime;
    bool jammed;
} gpsJammingDetector;

// Timing variables
unsigned long lastDebugTime = 0;
unsigned long lastNMEATime = 0;
unsigned long lastEKFTime = 0;
const unsigned long DEBUG_INTERVAL = 3000;  // 3 seconds (increased from 1 second)
const unsigned long NMEA_INTERVAL = 100;    // 100ms
const unsigned long EKF_INTERVAL = 20;      // 50Hz

// Function prototypes
void cameraTask(void *parameter);
void sensorTask(void *parameter);
void setupCamera();
void setupSensors();
void setupGPS();
void setupNMEA();
void setupEKF();
void readSensors();
void processOpticalFlow();
void updateEKF();
void detectGPSJamming();
void outputNMEA();
void debugOutput();
void calculateChecksum(const char* sentence, char* checksum);

// Core 1: Camera and Optical Flow Task
void cameraTask(void *parameter) {
    Serial.println("Camera task started on core " + String(xPortGetCoreID()));
    
    while (true) {
        // Capture camera frame
        fb = esp_camera_fb_get();
        if (fb) {
            // Process optical flow
            processOpticalFlow();
            
            // Return frame buffer
            esp_camera_fb_return(fb);
            fb = NULL;
        }
        
        // Much longer delay to prevent DMA overflow
        vTaskDelay(pdMS_TO_TICKS(200)); // 200ms delay
    }
}

// Core 0: Sensors, EKF, and NMEA Task
void sensorTask(void *parameter) {
    Serial.println("Sensor task started on core " + String(xPortGetCoreID()));
    
    while (true) {
        // Read sensors
        readSensors();
        
        // Update EKF
        if (millis() - lastEKFTime >= EKF_INTERVAL) {
            updateEKF();
            lastEKFTime = millis();
        }
        
        // Detect GPS jamming
        detectGPSJamming();
        
        // Output NMEA data
        if (millis() - lastNMEATime >= NMEA_INTERVAL) {
            outputNMEA();
            lastNMEATime = millis();
        }
        
        // Debug output
        if (millis() - lastDebugTime >= DEBUG_INTERVAL) {
            debugOutput();
            lastDebugTime = millis();
        }
        
        // Increase task delay to reduce CPU load
        vTaskDelay(pdMS_TO_TICKS(20)); // 20ms delay instead of 10ms
    }
}

void setup() {
    // Disable brownout detector
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    
    // Initialize serial communication
    Serial.begin(115200);
    Serial.println("==========================================");
    Serial.println("ESP32-CAM Positioning System Starting...");
    Serial.println("==========================================");
    
    // Initialize I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000); // 400kHz
    Serial.println("I2C bus initialized");
    
    // Setup camera
    setupCamera();
    
    // Setup sensors
    setupSensors();
    
    // Setup GPS
    setupGPS();
    
    // Setup NMEA output
    setupNMEA();
    
    // Setup EKF
    setupEKF();
    
    // Initialize GPS jamming detector
    memset(&gpsJammingDetector, 0, sizeof(gpsJammingDetector));
    gpsJammingDetector.maxJumpDistance = 100.0; // 100m max jump
    gpsJammingDetector.jammed = false;
    
    // Create tasks
    xTaskCreatePinnedToCore(
        cameraTask,        // Task function
        "CameraTask",      // Task name
        15000,            // Stack size (increased from 10000)
        NULL,             // Task parameters
        1,                // Task priority
        &cameraTaskHandle, // Task handle
        1                 // Core ID (Core 1)
    );
    
    xTaskCreatePinnedToCore(
        sensorTask,        // Task function
        "SensorTask",      // Task name
        20000,            // Stack size (increased from 15000)
        NULL,             // Task parameters
        1,                // Task priority
        &sensorTaskHandle, // Task handle
        0                 // Core ID (Core 0)
    );
    
    Serial.println("==========================================");
    Serial.println("ESP32-CAM Positioning System Ready!");
    Serial.printf("Camera: %s\n", cameraReady ? "Ready" : "Failed");
    Serial.printf("IMU: %s\n", imuConnected ? "Connected" : "Not Connected");
    Serial.printf("Barometer: %s\n", barometerConnected ? "Connected" : "Not Connected");
    Serial.printf("GPS: %s\n", gpsConnected ? "Connected" : "Not Connected");
    Serial.printf("Optical Flow: %s\n", cameraReady ? "Ready" : "Failed");
    Serial.println("==========================================");
}

void loop() {
    // Main loop is empty - everything is handled by tasks
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void setupCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size = FRAMESIZE_QVGA; // 320x240 - keep QVGA for optical flow
    config.jpeg_quality = 12;
    config.fb_count = 2; // Increase frame buffer count
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY; // Only grab when buffer is empty
    
    // Initialize camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }
    
    Serial.println("Camera initialized successfully");
    cameraReady = true;
}

void setupSensors() {
    Serial.println("Setting up sensors...");
    
    // Check I2C bus first with timeout
    Wire.setTimeOut(1000); // 1 second timeout
    Wire.beginTransmission(0x68); // MPU9250 address
    byte error = Wire.endTransmission();
    if (error == 0) {
        Serial.println("I2C bus is working");
    } else {
        Serial.printf("I2C bus error: %d - Sensors may not be connected\n", error);
        // Don't try to initialize sensors if I2C bus has errors
        imuConnected = false;
        barometerConnected = false;
        sensorsReady = false;
        Serial.println("Skipping sensor initialization due to I2C errors");
        return;
    }
    
    // Initialize MPU9250
    Serial.println("Checking MPU9250...");
    if (imu.begin(Wire)) {
        Serial.println("MPU9250 initialized successfully");
        imuConnected = true;
        
        // Configure MPU9250
        imu.setAccelRange(ACCEL_RANGE_16G);
        imu.setGyroRange(GYRO_RANGE_2000DPS);
        imu.setDLPFConfig(0x03); // 44Hz DLPF
        
        // Calibrate sensors
        Serial.println("Calibrating IMU...");
        imu.calibrateAccelerometer(100);
        imu.calibrateGyroscope(100);
        imu.calibrateMagnetometer(200);
        
    } else {
        Serial.println("MPU9250 not found or initialization failed");
        imuConnected = false;
    }
    
    // Initialize BMP280
    Serial.println("Checking BMP280...");
    if (barometer.begin(Wire)) {
        Serial.println("BMP280 initialized successfully");
        barometerConnected = true;
        
        // Configure BMP280
        barometer.setPressureOversampling(BMP280_OVERSAMP_16X);
        barometer.setTemperatureOversampling(BMP280_OVERSAMP_2X);
        barometer.setFilter(BMP280_FILTER_4);
        barometer.setMode(BMP280_MODE_NORMAL);
        
    } else {
        Serial.println("BMP280 not found or initialization failed");
        barometerConnected = false;
    }
    
    // Initialize optical flow
    if (opticalFlow.begin(320, 240)) {
        Serial.println("Optical flow initialized successfully");
        opticalFlow.setMotionThreshold(30);
        opticalFlow.setMotionAreaLimits(100, 10000);
    } else {
        Serial.println("Optical flow initialization failed");
    }
    
    // Set sensors ready if at least one sensor is connected
    sensorsReady = imuConnected || barometerConnected;
    
    Serial.printf("Sensor Status - IMU: %s, Barometer: %s, Sensors Ready: %s\n", 
                  imuConnected ? "Connected" : "Not Connected",
                  barometerConnected ? "Connected" : "Not Connected",
                  sensorsReady ? "Yes" : "No");
}

void setupGPS() {
    Serial.println("Setting up GPS...");
    
    // Initialize GPS
    if (gps.begin(Serial2, 9600)) {
        Serial.println("GPS initialized successfully");
        gpsConnected = true;
    } else {
        Serial.println("GPS initialization failed - GPS may not be connected");
        gpsConnected = false;
    }
}

void setupNMEA() {
    // Initialize NMEA output UART
    Serial1.begin(9600, SERIAL_8N1, NMEA_RX_PIN, NMEA_TX_PIN);
    Serial.println("NMEA output UART initialized");
}

void setupEKF() {
    // Initialize EKF state
    memset(&ekfState, 0, sizeof(ekfState));
    ekfState.timestamp = millis();
    
    // Initialize EKF
    ekf.initialize(ekfState);
    
    // Configure EKF parameters
    ekf.setProcessNoise(0.1, 0.1, 0.1, 0.01, 0.001); // pos, vel, acc, att, bias
    ekf.setMeasurementNoise(1.0, 0.5, 0.1); // gps, baro, flow
    ekf.setInitialUncertainty(10.0, 1.0, 0.1); // pos, vel, att
    
    Serial.println("EKF initialized successfully");
}

void readSensors() {
    if (!sensorsReady) return;
    
    // Read IMU data
    if (imuConnected && imu.readAll(imuData)) {
        // Data is already in the imuData structure
    }
    
    // Read barometer data
    if (barometerConnected && barometer.readAll(barometerData)) {
        // Data is already in the barometerData structure
    }
    
    // Read GPS data
    if (gpsConnected && gps.update()) {
        gpsData = gps.getData();
    }
}

void processOpticalFlow() {
    if (!cameraReady || !fb) return;
    
    // Only process every few frames to reduce load
    static uint32_t frameCount = 0;
    frameCount++;
    
    if (frameCount % 5 != 0) { // Process every 5th frame instead of 3rd
        return;
    }
    
    // Process optical flow using the library
    FlowResult flowResult = opticalFlow.calculateFlow(fb);
    
    if (flowResult.valid) {
        opticalFlowData.deltaX = flowResult.deltaX;
        opticalFlowData.deltaY = flowResult.deltaY;
        opticalFlowData.confidence = flowResult.confidence;
        opticalFlowData.timestamp = flowResult.timestamp;
    }
}

void updateEKF() {
    static uint32_t lastEKFTime = 0;
    uint32_t currentTime = millis();
    float dt = (currentTime - lastEKFTime) / 1000.0;
    
    if (dt > 0 && dt < 1.0) { // Prevent large time jumps
        // Prepare measurement vector
        memset(&ekfMeasurement, 0, sizeof(ekfMeasurement));
        
        // GPS measurements
        if (gpsConnected && gpsData.valid && !gpsJammingDetector.jammed) {
            ekfMeasurement.gps_x = gpsData.position.longitude * 111320.0; // Convert to meters
            ekfMeasurement.gps_y = gpsData.position.latitude * 110540.0;
            ekfMeasurement.gps_z = gpsData.position.altitude;
            ekfMeasurement.gps_valid = true;
        }
        
        // Barometer measurement
        if (barometerConnected && barometerData.timestamp > 0) {
            ekfMeasurement.baro_z = barometerData.altitude;
            ekfMeasurement.baro_valid = true;
        }
        
        // Optical flow measurements
        if (opticalFlowData.confidence > 0.5) {
            ekfMeasurement.flow_x = opticalFlowData.deltaX;
            ekfMeasurement.flow_y = opticalFlowData.deltaY;
            ekfMeasurement.flow_valid = true;
        }
        
        // Update EKF
        ekf.step(dt, ekfMeasurement);
        
        // Get updated state
        ekfState = ekf.getState();
        
        lastEKFTime = currentTime;
    }
}

void detectGPSJamming() {
    if (!gpsConnected) return;
    
    // Check for sudden position jumps
    if (gpsJammingDetector.lastLat != 0 && gpsJammingDetector.lastLon != 0) {
        float latDiff = gpsData.position.latitude - gpsJammingDetector.lastLat;
        float lonDiff = gpsData.position.longitude - gpsJammingDetector.lastLon;
        float distance = sqrt(latDiff * latDiff + lonDiff * lonDiff) * 111000.0; // Approximate meters
        
        if (distance > gpsJammingDetector.maxJumpDistance) {
            gpsJammingDetector.jammed = true;
            Serial.println("GPS jamming detected - large position jump");
        }
    }
    
    // Check HDOP
    if (gpsConnected && gpsData.quality.hdop > 3.0) {
        gpsJammingDetector.jammed = true;
        Serial.println("GPS jamming detected - high HDOP");
    }
    
    // Update last position
    gpsJammingDetector.lastLat = gpsData.position.latitude;
    gpsJammingDetector.lastLon = gpsData.position.longitude;
    gpsJammingDetector.lastUpdateTime = millis();
    
    // Reset jamming flag after some time
    if (millis() - gpsJammingDetector.lastUpdateTime > 30000) { // 30 seconds
        gpsJammingDetector.jammed = false;
    }
}

void outputNMEA() {
    char checksum[3];
    
    // Generate GGA sentence
    char ggaSentence[128];
    snprintf(ggaSentence, sizeof(ggaSentence), 
             "$GPGGA,%02d%02d%02d.00,%09.4f,N,%010.4f,E,1,%02d,%.1f,%.1f,M,0.0,M,,",
             (int)(millis() / 3600000) % 24,    // Hour
             (int)(millis() / 60000) % 60,      // Minute
             (int)(millis() / 1000) % 60,       // Second
             ekfState.y / 110540.0,             // Latitude
             ekfState.x / 111320.0,             // Longitude
             gpsConnected ? gpsData.quality.satellites : 0,        // Satellites
             gpsConnected ? gpsData.quality.hdop : 99.9,              // HDOP
             ekfState.z);                       // Altitude
    
    calculateChecksum(ggaSentence, checksum);
    String ggaOutput = String(ggaSentence) + "*" + String(checksum) + "\r\n";
    Serial1.print(ggaOutput);
    
    // Generate RMC sentence
    char rmcSentence[128];
    snprintf(rmcSentence, sizeof(rmcSentence),
             "$GPRMC,%02d%02d%02d.00,A,%09.4f,N,%010.4f,E,%.1f,%.1f,%02d%02d%02d,,",
             (int)(millis() / 3600000) % 24,    // Hour
             (int)(millis() / 60000) % 60,      // Minute
             (int)(millis() / 1000) % 60,       // Second
             ekfState.y / 110540.0,             // Latitude
             ekfState.x / 111320.0,             // Longitude
             gpsConnected ? sqrt(ekfState.vx * ekfState.vx + ekfState.vy * ekfState.vy) * 1.944 : 0.0, // Speed in knots
             gpsConnected ? atan2(ekfState.vy, ekfState.vx) * 180.0 / PI : 0.0, // Course
             (int)(millis() / 86400000) % 31 + 1, // Day
             (int)(millis() / 2592000000) % 12 + 1, // Month
             (int)(millis() / 31536000000) % 100 + 2024); // Year
    
    calculateChecksum(rmcSentence, checksum);
    String rmcOutput = String(rmcSentence) + "*" + String(checksum) + "\r\n";
    Serial1.print(rmcOutput);
}

void calculateChecksum(const char* sentence, char* checksum) {
    uint8_t sum = 0;
    for (int i = 1; sentence[i] != '\0' && sentence[i] != '*'; i++) {
        sum ^= sentence[i];
    }
    sprintf(checksum, "%02X", sum);
}

void debugOutput() {
    Serial.println("=== ESP32-CAM Positioning System Debug ===");
    
    // IMU data
    if (imuConnected) {
        Serial.printf("IMU - Accel: %.2f, %.2f, %.2f m/s² | Gyro: %.2f, %.2f, %.2f rad/s\n",
                      imuData.accel.x, imuData.accel.y, imuData.accel.z,
                      imuData.gyro.x, imuData.gyro.y, imuData.gyro.z);
    } else {
        Serial.println("IMU: Not connected");
    }
    
    // Barometer data
    if (barometerConnected) {
        Serial.printf("Barometer - Pressure: %.1f Pa | Altitude: %.1f m | Temp: %.1f°C\n",
                      barometerData.pressure, barometerData.altitude, barometerData.temperature);
    } else {
        Serial.println("Barometer: Not connected");
    }
    
    // GPS data
    if (gpsConnected) {
        Serial.printf("GPS - Lat: %.6f | Lon: %.6f | Alt: %.1f m | Satellites: %d | HDOP: %.1f | Valid: %s\n",
                      gpsData.position.latitude, gpsData.position.longitude, gpsData.position.altitude,
                      gpsData.quality.satellites, gpsData.quality.hdop, gpsData.valid ? "Yes" : "No");
    } else {
        Serial.println("GPS: Not connected");
    }
    
    // Optical flow data
    Serial.printf("Optical Flow - DeltaX: %.3f | DeltaY: %.3f | Confidence: %.2f\n",
                  opticalFlowData.deltaX, opticalFlowData.deltaY, opticalFlowData.confidence);
    
    // EKF state
    Serial.printf("EKF State - Pos: %.2f, %.2f, %.2f m | Vel: %.2f, %.2f, %.2f m/s\n",
                  ekfState.x, ekfState.y, ekfState.z,
                  ekfState.vx, ekfState.vy, ekfState.vz);
    
    Serial.printf("EKF Attitude - Roll: %.2f° | Pitch: %.2f° | Yaw: %.2f°\n",
                  ekfState.roll * 180.0 / PI, ekfState.pitch * 180.0 / PI, ekfState.yaw * 180.0 / PI);
    
    // GPS jamming status
    if (gpsConnected) {
        Serial.printf("GPS Jamming: %s\n", gpsJammingDetector.jammed ? "DETECTED" : "None");
    } else {
        Serial.println("GPS Jamming: Not applicable (GPS not connected)");
    }
    
    // EKF uncertainty
    Serial.printf("EKF Uncertainty - Pos: %.2f m | Vel: %.2f m/s | Att: %.2f°\n",
                  ekf.getPositionUncertainty(), ekf.getVelocityUncertainty(), 
                  ekf.getAttitudeUncertainty() * 180.0 / PI);
    
    Serial.println("==========================================");
} 