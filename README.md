# ESP32-CAM Positioning System

A complete Arduino project for an ESP32-CAM based positioning system with sensor fusion, optical flow, and GPS jamming detection.

******There are currently some issues and it is not working efficiently. There is a DMA Overflow error.*****

## Hardware Requirements

- **ESP32-CAM (AI Thinker)** - Main microcontroller with camera
- **MPU9250 IMU** - 9-axis motion sensor (accelerometer, gyroscope, magnetometer)
- **BMP280 Barometer** - Pressure and altitude sensor
- **u-blox GPS module (Neo-M8N)** - GPS receiver (9600 baud)
- **Downward-facing camera** - For optical flow motion detection

## Pin Connections

### ESP32-CAM AI Thinker Pinout

| Component | ESP32-CAM Pin | Description |
|-----------|---------------|-------------|
| Camera | Built-in | OV2640 camera module |
| I2C SDA | GPIO 14 | MPU9250 and BMP280 data line |
| I2C SCL | GPIO 15 | MPU9250 and BMP280 clock line |
| GPS RX | GPIO 13 | GPS module receive |
| GPS TX | GPIO 12 | GPS module transmit |
| NMEA RX | GPIO 16 | NMEA output receive |
| NMEA TX | GPIO 17 | NMEA output transmit |

### Sensor Connections

#### MPU9250 (I2C Address: 0x68)
- VCC → 3.3V
- GND → GND
- SDA → GPIO 14
- SCL → GPIO 15

#### BMP280 (I2C Address: 0x76)
- VCC → 3.3V
- GND → GND
- SDA → GPIO 14
- SCL → GPIO 15

#### u-blox GPS Module
- VCC → 3.3V
- GND → GND
- TX → GPIO 13 (ESP32 RX)
- RX → GPIO 12 (ESP32 TX)

## Features

### Dual-Core Processing
- **Core 1**: Camera capture and optical flow processing
- **Core 0**: IMU, barometer, GPS, EKF, and NMEA output

### Sensor Fusion
- **Extended Kalman Filter (EKF)** with 15-state vector
- Fuses IMU, GPS, barometer, and optical flow data
- Real-time position, velocity, and attitude estimation

### Optical Flow
- Lucas-Kanade optical flow algorithm
- Motion detection and velocity estimation
- Based on esp32-motion project

### GPS Jamming Detection
- HDOP threshold monitoring (HDOP > 3)
- Sudden position jump detection
- Automatic GPS data rejection when jamming detected

### NMEA Output
- UBlox-compatible GGA and RMC sentences
- Output on UART2 (GPIO 16/17)
- External devices can treat ESP32 as GPS receiver

### Sensor Libraries
- **MPU9250**: Full 9-axis IMU support with calibration
- **BMP280**: Pressure, temperature, and altitude
- **GPS**: NMEA parsing and u-blox support
- **OpticalFlow**: Motion detection and flow calculation
- **EKF**: Extended Kalman Filter for sensor fusion

## Installation

1. **Download the project** to your Arduino projects folder
2. **Install ESP32 board support** in Arduino IDE:
   - File → Preferences → Additional Board Manager URLs
   - Add: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
   - Tools → Board → Boards Manager → Search "ESP32" → Install

3. **Select the correct board**:
   - Tools → Board → ESP32 Arduino → AI Thinker ESP32-CAM

4. **Configure upload settings**:
   - Upload Speed: 115200
   - CPU Frequency: 240MHz
   - Flash Frequency: 80MHz
   - Flash Mode: QIO
   - Flash Size: 4MB
   - Partition Scheme: Default 4MB with spiffs

5. **Upload the code** to your ESP32-CAM

## Usage

### Initial Setup
1. Connect all sensors according to the pinout
2. Power the ESP32-CAM
3. Open Serial Monitor at 115200 baud
4. The system will initialize and show debug information

### Calibration
The system includes automatic calibration for:
- **MPU9250 Accelerometer**: Zero bias calibration
- **MPU9250 Gyroscope**: Zero bias calibration
- **MPU9250 Magnetometer**: Hard/soft iron calibration
- **BMP280**: Factory calibration data

### Operation
- The system runs continuously on both cores
- Core 1 processes camera frames for optical flow
- Core 0 handles sensor fusion and NMEA output
- GPS data is used for drift correction when not jammed
- NMEA sentences are output on UART2 for external devices

### Debug Output
The system provides comprehensive debug information:
- IMU data (acceleration, angular velocity)
- Barometer data (pressure, altitude, temperature)
- GPS data (position, satellites, HDOP)
- Optical flow data (motion vectors, confidence)
- EKF state (position, velocity, attitude)
- GPS jamming status

## Configuration

### EKF Parameters
```cpp
// Process noise (adjust based on your application)
ekf.setProcessNoise(0.1, 0.1, 0.1, 0.01, 0.001);

// Measurement noise (adjust based on sensor quality)
ekf.setMeasurementNoise(1.0, 0.5, 0.1);

// Initial uncertainty
ekf.setInitialUncertainty(10.0, 1.0, 0.1);
```

### GPS Jamming Detection
```cpp
// Maximum position jump distance (meters)
gpsJammingDetector.maxJumpDistance = 100.0;

// HDOP threshold
if (gpsData.hdop > 3.0) {
    // GPS jamming detected
}
```

### Optical Flow Parameters
```cpp
// Motion detection threshold
opticalFlow.setMotionThreshold(30);

// Motion area limits
opticalFlow.setMotionAreaLimits(100, 10000);
```

## NMEA Output Format

### GGA Sentence (Global Positioning System Fix Data)
```
$GPGGA,123456.00,3747.1654,N,12202.4490,W,1,08,1.0,10.0,M,0.0,M,,
```

### RMC Sentence (Recommended Minimum Navigation Information)
```
$GPRMC,123456.00,A,3747.1654,N,12202.4490,W,0.5,45.0,010120,,
```

## Troubleshooting

### Common Issues

1. **Camera not initializing**
   - Check power supply (3.3V required)
   - Verify camera module connection
   - Check brownout detector settings

2. **Sensors not detected**
   - Verify I2C connections (SDA/SCL)
   - Check sensor addresses (MPU9250: 0x68, BMP280: 0x76)
   - Ensure proper power supply

3. **GPS not working**
   - Check UART connections (TX/RX)
   - Verify GPS module power
   - Check antenna connection
   - Wait for GPS fix (can take several minutes)

4. **High memory usage**
   - Reduce frame buffer size
   - Optimize optical flow parameters
   - Increase stack sizes for tasks

### Performance Optimization

1. **Reduce camera resolution** for better performance
2. **Adjust task priorities** based on requirements
3. **Optimize EKF update rate** for your application
4. **Use external PSRAM** if available

## Technical Details

### EKF State Vector (15 states)
- Position: x, y, z (meters)
- Velocity: vx, vy, vz (m/s)
- Acceleration: ax, ay, az (m/s²)
- Attitude: roll, pitch, yaw (radians)
- Accelerometer bias: bx, by, bz (m/s²)

### Measurement Vector (6 measurements)
- GPS position: gps_x, gps_y, gps_z (meters)
- Barometer altitude: baro_z (meters)
- Optical flow: flow_x, flow_y (meters)

### Task Priorities
- Camera Task (Core 1): Priority 1
- Sensor Task (Core 0): Priority 1

## License

## License
This project is for individual use only, not commercial use. See the [`LICENSE.md`](LICENSE.md) file for details.

## Contributing

Feel free to submit issues, feature requests, or pull requests to improve the project.

## Acknowledgments

- Based on esp32-motion project by thomas-pegot
- Uses ESP32-CAM library for camera functionality
- Implements Lucas-Kanade optical flow algorithm

- Extended Kalman Filter implementation for sensor fusion 

