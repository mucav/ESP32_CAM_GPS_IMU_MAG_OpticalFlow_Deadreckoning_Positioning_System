/*
 * GPS Library for ESP32
 * 
 * Provides interface for u-blox GPS modules
 * Includes NMEA parsing and position data extraction
 */

#ifndef GPS_H
#define GPS_H

#include <Arduino.h>
#include <HardwareSerial.h>

// GPS data structures
struct GPSPosition {
    double latitude;
    double longitude;
    float altitude;
    uint32_t timestamp;
};

struct GPSVelocity {
    float speed;           // m/s
    float course;          // degrees
    uint32_t timestamp;
};

struct GPSQuality {
    uint8_t satellites;
    float hdop;            // Horizontal dilution of precision
    float vdop;            // Vertical dilution of precision
    float pdop;            // Position dilution of precision
    uint32_t timestamp;
};

struct GPSData {
    GPSPosition position;
    GPSVelocity velocity;
    GPSQuality quality;
    bool valid;
    uint32_t timestamp;
};

// NMEA sentence types
enum NMEASentenceType {
    NMEA_UNKNOWN,
    NMEA_GGA,      // Global Positioning System Fix Data
    NMEA_RMC,      // Recommended Minimum Navigation Information
    NMEA_GSA,      // GPS DOP and Active Satellites
    NMEA_GSV,      // GPS Satellites in View
    NMEA_VTG,      // Course Over Ground and Ground Speed
    NMEA_GLL       // Geographic Position - Latitude/Longitude
};

class GPS {
private:
    HardwareSerial* serial;
    String buffer;
    GPSData data;
    bool initialized;
    
    // NMEA parsing
    void processSentence(const String& sentence);
    String parseField(const String& sentence, uint8_t field);
    bool validateChecksum(const String& sentence);
    void parseGGA(const String& sentence);
    void parseRMC(const String& sentence);
    void parseGSA(const String& sentence);
    void parseVTG(const String& sentence);
    
    // Coordinate conversion
    double parseCoordinate(const String& coord, const String& direction);
    double ddmToDecimal(double ddm, char direction);

public:
    GPS();
    
    // Initialization
    bool begin(HardwareSerial& serialInstance, unsigned long baud = 9600);
    void end();
    
    // Data reading
    bool update();
    GPSData getData();
    GPSPosition getPosition();
    GPSVelocity getVelocity();
    GPSQuality getQuality();
    
    // Utility methods
    bool isConnected();
    bool hasValidFix();
    void clearBuffer();
    
    // Configuration
    void setBaudRate(unsigned long baud);
    void sendUBXCommand(const uint8_t* command, size_t length);
    
    // NMEA output
    String generateGGA();
    String generateRMC();
    String generateGSA();
};

#endif // GPS_H 