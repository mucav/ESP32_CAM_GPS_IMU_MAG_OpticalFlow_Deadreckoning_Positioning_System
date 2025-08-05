/*
 * GPS Library Implementation
 */

#include "GPS.h"
#include <math.h>

GPS::GPS() {
    serial = nullptr;
    initialized = false;
    buffer = "";
    
    // Initialize data structure
    memset(&data, 0, sizeof(data));
    data.valid = false;
}

bool GPS::begin(HardwareSerial& serialInstance, unsigned long baud) {
    serial = &serialInstance;
    serial->begin(baud);
    initialized = true;
    clearBuffer();
    
    Serial.printf("GPS initialized at %lu baud\n", baud);
    return true;
}

void GPS::end() {
    if (initialized) {
        serial->end();
        initialized = false;
    }
}

bool GPS::update() {
    if (!initialized) return false;
    
    // Read available data
    while (serial->available()) {
        char c = serial->read();
        
        if (c == '\n') {
            // Process complete sentence
            if (buffer.length() > 0) {
                processSentence(buffer);
                buffer = "";
            }
        } else if (c == '\r') {
            // Ignore carriage return
        } else {
            buffer += c;
        }
    }
    
    return data.valid;
}

GPSData GPS::getData() {
    return data;
}

GPSPosition GPS::getPosition() {
    return data.position;
}

GPSVelocity GPS::getVelocity() {
    return data.velocity;
}

GPSQuality GPS::getQuality() {
    return data.quality;
}

bool GPS::isConnected() {
    return initialized && serial->available() >= 0;
}

bool GPS::hasValidFix() {
    return data.valid && data.quality.satellites >= 4;
}

void GPS::clearBuffer() {
    buffer = "";
    while (serial->available()) {
        serial->read();
    }
}

void GPS::setBaudRate(unsigned long baud) {
    if (initialized) {
        serial->updateBaudRate(baud);
    }
}

void GPS::sendUBXCommand(const uint8_t* command, size_t length) {
    if (initialized) {
        serial->write(command, length);
    }
}

String GPS::generateGGA() {
    char gga[128];
    snprintf(gga, sizeof(gga), 
             "$GPGGA,%02d%02d%02d.00,%09.4f,%c,%010.4f,%c,1,%02d,%.1f,%.1f,M,0.0,M,,",
             (int)(millis() / 3600000) % 24,    // Hour
             (int)(millis() / 60000) % 60,      // Minute
             (int)(millis() / 1000) % 60,       // Second
             abs(data.position.latitude),       // Latitude
             data.position.latitude >= 0 ? 'N' : 'S',
             abs(data.position.longitude),      // Longitude
             data.position.longitude >= 0 ? 'E' : 'W',
             data.quality.satellites,           // Satellites
             data.quality.hdop,                 // HDOP
             data.position.altitude);           // Altitude
    
    // Calculate checksum
    uint8_t checksum = 0;
    for (int i = 1; gga[i] != '\0' && gga[i] != '*'; i++) {
        checksum ^= gga[i];
    }
    
    char result[150];
    snprintf(result, sizeof(result), "%s*%02X\r\n", gga, checksum);
    return String(result);
}

String GPS::generateRMC() {
    char rmc[128];
    snprintf(rmc, sizeof(rmc),
             "$GPRMC,%02d%02d%02d.00,A,%09.4f,%c,%010.4f,%c,%.1f,%.1f,%02d%02d%02d,,",
             (int)(millis() / 3600000) % 24,    // Hour
             (int)(millis() / 60000) % 60,      // Minute
             (int)(millis() / 1000) % 60,       // Second
             abs(data.position.latitude),       // Latitude
             data.position.latitude >= 0 ? 'N' : 'S',
             abs(data.position.longitude),      // Longitude
             data.position.longitude >= 0 ? 'E' : 'W',
             data.velocity.speed * 1.944,       // Speed in knots
             data.velocity.course,              // Course
             (int)(millis() / 86400000) % 31 + 1, // Day
             (int)(millis() / 2592000000) % 12 + 1, // Month
             (int)(millis() / 31536000000) % 100 + 2024); // Year
    
    // Calculate checksum
    uint8_t checksum = 0;
    for (int i = 1; rmc[i] != '\0' && rmc[i] != '*'; i++) {
        checksum ^= rmc[i];
    }
    
    char result[150];
    snprintf(result, sizeof(result), "%s*%02X\r\n", rmc, checksum);
    return String(result);
}

String GPS::generateGSA() {
    char gsa[128];
    snprintf(gsa, sizeof(gsa),
             "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,%.1f,%.1f,%.1f",
             data.quality.pdop, data.quality.hdop, data.quality.vdop);
    
    // Calculate checksum
    uint8_t checksum = 0;
    for (int i = 1; gsa[i] != '\0' && gsa[i] != '*'; i++) {
        checksum ^= gsa[i];
    }
    
    char result[150];
    snprintf(result, sizeof(result), "%s*%02X\r\n", gsa, checksum);
    return String(result);
}

// Private methods
void GPS::processSentence(const String& sentence) {
    if (sentence.length() < 6) return;
    
    // Check if sentence starts with $
    if (sentence[0] != '$') return;
    
    // Validate checksum
    if (!validateChecksum(sentence)) {
        Serial.println("GPS checksum error");
        return;
    }
    
    // Parse based on sentence type
    if (sentence.startsWith("$GPGGA") || sentence.startsWith("$GNGGA")) {
        parseGGA(sentence);
    } else if (sentence.startsWith("$GPRMC") || sentence.startsWith("$GNRMC")) {
        parseRMC(sentence);
    } else if (sentence.startsWith("$GPGSA") || sentence.startsWith("$GNGSA")) {
        parseGSA(sentence);
    } else if (sentence.startsWith("$GPVTG") || sentence.startsWith("$GNVTG")) {
        parseVTG(sentence);
    }
}

String GPS::parseField(const String& sentence, uint8_t field) {
    int start = 0;
    int end = 0;
    int currentField = 0;
    
    for (int i = 0; i < sentence.length(); i++) {
        if (sentence[i] == ',') {
            currentField++;
            if (currentField == field) {
                start = i + 1;
            } else if (currentField == field + 1) {
                end = i;
                break;
            }
        }
    }
    
    if (currentField == field && start < sentence.length()) {
        end = sentence.length();
        // Remove checksum if present
        int asterisk = sentence.indexOf('*', start);
        if (asterisk > start) {
            end = asterisk;
        }
    }
    
    if (start < end && end <= sentence.length()) {
        return sentence.substring(start, end);
    }
    
    return "";
}

bool GPS::validateChecksum(const String& sentence) {
    int asterisk = sentence.indexOf('*');
    if (asterisk == -1) return false;
    
    String data = sentence.substring(1, asterisk);
    String checksumStr = sentence.substring(asterisk + 1);
    
    uint8_t calculatedChecksum = 0;
    for (int i = 0; i < data.length(); i++) {
        calculatedChecksum ^= data[i];
    }
    
    uint8_t receivedChecksum = strtol(checksumStr.c_str(), NULL, 16);
    return calculatedChecksum == receivedChecksum;
}

void GPS::parseGGA(const String& sentence) {
    // Parse GGA sentence
    String timeStr = parseField(sentence, 1);
    String latStr = parseField(sentence, 2);
    String latDir = parseField(sentence, 3);
    String lonStr = parseField(sentence, 4);
    String lonDir = parseField(sentence, 5);
    String fixStr = parseField(sentence, 6);
    String satStr = parseField(sentence, 7);
    String hdopStr = parseField(sentence, 8);
    String altStr = parseField(sentence, 9);
    
    if (latStr.length() > 0 && lonStr.length() > 0) {
        data.position.latitude = parseCoordinate(latStr, latDir);
        data.position.longitude = parseCoordinate(lonStr, lonDir);
        
        if (altStr.length() > 0) {
            data.position.altitude = altStr.toFloat();
        }
        
        if (satStr.length() > 0) {
            data.quality.satellites = satStr.toInt();
        }
        
        if (hdopStr.length() > 0) {
            data.quality.hdop = hdopStr.toFloat();
        }
        
        data.position.timestamp = millis();
        data.quality.timestamp = millis();
        data.valid = (fixStr.toInt() > 0);
    }
}

void GPS::parseRMC(const String& sentence) {
    // Parse RMC sentence
    String timeStr = parseField(sentence, 1);
    String status = parseField(sentence, 2);
    String latStr = parseField(sentence, 3);
    String latDir = parseField(sentence, 4);
    String lonStr = parseField(sentence, 5);
    String lonDir = parseField(sentence, 6);
    String speedStr = parseField(sentence, 7);
    String courseStr = parseField(sentence, 8);
    
    if (status == "A" && latStr.length() > 0 && lonStr.length() > 0) {
        data.position.latitude = parseCoordinate(latStr, latDir);
        data.position.longitude = parseCoordinate(lonStr, lonDir);
        
        if (speedStr.length() > 0) {
            data.velocity.speed = speedStr.toFloat() * 0.514444; // Convert knots to m/s
        }
        
        if (courseStr.length() > 0) {
            data.velocity.course = courseStr.toFloat();
        }
        
        data.position.timestamp = millis();
        data.velocity.timestamp = millis();
        data.valid = true;
    }
}

void GPS::parseGSA(const String& sentence) {
    // Parse GSA sentence
    String pdopStr = parseField(sentence, 15);
    String hdopStr = parseField(sentence, 16);
    String vdopStr = parseField(sentence, 17);
    
    if (pdopStr.length() > 0) {
        data.quality.pdop = pdopStr.toFloat();
    }
    
    if (hdopStr.length() > 0) {
        data.quality.hdop = hdopStr.toFloat();
    }
    
    if (vdopStr.length() > 0) {
        data.quality.vdop = vdopStr.toFloat();
    }
    
    data.quality.timestamp = millis();
}

void GPS::parseVTG(const String& sentence) {
    // Parse VTG sentence
    String courseStr = parseField(sentence, 1);
    String speedStr = parseField(sentence, 5);
    
    if (courseStr.length() > 0) {
        data.velocity.course = courseStr.toFloat();
    }
    
    if (speedStr.length() > 0) {
        data.velocity.speed = speedStr.toFloat() * 0.277778; // Convert km/h to m/s
    }
    
    data.velocity.timestamp = millis();
}

double GPS::parseCoordinate(const String& coord, const String& direction) {
    if (coord.length() == 0) return 0.0;
    
    double value = coord.toDouble();
    double degrees = floor(value / 100.0);
    double minutes = value - (degrees * 100.0);
    
    double decimal = degrees + (minutes / 60.0);
    
    if (direction == "S" || direction == "W") {
        decimal = -decimal;
    }
    
    return decimal;
}

double GPS::ddmToDecimal(double ddm, char direction) {
    double degrees = floor(ddm / 100.0);
    double minutes = ddm - (degrees * 100.0);
    
    double decimal = degrees + (minutes / 60.0);
    
    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }
    
    return decimal;
} 