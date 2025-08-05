/*
 * Optical Flow Library for ESP32-CAM
 * Based on esp32-motion project
 * 
 * Provides motion detection and optical flow calculation
 * for downward-facing camera applications
 */

#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H

#include <Arduino.h>
#include "esp_camera.h"

// Optical flow configuration
#define FLOW_WINDOW_SIZE 32  // Increased from 16 to reduce processing
#define FLOW_SEARCH_SIZE 8
#define FLOW_THRESHOLD 10
#define FLOW_MAX_DISPLACEMENT 32

// Motion detection configuration
#define MOTION_THRESHOLD 30
#define MOTION_MIN_AREA 100
#define MOTION_MAX_AREA 10000

// Structure for optical flow result
struct FlowResult {
    float deltaX;
    float deltaY;
    float confidence;
    uint32_t timestamp;
    bool valid;
};

// Structure for motion detection result
struct MotionResult {
    bool motionDetected;
    uint16_t motionArea;
    uint16_t motionCenterX;
    uint16_t motionCenterY;
    uint32_t timestamp;
};

class OpticalFlow {
private:
    uint8_t* prevFrame;
    uint8_t* currFrame;
    uint16_t frameWidth;
    uint16_t frameHeight;
    uint32_t frameSize;
    bool initialized;
    
    // Lucas-Kanade optical flow parameters
    float lambda;
    uint8_t maxIterations;
    
    // Motion detection parameters
    uint8_t motionThreshold;
    uint16_t minMotionArea;
    uint16_t maxMotionArea;

public:
    OpticalFlow();
    ~OpticalFlow();
    
    // Initialize optical flow with frame dimensions
    bool begin(uint16_t width, uint16_t height);
    
    // Process new frame and calculate optical flow
    FlowResult calculateFlow(camera_fb_t* frame);
    
    // Detect motion in the frame
    MotionResult detectMotion(camera_fb_t* frame);
    
    // Get frame difference for debugging
    void getFrameDifference(uint8_t* diffBuffer);
    
    // Configuration methods
    void setFlowThreshold(uint8_t threshold);
    void setMotionThreshold(uint8_t threshold);
    void setMotionAreaLimits(uint16_t minArea, uint16_t maxArea);
    
private:
    // Lucas-Kanade optical flow implementation
    bool lucasKanadeFlow(uint16_t x, uint16_t y, float& u, float& v);
    
    // Calculate image gradients
    void calculateGradients(uint16_t x, uint16_t y, float& Ix, float& Iy, float& It);
    
    // Motion detection using frame differencing
    uint16_t calculateMotionArea();
    
    // Helper functions
    uint8_t getPixel(uint8_t* frame, uint16_t x, uint16_t y);
    void setPixel(uint8_t* frame, uint16_t x, uint16_t y, uint8_t value);
    bool isValidPixel(uint16_t x, uint16_t y);
};

#endif // OPTICAL_FLOW_H 