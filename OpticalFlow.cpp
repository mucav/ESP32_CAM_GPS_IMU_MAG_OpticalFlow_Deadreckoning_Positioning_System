/*
 * Optical Flow Library Implementation
 * Based on esp32-motion project
 */

#include "OpticalFlow.h"
#include <math.h>

OpticalFlow::OpticalFlow() {
    prevFrame = nullptr;
    currFrame = nullptr;
    frameWidth = 0;
    frameHeight = 0;
    frameSize = 0;
    initialized = false;
    
    // Default parameters
    lambda = 0.1f;
    maxIterations = 10;
    motionThreshold = MOTION_THRESHOLD;
    minMotionArea = MOTION_MIN_AREA;
    maxMotionArea = MOTION_MAX_AREA;
}

OpticalFlow::~OpticalFlow() {
    if (prevFrame) {
        free(prevFrame);
        prevFrame = nullptr;
    }
    if (currFrame) {
        free(currFrame);
        currFrame = nullptr;
    }
}

bool OpticalFlow::begin(uint16_t width, uint16_t height) {
    frameWidth = width;
    frameHeight = height;
    frameSize = width * height;
    
    // Allocate frame buffers
    prevFrame = (uint8_t*)malloc(frameSize);
    currFrame = (uint8_t*)malloc(frameSize);
    
    if (!prevFrame || !currFrame) {
        Serial.println("Failed to allocate frame buffers");
        return false;
    }
    
    // Initialize buffers
    memset(prevFrame, 0, frameSize);
    memset(currFrame, 0, frameSize);
    
    initialized = true;
    Serial.printf("Optical flow initialized: %dx%d\n", width, height);
    return true;
}

FlowResult OpticalFlow::calculateFlow(camera_fb_t* frame) {
    FlowResult result = {0};
    result.valid = false;
    result.timestamp = millis();
    
    if (!initialized || !frame || frame->len != frameSize) {
        return result;
    }
    
    // Copy current frame to buffer
    memcpy(currFrame, frame->buf, frameSize);
    
    // Calculate optical flow using Lucas-Kanade method
    float totalU = 0, totalV = 0;
    int validPoints = 0;
    
    // Sample fewer points to reduce processing time
    for (uint16_t y = FLOW_WINDOW_SIZE; y < frameHeight - FLOW_WINDOW_SIZE; y += FLOW_WINDOW_SIZE * 4) {
        for (uint16_t x = FLOW_WINDOW_SIZE; x < frameWidth - FLOW_WINDOW_SIZE; x += FLOW_WINDOW_SIZE * 4) {
            float u = 0, v = 0;
            
            if (lucasKanadeFlow(x, y, u, v)) {
                // Check if displacement is reasonable
                if (abs(u) < FLOW_MAX_DISPLACEMENT && abs(v) < FLOW_MAX_DISPLACEMENT) {
                    totalU += u;
                    totalV += v;
                    validPoints++;
                }
            }
        }
    }
    
    if (validPoints > 0) {
        result.deltaX = totalU / validPoints;
        result.deltaY = totalV / validPoints;
        result.confidence = (float)validPoints / ((frameWidth / (FLOW_WINDOW_SIZE * 4)) * (frameHeight / (FLOW_WINDOW_SIZE * 4)));
        result.valid = true;
    }
    
    // Swap frame buffers
    uint8_t* temp = prevFrame;
    prevFrame = currFrame;
    currFrame = temp;
    
    return result;
}

MotionResult OpticalFlow::detectMotion(camera_fb_t* frame) {
    MotionResult result = {0};
    result.timestamp = millis();
    
    if (!initialized || !frame || frame->len != frameSize) {
        return result;
    }
    
    // Copy current frame
    memcpy(currFrame, frame->buf, frameSize);
    
    // Calculate motion area
    uint16_t motionArea = calculateMotionArea();
    
    if (motionArea > minMotionArea && motionArea < maxMotionArea) {
        result.motionDetected = true;
        result.motionArea = motionArea;
        
        // Calculate motion center (simplified)
        result.motionCenterX = frameWidth / 2;
        result.motionCenterY = frameHeight / 2;
    }
    
    // Swap frame buffers
    uint8_t* temp = prevFrame;
    prevFrame = currFrame;
    currFrame = temp;
    
    return result;
}

bool OpticalFlow::lucasKanadeFlow(uint16_t x, uint16_t y, float& u, float& v) {
    float Ix, Iy, It;
    float A11 = 0, A12 = 0, A22 = 0;
    float b1 = 0, b2 = 0;
    
    // Calculate structure tensor and temporal gradient
    for (int dy = -FLOW_WINDOW_SIZE/2; dy <= FLOW_WINDOW_SIZE/2; dy++) {
        for (int dx = -FLOW_WINDOW_SIZE/2; dx <= FLOW_WINDOW_SIZE/2; dx++) {
            uint16_t px = x + dx;
            uint16_t py = y + dy;
            
            if (!isValidPixel(px, py)) continue;
            
            calculateGradients(px, py, Ix, Iy, It);
            
            A11 += Ix * Ix;
            A12 += Ix * Iy;
            A22 += Iy * Iy;
            b1 += Ix * It;
            b2 += Iy * It;
        }
    }
    
    // Calculate determinant
    float det = A11 * A22 - A12 * A12;
    
    if (det < 1e-6) {
        return false; // Singular matrix
    }
    
    // Solve linear system
    float invDet = 1.0f / det;
    u = -(A22 * b1 - A12 * b2) * invDet;
    v = -(-A12 * b1 + A11 * b2) * invDet;
    
    return true;
}

void OpticalFlow::calculateGradients(uint16_t x, uint16_t y, float& Ix, float& Iy, float& It) {
    // Spatial gradients using central differences
    if (x > 0 && x < frameWidth - 1) {
        Ix = (getPixel(currFrame, x + 1, y) - getPixel(currFrame, x - 1, y)) / 2.0f;
    } else {
        Ix = 0;
    }
    
    if (y > 0 && y < frameHeight - 1) {
        Iy = (getPixel(currFrame, x, y + 1) - getPixel(currFrame, x, y - 1)) / 2.0f;
    } else {
        Iy = 0;
    }
    
    // Temporal gradient
    It = getPixel(currFrame, x, y) - getPixel(prevFrame, x, y);
}

uint16_t OpticalFlow::calculateMotionArea() {
    uint16_t motionPixels = 0;
    
    for (uint16_t y = 0; y < frameHeight; y++) {
        for (uint16_t x = 0; x < frameWidth; x++) {
            uint8_t curr = getPixel(currFrame, x, y);
            uint8_t prev = getPixel(prevFrame, x, y);
            
            if (abs(curr - prev) > motionThreshold) {
                motionPixels++;
            }
        }
    }
    
    return motionPixels;
}

void OpticalFlow::getFrameDifference(uint8_t* diffBuffer) {
    if (!initialized || !diffBuffer) return;
    
    for (uint32_t i = 0; i < frameSize; i++) {
        int16_t diff = currFrame[i] - prevFrame[i];
        diffBuffer[i] = abs(diff);
    }
}

void OpticalFlow::setFlowThreshold(uint8_t threshold) {
    // Not used in current implementation
}

void OpticalFlow::setMotionThreshold(uint8_t threshold) {
    motionThreshold = threshold;
}

void OpticalFlow::setMotionAreaLimits(uint16_t minArea, uint16_t maxArea) {
    minMotionArea = minArea;
    maxMotionArea = maxArea;
}

uint8_t OpticalFlow::getPixel(uint8_t* frame, uint16_t x, uint16_t y) {
    if (!isValidPixel(x, y)) return 0;
    return frame[y * frameWidth + x];
}

void OpticalFlow::setPixel(uint8_t* frame, uint16_t x, uint16_t y, uint8_t value) {
    if (!isValidPixel(x, y)) return;
    frame[y * frameWidth + x] = value;
}

bool OpticalFlow::isValidPixel(uint16_t x, uint16_t y) {
    return x < frameWidth && y < frameHeight;
} 