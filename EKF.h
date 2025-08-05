/*
 * Extended Kalman Filter Library for ESP32
 * 
 * Provides sensor fusion for IMU, GPS, barometer, and optical flow
 * Implements a 15-state EKF for position, velocity, and attitude estimation
 */

#ifndef EKF_H
#define EKF_H

#include <Arduino.h>
#include <math.h>

// EKF state vector (15 states)
// [x, y, z, vx, vy, vz, ax, ay, az, roll, pitch, yaw, bx, by, bz]
// where bx, by, bz are accelerometer biases
#define EKF_STATE_SIZE 15

// Measurement vector (6 measurements)
// [gps_x, gps_y, gps_z, baro_z, flow_x, flow_y]
#define EKF_MEASUREMENT_SIZE 6

// Process noise covariance matrix
#define EKF_PROCESS_NOISE_SIZE EKF_STATE_SIZE

// Measurement noise covariance matrix
#define EKF_MEASUREMENT_NOISE_SIZE EKF_MEASUREMENT_SIZE

// Data structures
struct EKFState {
    float x, y, z;           // Position (m)
    float vx, vy, vz;        // Velocity (m/s)
    float ax, ay, az;        // Acceleration (m/s²)
    float roll, pitch, yaw;  // Attitude (rad)
    float bx, by, bz;        // Accelerometer bias (m/s²)
    uint32_t timestamp;
};

struct EKFMeasurement {
    float gps_x, gps_y, gps_z;    // GPS position (m)
    float baro_z;                 // Barometer altitude (m)
    float flow_x, flow_y;         // Optical flow (m)
    bool gps_valid;
    bool baro_valid;
    bool flow_valid;
    uint32_t timestamp;
};

struct EKFCovariance {
    float P[EKF_STATE_SIZE][EKF_STATE_SIZE];  // State covariance matrix
    float Q[EKF_PROCESS_NOISE_SIZE][EKF_PROCESS_NOISE_SIZE];  // Process noise
    float R[EKF_MEASUREMENT_NOISE_SIZE][EKF_MEASUREMENT_NOISE_SIZE];  // Measurement noise
};

class EKF {
private:
    EKFState state;
    EKFCovariance covariance;
    bool initialized;
    
    // Temporary matrices for calculations
    float F[EKF_STATE_SIZE][EKF_STATE_SIZE];  // State transition matrix
    float H[EKF_MEASUREMENT_SIZE][EKF_STATE_SIZE];  // Measurement matrix
    float K[EKF_STATE_SIZE][EKF_MEASUREMENT_SIZE];  // Kalman gain
    float S[EKF_MEASUREMENT_SIZE][EKF_MEASUREMENT_SIZE];  // Innovation covariance
    float y[EKF_MEASUREMENT_SIZE];  // Innovation vector
    
    // Private methods
    void initializeMatrices();
    void predictStep(float dt);
    void updateStep(const EKFMeasurement& measurement);
    void computeStateTransitionMatrix(float dt);
    void computeMeasurementMatrix();
    void computeKalmanGain();
    void updateState(const EKFMeasurement& measurement);
    void updateCovariance();
    void normalizeAngles();
    void constrainCovariance();
    
    // Matrix operations
    void matrixMultiply(const float* A, const float* B, float* C, int rowsA, int colsA, int colsB);
    void matrixTranspose(const float* A, float* AT, int rows, int cols);
    void matrixInverse3x3(const float* A, float* Ainv);
    void matrixInverse6x6(const float* A, float* Ainv);
    void choleskyDecomposition(const float* A, float* L, int size);
    void forwardSubstitution(const float* L, const float* b, float* y, int size);
    void backwardSubstitution(const float* L, const float* y, float* x, int size);

public:
    EKF();
    
    // Initialization
    void initialize(const EKFState& initialState);
    void reset();
    
    // Main EKF functions
    void predict(float dt);
    void update(const EKFMeasurement& measurement);
    void step(float dt, const EKFMeasurement& measurement);
    
    // State access
    EKFState getState() const;
    void getState(float* stateArray) const;
    void getCovariance(float* covArray) const;
    
    // Configuration
    void setProcessNoise(float pos_noise, float vel_noise, float acc_noise, float att_noise, float bias_noise);
    void setMeasurementNoise(float gps_noise, float baro_noise, float flow_noise);
    void setInitialUncertainty(float pos_uncertainty, float vel_uncertainty, float att_uncertainty);
    
    // Utility methods
    bool isInitialized() const;
    float getPositionUncertainty() const;
    float getVelocityUncertainty() const;
    float getAttitudeUncertainty() const;
    
    // Debug
    void printState() const;
    void printCovariance() const;
};

#endif // EKF_H 