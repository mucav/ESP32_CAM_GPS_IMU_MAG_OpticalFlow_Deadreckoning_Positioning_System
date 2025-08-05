/*
 * Extended Kalman Filter Implementation
 * Simplified version for ESP32-CAM positioning system
 */

#include "EKF.h"
#include <math.h>

EKF::EKF() {
    initialized = false;
    
    // Initialize all matrices to zero
    memset(&state, 0, sizeof(state));
    memset(&covariance, 0, sizeof(covariance));
    memset(F, 0, sizeof(F));
    memset(H, 0, sizeof(H));
    memset(K, 0, sizeof(K));
    memset(S, 0, sizeof(S));
    memset(y, 0, sizeof(y));
}

void EKF::initialize(const EKFState& initialState) {
    state = initialState;
    
    // Initialize covariance matrices
    initializeMatrices();
    
    initialized = true;
    Serial.println("EKF initialized");
}

void EKF::reset() {
    memset(&state, 0, sizeof(state));
    memset(&covariance, 0, sizeof(covariance));
    initialized = false;
}

void EKF::predict(float dt) {
    if (!initialized || dt <= 0) return;
    
    predictStep(dt);
}

void EKF::update(const EKFMeasurement& measurement) {
    if (!initialized) return;
    
    updateStep(measurement);
}

void EKF::step(float dt, const EKFMeasurement& measurement) {
    if (!initialized) return;
    
    predict(dt);
    update(measurement);
}

EKFState EKF::getState() const {
    return state;
}

void EKF::getState(float* stateArray) const {
    if (!stateArray) return;
    
    stateArray[0] = state.x;
    stateArray[1] = state.y;
    stateArray[2] = state.z;
    stateArray[3] = state.vx;
    stateArray[4] = state.vy;
    stateArray[5] = state.vz;
    stateArray[6] = state.ax;
    stateArray[7] = state.ay;
    stateArray[8] = state.az;
    stateArray[9] = state.roll;
    stateArray[10] = state.pitch;
    stateArray[11] = state.yaw;
    stateArray[12] = state.bx;
    stateArray[13] = state.by;
    stateArray[14] = state.bz;
}

void EKF::getCovariance(float* covArray) const {
    if (!covArray) return;
    
    for (int i = 0; i < EKF_STATE_SIZE; i++) {
        for (int j = 0; j < EKF_STATE_SIZE; j++) {
            covArray[i * EKF_STATE_SIZE + j] = covariance.P[i][j];
        }
    }
}

void EKF::setProcessNoise(float pos_noise, float vel_noise, float acc_noise, float att_noise, float bias_noise) {
    // Set process noise covariance matrix
    memset(covariance.Q, 0, sizeof(covariance.Q));
    
    // Position noise
    covariance.Q[0][0] = pos_noise * pos_noise;
    covariance.Q[1][1] = pos_noise * pos_noise;
    covariance.Q[2][2] = pos_noise * pos_noise;
    
    // Velocity noise
    covariance.Q[3][3] = vel_noise * vel_noise;
    covariance.Q[4][4] = vel_noise * vel_noise;
    covariance.Q[5][5] = vel_noise * vel_noise;
    
    // Acceleration noise
    covariance.Q[6][6] = acc_noise * acc_noise;
    covariance.Q[7][7] = acc_noise * acc_noise;
    covariance.Q[8][8] = acc_noise * acc_noise;
    
    // Attitude noise
    covariance.Q[9][9] = att_noise * att_noise;
    covariance.Q[10][10] = att_noise * att_noise;
    covariance.Q[11][11] = att_noise * att_noise;
    
    // Bias noise
    covariance.Q[12][12] = bias_noise * bias_noise;
    covariance.Q[13][13] = bias_noise * bias_noise;
    covariance.Q[14][14] = bias_noise * bias_noise;
}

void EKF::setMeasurementNoise(float gps_noise, float baro_noise, float flow_noise) {
    // Set measurement noise covariance matrix
    memset(covariance.R, 0, sizeof(covariance.R));
    
    // GPS noise
    covariance.R[0][0] = gps_noise * gps_noise;
    covariance.R[1][1] = gps_noise * gps_noise;
    covariance.R[2][2] = gps_noise * gps_noise;
    
    // Barometer noise
    covariance.R[3][3] = baro_noise * baro_noise;
    
    // Optical flow noise
    covariance.R[4][4] = flow_noise * flow_noise;
    covariance.R[5][5] = flow_noise * flow_noise;
}

void EKF::setInitialUncertainty(float pos_uncertainty, float vel_uncertainty, float att_uncertainty) {
    // Set initial state covariance matrix
    memset(covariance.P, 0, sizeof(covariance.P));
    
    // Position uncertainty
    covariance.P[0][0] = pos_uncertainty * pos_uncertainty;
    covariance.P[1][1] = pos_uncertainty * pos_uncertainty;
    covariance.P[2][2] = pos_uncertainty * pos_uncertainty;
    
    // Velocity uncertainty
    covariance.P[3][3] = vel_uncertainty * vel_uncertainty;
    covariance.P[4][4] = vel_uncertainty * vel_uncertainty;
    covariance.P[5][5] = vel_uncertainty * vel_uncertainty;
    
    // Attitude uncertainty
    covariance.P[9][9] = att_uncertainty * att_uncertainty;
    covariance.P[10][10] = att_uncertainty * att_uncertainty;
    covariance.P[11][11] = att_uncertainty * att_uncertainty;
}

bool EKF::isInitialized() const {
    return initialized;
}

float EKF::getPositionUncertainty() const {
    return sqrt(covariance.P[0][0] + covariance.P[1][1] + covariance.P[2][2]);
}

float EKF::getVelocityUncertainty() const {
    return sqrt(covariance.P[3][3] + covariance.P[4][4] + covariance.P[5][5]);
}

float EKF::getAttitudeUncertainty() const {
    return sqrt(covariance.P[9][9] + covariance.P[10][10] + covariance.P[11][11]);
}

void EKF::printState() const {
    Serial.println("EKF State:");
    Serial.printf("Position: %.2f, %.2f, %.2f m\n", state.x, state.y, state.z);
    Serial.printf("Velocity: %.2f, %.2f, %.2f m/s\n", state.vx, state.vy, state.vz);
    Serial.printf("Attitude: %.2f, %.2f, %.2f rad\n", state.roll, state.pitch, state.yaw);
    Serial.printf("Bias: %.4f, %.4f, %.4f m/s²\n", state.bx, state.by, state.bz);
}

void EKF::printCovariance() const {
    Serial.println("EKF Covariance (diagonal):");
    for (int i = 0; i < EKF_STATE_SIZE; i++) {
        Serial.printf("P[%d][%d] = %.6f\n", i, i, covariance.P[i][i]);
    }
}

// Private methods
void EKF::initializeMatrices() {
    // Initialize state transition matrix (identity matrix)
    memset(F, 0, sizeof(F));
    for (int i = 0; i < EKF_STATE_SIZE; i++) {
        F[i][i] = 1.0f;
    }
    
    // Initialize measurement matrix
    memset(H, 0, sizeof(H));
    // GPS measurements (x, y, z)
    H[0][0] = 1.0f;  // x
    H[1][1] = 1.0f;  // y
    H[2][2] = 1.0f;  // z
    // Barometer measurement (z)
    H[3][2] = 1.0f;  // z
    // Optical flow measurements (x, y)
    H[4][0] = 1.0f;  // x
    H[5][1] = 1.0f;  // y
}

void EKF::predictStep(float dt) {
    // Simple prediction step
    // Update position based on velocity
    state.x += state.vx * dt;
    state.y += state.vy * dt;
    state.z += state.vz * dt;
    
    // Update velocity based on acceleration
    state.vx += state.ax * dt;
    state.vy += state.ay * dt;
    state.vz += state.az * dt;
    
    // Update covariance matrix
    computeStateTransitionMatrix(dt);
    
    // P = F * P * F^T + Q
    float temp[EKF_STATE_SIZE][EKF_STATE_SIZE];
    float temp2[EKF_STATE_SIZE][EKF_STATE_SIZE];
    
    // temp = F * P
    matrixMultiply((float*)F, (float*)covariance.P, (float*)temp, EKF_STATE_SIZE, EKF_STATE_SIZE, EKF_STATE_SIZE);
    
    // temp2 = temp * F^T
    matrixTranspose((float*)F, (float*)temp2, EKF_STATE_SIZE, EKF_STATE_SIZE);
    matrixMultiply((float*)temp, (float*)temp2, (float*)covariance.P, EKF_STATE_SIZE, EKF_STATE_SIZE, EKF_STATE_SIZE);
    
    // Add process noise
    for (int i = 0; i < EKF_STATE_SIZE; i++) {
        for (int j = 0; j < EKF_STATE_SIZE; j++) {
            covariance.P[i][j] += covariance.Q[i][j];
        }
    }
    
    state.timestamp = millis();
}

void EKF::updateStep(const EKFMeasurement& measurement) {
    // Prepare measurement vector
    memset(y, 0, sizeof(y));
    int validMeasurements = 0;
    
    if (measurement.gps_valid) {
        y[0] = measurement.gps_x - state.x;
        y[1] = measurement.gps_y - state.y;
        y[2] = measurement.gps_z - state.z;
        validMeasurements += 3;
    }
    
    if (measurement.baro_valid) {
        y[3] = measurement.baro_z - state.z;
        validMeasurements++;
    }
    
    if (measurement.flow_valid) {
        y[4] = measurement.flow_x;
        y[5] = measurement.flow_y;
        validMeasurements += 2;
    }
    
    if (validMeasurements == 0) return;
    
    // Compute Kalman gain
    computeKalmanGain();
    
    // Update state
    updateState(measurement);
    
    // Update covariance
    updateCovariance();
    
    // Normalize angles
    normalizeAngles();
    
    // Constrain covariance
    constrainCovariance();
}

void EKF::computeStateTransitionMatrix(float dt) {
    // Simple state transition matrix
    // Position derivatives
    F[0][3] = dt;  // dx/dvx
    F[1][4] = dt;  // dy/dvy
    F[2][5] = dt;  // dz/dvz
    
    // Velocity derivatives
    F[3][6] = dt;  // dvx/dax
    F[4][7] = dt;  // dvy/day
    F[5][8] = dt;  // dvz/daz
}

void EKF::computeMeasurementMatrix() {
    // Measurement matrix is already initialized in initializeMatrices()
}

void EKF::computeKalmanGain() {
    // S = H * P * H^T + R
    float temp[EKF_MEASUREMENT_SIZE][EKF_STATE_SIZE];
    float H_transpose[EKF_STATE_SIZE][EKF_MEASUREMENT_SIZE];
    
    // temp = H * P
    matrixMultiply((float*)H, (float*)covariance.P, (float*)temp, EKF_MEASUREMENT_SIZE, EKF_STATE_SIZE, EKF_STATE_SIZE);
    
    // H_transpose = H^T
    matrixTranspose((float*)H, (float*)H_transpose, EKF_MEASUREMENT_SIZE, EKF_STATE_SIZE);
    
    // S = temp * H^T + R
    matrixMultiply((float*)temp, (float*)H_transpose, (float*)S, EKF_MEASUREMENT_SIZE, EKF_STATE_SIZE, EKF_MEASUREMENT_SIZE);
    
    for (int i = 0; i < EKF_MEASUREMENT_SIZE; i++) {
        for (int j = 0; j < EKF_MEASUREMENT_SIZE; j++) {
            S[i][j] += covariance.R[i][j];
        }
    }
    
    // K = P * H^T * S^(-1)
    float temp2[EKF_STATE_SIZE][EKF_MEASUREMENT_SIZE];
    matrixMultiply((float*)covariance.P, (float*)H_transpose, (float*)temp2, EKF_STATE_SIZE, EKF_STATE_SIZE, EKF_MEASUREMENT_SIZE);
    
    // For simplicity, use diagonal approximation of S^(-1)
    for (int i = 0; i < EKF_STATE_SIZE; i++) {
        for (int j = 0; j < EKF_MEASUREMENT_SIZE; j++) {
            K[i][j] = 0;
            for (int k = 0; k < EKF_MEASUREMENT_SIZE; k++) {
                if (S[k][k] > 1e-6) {
                    K[i][j] += temp2[i][k] * H_transpose[k][j] / S[k][k];
                }
            }
        }
    }
}

void EKF::updateState(const EKFMeasurement& measurement) {
    // x = x + K * y
    for (int i = 0; i < EKF_STATE_SIZE; i++) {
        float correction = 0;
        for (int j = 0; j < EKF_MEASUREMENT_SIZE; j++) {
            correction += K[i][j] * y[j];
        }
        
        switch (i) {
            case 0: state.x += correction; break;
            case 1: state.y += correction; break;
            case 2: state.z += correction; break;
            case 3: state.vx += correction; break;
            case 4: state.vy += correction; break;
            case 5: state.vz += correction; break;
            case 6: state.ax += correction; break;
            case 7: state.ay += correction; break;
            case 8: state.az += correction; break;
            case 9: state.roll += correction; break;
            case 10: state.pitch += correction; break;
            case 11: state.yaw += correction; break;
            case 12: state.bx += correction; break;
            case 13: state.by += correction; break;
            case 14: state.bz += correction; break;
        }
    }
}

void EKF::updateCovariance() {
    // P = (I - K * H) * P
    float I[EKF_STATE_SIZE][EKF_STATE_SIZE];
    float KH[EKF_STATE_SIZE][EKF_STATE_SIZE];
    float temp[EKF_STATE_SIZE][EKF_STATE_SIZE];
    
    // Initialize identity matrix
    memset(I, 0, sizeof(I));
    for (int i = 0; i < EKF_STATE_SIZE; i++) {
        I[i][i] = 1.0f;
    }
    
    // KH = K * H
    matrixMultiply((float*)K, (float*)H, (float*)KH, EKF_STATE_SIZE, EKF_MEASUREMENT_SIZE, EKF_STATE_SIZE);
    
    // temp = (I - KH)
    for (int i = 0; i < EKF_STATE_SIZE; i++) {
        for (int j = 0; j < EKF_STATE_SIZE; j++) {
            temp[i][j] = I[i][j] - KH[i][j];
        }
    }
    
    // P = temp * P
    float oldP[EKF_STATE_SIZE][EKF_STATE_SIZE];
    memcpy(oldP, covariance.P, sizeof(oldP));
    matrixMultiply((float*)temp, (float*)oldP, (float*)covariance.P, EKF_STATE_SIZE, EKF_STATE_SIZE, EKF_STATE_SIZE);
}

void EKF::normalizeAngles() {
    // Normalize roll, pitch, yaw to [-π, π]
    while (state.roll > PI) state.roll -= 2 * PI;
    while (state.roll < -PI) state.roll += 2 * PI;
    
    while (state.pitch > PI) state.pitch -= 2 * PI;
    while (state.pitch < -PI) state.pitch += 2 * PI;
    
    while (state.yaw > PI) state.yaw -= 2 * PI;
    while (state.yaw < -PI) state.yaw += 2 * PI;
}

void EKF::constrainCovariance() {
    // Ensure covariance matrix remains positive definite
    for (int i = 0; i < EKF_STATE_SIZE; i++) {
        if (covariance.P[i][i] < 1e-6) {
            covariance.P[i][i] = 1e-6;
        }
    }
}

// Matrix operations
void EKF::matrixMultiply(const float* A, const float* B, float* C, int rowsA, int colsA, int colsB) {
    for (int i = 0; i < rowsA; i++) {
        for (int j = 0; j < colsB; j++) {
            C[i * colsB + j] = 0;
            for (int k = 0; k < colsA; k++) {
                C[i * colsB + j] += A[i * colsA + k] * B[k * colsB + j];
            }
        }
    }
}

void EKF::matrixTranspose(const float* A, float* AT, int rows, int cols) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            AT[j * rows + i] = A[i * cols + j];
        }
    }
}

void EKF::matrixInverse3x3(const float* A, float* Ainv) {
    // Simple 3x3 matrix inverse implementation
    float det = A[0] * (A[4] * A[8] - A[5] * A[7]) -
                A[1] * (A[3] * A[8] - A[5] * A[6]) +
                A[2] * (A[3] * A[7] - A[4] * A[6]);
    
    if (abs(det) < 1e-6) {
        // Matrix is singular, set to identity
        Ainv[0] = Ainv[4] = Ainv[8] = 1.0f;
        Ainv[1] = Ainv[2] = Ainv[3] = Ainv[5] = Ainv[6] = Ainv[7] = 0.0f;
        return;
    }
    
    float invDet = 1.0f / det;
    
    Ainv[0] = (A[4] * A[8] - A[5] * A[7]) * invDet;
    Ainv[1] = (A[2] * A[7] - A[1] * A[8]) * invDet;
    Ainv[2] = (A[1] * A[5] - A[2] * A[4]) * invDet;
    Ainv[3] = (A[5] * A[6] - A[3] * A[8]) * invDet;
    Ainv[4] = (A[0] * A[8] - A[2] * A[6]) * invDet;
    Ainv[5] = (A[2] * A[3] - A[0] * A[5]) * invDet;
    Ainv[6] = (A[3] * A[7] - A[4] * A[6]) * invDet;
    Ainv[7] = (A[1] * A[6] - A[0] * A[7]) * invDet;
    Ainv[8] = (A[0] * A[4] - A[1] * A[3]) * invDet;
}

void EKF::matrixInverse6x6(const float* A, float* Ainv) {
    // For 6x6 matrix, use a simplified approach
    // In practice, you might want to use a more robust method
    // For now, we'll use a diagonal approximation
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            if (i == j && A[i * 6 + i] != 0) {
                Ainv[i * 6 + j] = 1.0f / A[i * 6 + i];
            } else {
                Ainv[i * 6 + j] = 0.0f;
            }
        }
    }
}

void EKF::choleskyDecomposition(const float* A, float* L, int size) {
    // Cholesky decomposition: A = L * L^T
    for (int i = 0; i < size; i++) {
        for (int j = 0; j <= i; j++) {
            float sum = 0.0f;
            
            if (j == i) {
                for (int k = 0; k < j; k++) {
                    sum += L[j * size + k] * L[j * size + k];
                }
                L[j * size + j] = sqrt(A[j * size + j] - sum);
            } else {
                for (int k = 0; k < j; k++) {
                    sum += L[i * size + k] * L[j * size + k];
                }
                L[i * size + j] = (A[i * size + j] - sum) / L[j * size + j];
            }
        }
    }
}

void EKF::forwardSubstitution(const float* L, const float* b, float* y, int size) {
    // Solve L * y = b for y
    for (int i = 0; i < size; i++) {
        float sum = 0.0f;
        for (int j = 0; j < i; j++) {
            sum += L[i * size + j] * y[j];
        }
        y[i] = (b[i] - sum) / L[i * size + i];
    }
}

void EKF::backwardSubstitution(const float* L, const float* y, float* x, int size) {
    // Solve L^T * x = y for x
    for (int i = size - 1; i >= 0; i--) {
        float sum = 0.0f;
        for (int j = i + 1; j < size; j++) {
            sum += L[j * size + i] * x[j];
        }
        x[i] = (y[i] - sum) / L[i * size + i];
    }
} 