#ifndef READ_YPR_H
#define READ_YPR_H

// Make the MPU object accessible to other files
extern MPU6050 mpu;
extern float loop_time;
extern float Desired_Yaw_Rate;

// MPU calibration values - if you are running for the first time update these values using calibrateMPU()
float Acc_Error_X = 0.0354;
float Acc_Error_Y = 0.0192;
float Acc_Error_Z = 0.0629;
float Gyro_Error_X = -2.9746;
float Gyro_Error_Y = 0.6573;
float Gyro_Error_Z = -2.8568;

// Dynamic bias estimation variables
float Gyro_Dynamic_Bias_Z = 0;
float Gyro_Noise_Threshold = 0.12; // Adjusted threshold

// Angle and rate variables
float Acc_Roll_Angle, Acc_Pitch_Angle;
float Roll_Rate, Pitch_Rate, Yaw_Rate;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float Yaw_Angle = 0;

// Moving average filter for gyroscope data
const int GYRO_FILTER_SIZE = 12;
float GyroZ_history[GYRO_FILTER_SIZE] = {0};
int gyro_history_index = 0;

// Kalman filter variables
float Kalman_Roll_Angle = 0, Kalman_Roll_Angle_Uncertinity = 2*2;
float Kalman_Pitch_Angle = 0, Kalman_Pitch_Angle_Uncertinity = 2*2;
float Kalman_Yaw_Angle = 0, Kalman_Yaw_Angle_Uncertinity = 2*2;
float Kalman_Output[] = {0, 0};
float previous_time = 0;
uint32_t LoopTimer;

// Stability detection variables - MORE STRINGENT
bool isStable = false;
unsigned long lastStableTime = 0;
unsigned long lastMovementTime = 0;
int stable_reading_count = 0;
const int REQUIRED_STABLE_READINGS = 500; // Must be stable for 2 seconds (at 250Hz)

// Temperature compensation
float temp_coefficient = 0.002;
float baseline_temp = 25.0;
float current_temp = 25.0;

// Constants
const float MY_RAD_TO_DEG = 57.295779513;
const float DT = 0.004;

// Drift correction variables
float yaw_drift_correction = 0;
bool drift_correction_enabled = false;
unsigned long drift_detection_start = 0;
const unsigned long DRIFT_DETECTION_TIME = 5000000; // 5 seconds in microseconds

// Function prototypes
void readMPUData();
void kalmanFilter();
void printMPUValues();
void calibrateMPU();
void setupMPU();
void updateAngles();
float applyMovingAverageFilter(float newValue);
float estimateGyroBias(float gyroReading);

// Implementation of the 1D Kalman filter for roll and pitch
void kalman_1d(float &KalmanState, float &KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  // Prediction step
  KalmanState = KalmanState + DT * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + DT * DT * 4 * 4;
  
  // Update step
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  
  Kalman_Output[0] = KalmanState; 
  Kalman_Output[1] = KalmanUncertainty;
}

// Simpler Kalman filter for yaw with drift correction ONLY when appropriate
void kalman_1d_yaw(float &KalmanState, float &KalmanUncertainty, float KalmanInput) {
  // Apply noise filtering to gyro input
  if (abs(KalmanInput) < Gyro_Noise_Threshold) {
    KalmanInput = 0;
  }
  
  // Standard Kalman prediction step
  KalmanState = KalmanState + DT * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + DT * DT * 3 * 3;
  
  // ONLY apply drift correction if we've detected prolonged stationary drift
  if (drift_correction_enabled && isStable && micros() - lastMovementTime > DRIFT_DETECTION_TIME) {
    // Very gentle drift correction - only 0.1% per update
    KalmanState *= 0.999;
  }
  
  Kalman_Output[0] = KalmanState; 
  Kalman_Output[1] = KalmanUncertainty;
}

void setupMPU() {
  // Initialize the loop timer
  LoopTimer = micros();
  previous_time = LoopTimer;
  lastMovementTime = LoopTimer;
  
  // Set gyro range to ±500°/s
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  
  // Set accelerometer range to ±8g
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
  
  // Initialize Kalman filter with first accelerometer reading
  readMPUData();
  Kalman_Roll_Angle = Acc_Roll_Angle;
  Kalman_Pitch_Angle = Acc_Pitch_Angle;
  Kalman_Yaw_Angle = 0;
  
  // Get initial temperature
  int16_t temp_raw = mpu.getTemperature();
  baseline_temp = temp_raw / 340.0 + 36.53;
  current_temp = baseline_temp;
  
  Serial.println("MPU6050 setup complete");
}

// Apply moving average filter to smooth gyro readings
float applyMovingAverageFilter(float newValue) {
  GyroZ_history[gyro_history_index] = newValue;
  gyro_history_index = (gyro_history_index + 1) % GYRO_FILTER_SIZE;
  
  float sum = 0;
  for (int i = 0; i < GYRO_FILTER_SIZE; i++) {
    sum += GyroZ_history[i];
  }
  
  return sum / GYRO_FILTER_SIZE;
}

// Estimate gyro bias when sensor is deemed stationary
float estimateGyroBias(float gyroReading) {
  static float bias = 0;
  
  // Only update bias when readings are very small
  if (abs(gyroReading) < Gyro_Noise_Threshold * 0.8) {
    // Slow learning rate for bias estimation
    float learning_rate = 0.001;
    bias = bias * (1 - learning_rate) + gyroReading * learning_rate;
  }
  
  return bias;
}

void readMPUData() {
  int16_t AccX_Raw, AccY_Raw, AccZ_Raw;
  int16_t GyX, GyY, GyZ;
  int16_t temp_raw;
  
  // Get raw sensor data including temperature
  mpu.getMotion6(&AccX_Raw, &AccY_Raw, &AccZ_Raw, &GyX, &GyY, &GyZ);
  temp_raw = mpu.getTemperature();
  
  // Update temperature reading
  current_temp = temp_raw / 340.0 + 36.53;
  
  // Calculate temperature-based correction
  float temp_correction = (current_temp - baseline_temp) * temp_coefficient;
  
  // Process accelerometer data (convert to G units)
  AccX = (float)AccX_Raw / 4096.0;
  AccY = (float)AccY_Raw / 4096.0;
  AccZ = (float)AccZ_Raw / 4096.0;
  
  // Apply calibration offsets
  AccX -= Acc_Error_X;
  AccY -= Acc_Error_Y;
  AccZ -= Acc_Error_Z;
  
  // Calculate angles using atan2 for consistency and convert rad to degree
  Acc_Pitch_Angle = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * MY_RAD_TO_DEG;
  Acc_Roll_Angle = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * MY_RAD_TO_DEG;
  
  // Process gyroscope data (convert to degrees per second)
  GyroX = (float)GyX / 65.5;
  GyroY = (float)GyY / 65.5;
  GyroZ = (float)GyZ / 65.5;
  
  // Apply calibration offsets
  GyroX -= Gyro_Error_X;
  GyroY -= Gyro_Error_Y;
  GyroZ -= Gyro_Error_Z;
  
  // Apply temperature compensation to gyro readings
  GyroX -= temp_correction;
  GyroY -= temp_correction;
  GyroZ -= temp_correction;
  
  // Apply moving average filter to GyroZ for smoother readings
  GyroZ = applyMovingAverageFilter(GyroZ);
  
  // Apply dynamic bias estimation and correction
  Gyro_Dynamic_Bias_Z = estimateGyroBias(GyroZ);
  GyroZ -= Gyro_Dynamic_Bias_Z;
  
  // Apply threshold to reduce noise
  if (abs(GyroZ) < Gyro_Noise_Threshold) {
    GyroZ = 0;
  }
  
  // Store rotation rates
  Roll_Rate = GyroY;
  Pitch_Rate = GyroX;
  Yaw_Rate = GyroZ;
}

void kalmanFilter() {
  // Apply Kalman filter to Roll angle
  kalman_1d(Kalman_Roll_Angle, Kalman_Roll_Angle_Uncertinity, Roll_Rate, Acc_Roll_Angle);
  Kalman_Roll_Angle = Kalman_Output[0]; 
  Kalman_Roll_Angle_Uncertinity = Kalman_Output[1];
  
  // Apply Kalman filter to Pitch angle
  kalman_1d(Kalman_Pitch_Angle, Kalman_Pitch_Angle_Uncertinity, Pitch_Rate, Acc_Pitch_Angle);
  Kalman_Pitch_Angle = Kalman_Output[0]; 
  Kalman_Pitch_Angle_Uncertinity = Kalman_Output[1];
  
  // For yaw, use specialized Kalman filter with controlled drift correction
  kalman_1d_yaw(Kalman_Yaw_Angle, Kalman_Yaw_Angle_Uncertinity, Yaw_Rate);
  Kalman_Yaw_Angle = Kalman_Output[0];
  Kalman_Yaw_Angle_Uncertinity = Kalman_Output[1];
}

void printMPUValues() {
  Serial.println("-------------------------------------");
  Serial.println("Orientation Values:");
  
  Serial.print("Accel-based Roll: ");
  Serial.print(Acc_Roll_Angle);
  Serial.print("°\tPitch: ");
  Serial.print(Acc_Pitch_Angle);
  Serial.println("°");
  
  Serial.print("Kalman Roll: ");
  Serial.print(Kalman_Roll_Angle);
  Serial.print("°\tPitch: ");
  Serial.print(Kalman_Pitch_Angle);
  Serial.print("°\tYaw: ");
  Serial.print(Kalman_Yaw_Angle);
  Serial.println("°");
  
  Serial.print("Rotation Rate - Roll: ");
  Serial.print(Roll_Rate);
  Serial.print("°/s\tPitch: ");
  Serial.print(Pitch_Rate);
  Serial.print("°/s\tYaw: ");
  Serial.print(Yaw_Rate);
  Serial.println("°/s");
  
  Serial.print("Gyro Z-axis bias: ");
  Serial.print(Gyro_Dynamic_Bias_Z);
  
  if (isStable) {
    Serial.print("\tStable for: ");
    Serial.print((micros() - lastStableTime) / 1000000.0);
    Serial.print("s");
  }
  
  if (drift_correction_enabled) {
    Serial.print("\tDrift correction ACTIVE");
  }
  
  Serial.println();
  Serial.println("-------------------------------------");
}

void calibrateMPU() {
  Serial.println("Calibrating MPU6050...");
  Serial.println("Keep the sensor still and level.");
  delay(3000);
  
  int16_t AccX_Raw, AccY_Raw, AccZ_Raw;
  int16_t GyX, GyY, GyZ;
  int16_t temp_raw;
  float accX_sum = 0, accY_sum = 0, accZ_sum = 0;
  float gyroX_sum = 0, gyroY_sum = 0, gyroZ_sum = 0;
  float temp_sum = 0;
  int validSamples = 0;
  const int totalSamples = 5000;
  
  for (int i = 0; i < totalSamples; i++) {
    mpu.getMotion6(&AccX_Raw, &AccY_Raw, &AccZ_Raw, &GyX, &GyY, &GyZ);
    temp_raw = mpu.getTemperature();
    
    float gyroZ_temp = (float)GyZ / 65.5;
    
    if (abs(gyroZ_temp) < 5.0) {
      accX_sum += (float)AccX_Raw / 4096.0;
      accY_sum += (float)AccY_Raw / 4096.0;
      accZ_sum += (float)AccZ_Raw / 4096.0;
      
      gyroX_sum += (float)GyX / 65.5;
      gyroY_sum += (float)GyY / 65.5;
      gyroZ_sum += gyroZ_temp;
      
      temp_sum += temp_raw / 340.0 + 36.53;
      
      validSamples++;
    }
    
    if (i % 500 == 0) Serial.print(".");
    delay(1);
  }
  
  if (validSamples > 0) {
    Acc_Error_X = accX_sum / validSamples;
    Acc_Error_Y = accY_sum / validSamples;
    Acc_Error_Z = (accZ_sum / validSamples) - 1.0;
    
    Gyro_Error_X = gyroX_sum / validSamples;
    Gyro_Error_Y = gyroY_sum / validSamples;
    Gyro_Error_Z = gyroZ_sum / validSamples;
    
    baseline_temp = temp_sum / validSamples;
  } else {
    Serial.println("Error: No valid samples for calibration!");
    return;
  }
  
  Gyro_Dynamic_Bias_Z = Gyro_Error_Z;
  
  for (int i = 0; i < GYRO_FILTER_SIZE; i++) {
    GyroZ_history[i] = 0;
  }
  
  Serial.println("\nCalibration complete!");
  Serial.print("Accel Offsets: X=");
  Serial.print(Acc_Error_X, 4);
  Serial.print(" Y=");
  Serial.print(Acc_Error_Y, 4);
  Serial.print(" Z=");
  Serial.println(Acc_Error_Z, 4);
  
  Serial.print("Gyro Offsets: X=");
  Serial.print(Gyro_Error_X, 4);
  Serial.print(" Y=");
  Serial.print(Gyro_Error_Y, 4);
  Serial.print(" Z=");
  Serial.println(Gyro_Error_Z, 4);
  
  delay(1000);
}

void updateAngles() {
  // Read sensor data
  readMPUData();
  
  // Apply Kalman filter
  kalmanFilter();
  
  // Detect if drone is truly stationary (more stringent requirements)
  bool currentlyStable = (abs(Roll_Rate) < 0.05 && 
                         abs(Pitch_Rate) < 0.05 && 
                         abs(Yaw_Rate) < 0.03 && 
                         abs(Desired_Yaw_Rate) < 0.03);
  
  if (currentlyStable) {
    stable_reading_count++;
    
    if (stable_reading_count >= REQUIRED_STABLE_READINGS) {
      // We've had enough consecutive stable readings
      if (!isStable) {
        isStable = true;
        lastStableTime = micros();
      }
      
      // Check if we've been stable long enough to start drift correction
      if (micros() - lastStableTime > DRIFT_DETECTION_TIME && !drift_correction_enabled) {
        drift_correction_enabled = true;
        drift_detection_start = micros();
        Serial.println("Drift correction enabled - drone stationary for 5 seconds");
      }
      
      // If drift correction is active and no movement in very long time (15 seconds)
      if (drift_correction_enabled && micros() - lastMovementTime > 15000000) {
        // Complete reset only after extreme stationary period
        Kalman_Yaw_Angle = 0;
        Kalman_Yaw_Angle_Uncertinity = 2 * 2;
        drift_correction_enabled = false;
        Serial.println("Yaw reset after 15 seconds of stationarity");
      }
    }
  } else {
    // Movement detected - disable drift correction immediately
    stable_reading_count = 0;
    isStable = false;
    drift_correction_enabled = false;
    lastMovementTime = micros();
  }
  
  // Boundary check for yaw angle (keep within -180 to +180)
  if (Kalman_Yaw_Angle > 180) {
    Kalman_Yaw_Angle -= 360;
  } else if (Kalman_Yaw_Angle < -180) {
    Kalman_Yaw_Angle += 360;
  }
}

#endif // READ_YPR_H
