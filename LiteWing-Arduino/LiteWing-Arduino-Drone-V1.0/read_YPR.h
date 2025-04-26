#ifndef READ_YPR_H
#define READ_YPR_H

// Make the MPU object accessible to other files
extern MPU6050 mpu;
extern float loop_time;

// MPU calibration values - if you are running for the first time update these values using calibrateMPU()
float Acc_Error_X = 0.0061;
float Acc_Error_Y = -0.0710;
float Acc_Error_Z = -0.1368;
float Gyro_Error_X = -1.4008;
float Gyro_Error_Y = -3.9069;
float Gyro_Error_Z = -1.3297;

// Angle and rate variables
float Acc_Roll_Angle, Acc_Pitch_Angle;
float Roll_Rate, Pitch_Rate, Yaw_Rate;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float Yaw_Angle = 0; // New variable to store integrated yaw angle

// Initialize previous values to prevent first-loop issues
float AccX_prev = 0, AccY_prev = 0, AccZ_prev = 0;
float GyroX_prev = 0, GyroY_prev = 0, GyroZ_prev = 0;

// Kalman filter variables
float Kalman_Roll_Angle = 0, Kalman_Roll_Angle_Uncertinity = 2*2;
float Kalman_Pitch_Angle = 0, Kalman_Pitch_Angle_Uncertinity = 2*2;
float Kalman_Yaw_Angle = 0, Kalman_Yaw_Angle_Uncertinity = 2*2; // New for yaw
float Kalman_Output[] = {0, 0};
float previous_time = 0;
uint32_t LoopTimer;

// Constants
const float DT = 0.004; // Time step in seconds (4ms loop)

// Function prototypes
void readMPUData();
void kalmanFilter();
void printMPUValues();
void calibrateMPU();
void setupMPU();

// Implementation of the 1D Kalman filter 
void kalman_1d(float &KalmanState, float &KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + DT * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + DT * DT * 4 * 4;
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman_Output[0] = KalmanState; 
  Kalman_Output[1] = KalmanUncertainty;
}

void setupMPU() {
  // Initialize the loop timer
  LoopTimer = micros();
  previous_time = LoopTimer;
  
  // Initialize Kalman filter with first accelerometer reading
  readMPUData();
  Kalman_Roll_Angle = Acc_Roll_Angle;
  Kalman_Pitch_Angle = Acc_Pitch_Angle;
  Kalman_Yaw_Angle = 0; // Initialize yaw angle to 0
}

void readMPUData() {
  int16_t AccX_Raw, AccY_Raw, AccZ_Raw;
  int16_t GyX, GyY, GyZ;
  
  // Get raw sensor data
  mpu.getMotion6(&AccX_Raw, &AccY_Raw, &AccZ_Raw, &GyX, &GyY, &GyZ);
  
  // Process accelerometer data (convert to G units)
  AccX = (float)AccX_Raw / 4096.0;  // For ±8g range
  AccY = (float)AccY_Raw / 4096.0;
  AccZ = (float)AccZ_Raw / 4096.0;
  
  // Apply calibration offsets
  AccX -= Acc_Error_X;
  AccY -= Acc_Error_Y;
  AccZ -= Acc_Error_Z;
  
  // Calculate angles using atan2 for consistency and convert rad to degree
  Acc_Pitch_Angle = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * 57.295779513; //math formula to calculate Euler angle θ (theta)
  Acc_Roll_Angle = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 57.295779513; //math formula to calculate Euler angle φ (phi)
  
  // Process gyroscope data (convert to degrees per second)
  GyroX = (float)GyX / 65.5;  // For ±500°/s range
  GyroY = (float)GyY / 65.5;
  GyroZ = (float)GyZ / 65.5;
  
  // Apply calibration offsets
  GyroX -= Gyro_Error_X;
  GyroY -= Gyro_Error_Y;
  GyroZ -= Gyro_Error_Z;
  
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
  
  // For yaw, use gyroscope only (no accelerometer measurement)
  // Note: Without a measurement, this is equivalent to integration with smoothing
  kalman_1d(Kalman_Yaw_Angle, Kalman_Yaw_Angle_Uncertinity, Yaw_Rate, Kalman_Yaw_Angle);
  Kalman_Yaw_Angle = Kalman_Output[0];
  Kalman_Yaw_Angle_Uncertinity = Kalman_Output[1];
}

void printMPUValues() {
  Serial.println("-------------------------------------");
  Serial.println("Orientation Values:");
  
  // Print accelerometer-based angles
  Serial.print("Accel-based Roll: ");
  Serial.print(Acc_Roll_Angle);
  Serial.print("°\tPitch: ");
  Serial.print(Acc_Pitch_Angle);
  Serial.println("°");
  
  // Print Kalman-filtered angles
  Serial.print("Kalman Roll: ");
  Serial.print(Kalman_Roll_Angle);
  Serial.print("°\tPitch: ");
  Serial.print(Kalman_Pitch_Angle);
  Serial.print("°\tYaw: ");
  Serial.print(Kalman_Yaw_Angle);
  Serial.println("°");
  
  // Print rotation rates
  Serial.print("Rotation Rate - Roll: ");
  Serial.print(Roll_Rate);
  Serial.print("°/s\tPitch: ");
  Serial.print(Pitch_Rate);
  Serial.print("°/s\tYaw: ");
  Serial.print(Yaw_Rate);
  Serial.println("°/s");
  
  Serial.println("-------------------------------------");
}

void calibrateMPU() {
  Serial.println("Calibrating MPU6050...");
  Serial.println("Keep the sensor still and level.");
  delay(3000);
  
  int16_t AccX_Raw, AccY_Raw, AccZ_Raw;
  int16_t GyX, GyY, GyZ;
  float accX_sum = 0, accY_sum = 0, accZ_sum = 0;
  float gyroX_sum = 0, gyroY_sum = 0, gyroZ_sum = 0;
  
  // Take 2000 samples for calibration
  for (int i = 0; i < 2000; i++) {
    mpu.getMotion6(&AccX_Raw, &AccY_Raw, &AccZ_Raw, &GyX, &GyY, &GyZ);
    
    accX_sum += (float)AccX_Raw / 4096.0;
    accY_sum += (float)AccY_Raw / 4096.0;
    accZ_sum += (float)AccZ_Raw / 4096.0;
    
    gyroX_sum += (float)GyX / 65.5;
    gyroY_sum += (float)GyY / 65.5;
    gyroZ_sum += (float)GyZ / 65.5;
    
    if (i % 200 == 0) Serial.print(".");
    delay(1);
  }
  
  Acc_Error_X = accX_sum / 2000.0;
  Acc_Error_Y = accY_sum / 2000.0;
  Acc_Error_Z = (accZ_sum / 2000.0) - 1.0;  // Subtract 1g for Z axis
  
  Gyro_Error_X = gyroX_sum / 2000.0;
  Gyro_Error_Y = gyroY_sum / 2000.0;
  Gyro_Error_Z = gyroZ_sum / 2000.0;
  
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
  
  // Ensure consistent timing (4ms)
  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}

#endif // READ_YPR_H