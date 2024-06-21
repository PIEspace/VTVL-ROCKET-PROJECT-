<h1>Project Title: MPU6050 Sensor Data with Kalman Filter</h1>
Introduction
This code reads data from an MPU6050 sensor, which includes accelerometer and gyroscope data. It then processes this data to calculate roll, pitch, and yaw angles using a Kalman filter for more accurate results. These angles help us understand the orientation of the sensor in 3D space.

Code Breakdown
1. Including Necessary Libraries
cpp
Copy code
#include <Arduino.h>
#include <Wire.h>
Arduino.h: Includes basic functions for the Arduino.
Wire.h: Allows communication with I2C devices like the MPU6050 sensor.
2. Defining Constants and Variables
cpp
Copy code
#define MPU6050_ADDRESS 0x68 // The address of the MPU6050 sensor
#define RADIAN_TO_DEGREE 180.0 / M_PI // Conversion factor to change radians to degrees
MPU6050_ADDRESS: The I2C address of the MPU6050 sensor.
RADIAN_TO_DEGREE: Conversion factor to change radians to degrees.
cpp
Copy code
int16_t Ax, Ay, Az; // Accelerometer raw data
int16_t Gx, Gy, Gz; // Gyroscope raw data
int16_t Temperature_Raw; // Raw temperature data
float AccX, AccY, AccZ; // Accelerometer scaled data
float GyroX, GyroY, GyroZ; // Gyroscope scaled data
float Roll, Pitch, Yaw; // Angles for roll, pitch, and yaw
float Temperature; // Temperature in Celsius
Variables to store raw and scaled data from the accelerometer, gyroscope, and temperature sensor.
3. Kalman Filter Variables
cpp
Copy code
float Roll_Estimate = 0;
float Pitch_Estimate = 0;
float Yaw_Estimate = 0;
float Roll_Bias = 0;
float Pitch_Bias = 0;
float Yaw_Bias = 0;
float Roll_Rate;
float Pitch_Rate;
float Yaw_Rate;
float dt = 0.01; // Time step for 100Hz loop
float Q_angle = 0.001; // Process noise variance for the accelerometer
float Q_bias = 0.003; // Process noise variance for the gyroscope bias
float R_measure = 0.03; // Measurement noise variance
float P[2][2] = {{1, 0}, {0, 1}};
float K[2]; // Kalman gain
float Y, S;
Variables used in the Kalman filter to estimate angles and biases.
4. Function Prototypes
cpp
Copy code
void KalmanFilter(float Acc_Angles, float Gyro_Rate, float &angles, float &bias);
Declares the KalmanFilter function, which we'll explain later.
5. Setup Function
cpp
Copy code
void setup() {
    Serial.begin(230400);
    Wire.begin();
    
    // Initialize the MPU6050 sensor
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x6B); // Access the PWR_MGMT_1 register
    Wire.write(0); // Set it to zero to wake up the MPU6050 sensor
    Wire.endTransmission(true);

    Serial.println("MPU6050 initialized");
}
Initializes serial communication at a baud rate of 230400.
Wakes up the MPU6050 sensor by writing to its power management register.
6. Loop Function
cpp
Copy code
void loop() {
    MPU6050_DATA();
}
Continuously calls the MPU6050_DATA function to read and process sensor data.
7. Reading MPU6050 Data
cpp
Copy code
void MPU6050_DATA() {
    // Read accelerometer and gyroscope data
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDRESS, 14, true); // Request 14 registers

    // Read accelerometer data
    Ax = Wire.read() << 8 | Wire.read(); // Combine high and low bytes
    Ay = Wire.read() << 8 | Wire.read();
    Az = Wire.read() << 8 | Wire.read();

    // Read temperature data
    Temperature_Raw = Wire.read() << 8 | Wire.read();

    // Read gyroscope data
    Gx = Wire.read() << 8 | Wire.read();
    Gy = Wire.read() << 8 | Wire.read();
    Gz = Wire.read() << 8 | Wire.read();

    // Convert raw accelerometer data to G's
    AccX = (float)Ax / 16384.0;
    AccY = (float)Ay / 16384.0;
    AccZ = (float)Az / 16384.0;

    // Convert raw gyroscope data to degrees per second
    GyroX = (float)Gx / 131.0;
    GyroY = (float)Gy / 131.0;
    GyroZ = (float)Gz / 131.0;

    // Calculate roll, pitch, and yaw angles from accelerometer data
    float roll_acc = atan2(AccY, AccZ) * RADIAN_TO_DEGREE;
    float pitch_acc = atan(-AccX / sqrt(AccY * AccY + AccZ * AccZ)) * RADIAN_TO_DEGREE;
    float yaw_acc = atan2(-GyroX, sqrt(GyroY * GyroY + GyroZ * GyroZ)) * RADIAN_TO_DEGREE;

    // Gyro rates
    Roll_Rate = GyroX;
    Pitch_Rate = GyroY;
    Yaw_Rate = GyroZ;

    // Apply Kalman filter
    KalmanFilter(roll_acc, Roll_Rate, Roll_Estimate, Roll_Bias);
    KalmanFilter(pitch_acc, Pitch_Rate, Pitch_Estimate, Pitch_Bias);
    KalmanFilter(yaw_acc, Yaw_Rate, Yaw_Estimate, Yaw_Bias);

    // Convert raw temperature data to degrees Celsius
    Temperature = (float)Temperature_Raw / 340.0 + 36.53;

    // Print the values
    Serial.print(Roll_Estimate - 90); // Adjusted roll
    Serial.print(",");
    Serial.print(Pitch_Estimate);
    Serial.print(",");
    Serial.print(Yaw_Estimate);
    Serial.print(",");
    Serial.print(Temperature);
    Serial.print(",");

    // Print accelerometer values
    Serial.print(AccX);
    Serial.print(",");
    Serial.print(AccY);
    Serial.print(",");
    Serial.print(AccZ);
    Serial.print(",");

    // Print gyroscope values
    Serial.print(GyroX);
    Serial.print(",");
    Serial.print(GyroY);
    Serial.println(GyroZ);

    delay(10); // Delay to stabilize loop, assuming it runs at 100Hz
}
Reads data from the MPU6050 sensor.
Converts raw data to scaled values.
Calculates roll, pitch, and yaw angles.
Uses a Kalman filter to smooth these angles.
Prints the results to the serial monitor.
8. Kalman Filter Function
cpp
Copy code
void KalmanFilter(float acc_angle, float gyro_rate, float &angle, float &bias) {
    // Predict step
    angle += dt * (gyro_rate - bias);
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Update step
    Y = acc_angle - angle;
    S = P[0][0] + R_measure;
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    angle += K[0] * Y;
    bias += K[1] * Y;

    P[0][0] -= K[0] * P[0][0];
    P[0][1] -= K[0] * P[0][1];
    P[1][0] -= K[1] * P[0][0];
    P[1][1] -= K[1] * P[0][1];
}
Applies the Kalman filter to refine the angle estimates.
Predicts the new angle based on the gyro rate.
Updates the angle based on the accelerometer angle.
How It Works
Reading Sensor Data: The code reads raw data from the MPU6050 sensor, including accelerometer, gyroscope, and temperature data.

Converting Raw Data: The raw data is converted into meaningful values (G's for acceleration and degrees per second for rotation).

Calculating Angles: Using the accelerometer data, the code calculates roll, pitch, and yaw angles.

Applying the Kalman Filter: The Kalman filter refines these angles by combining the accelerometer and gyroscope data, providing more accurate estimates.

Displaying Data: The refined angles and sensor data are printed to the serial monitor, which can be viewed on a computer.

Mathematics Involved
Conversion to Degrees: The accelerometer data is converted from radians to degrees using the conversion factor RADIAN_TO_DEGREE.
Trigonometric Functions: Functions like atan2 and atan are used to calculate angles from the accelerometer data.
Kalman Filter: This is an advanced mathematical algorithm used to estimate the true value of a measured quantity by filtering out noise and inaccuracies from the measurements.
