//hello everyone this is a first version of Software 
/*

Follow me on Social Media Account :-

Instagram :- https://www.instagram.com/creativeindia__/?hl=en
FaceBook :- https://www.facebook.com/profile.php?id=100052831652668
Youtube :- https://www.youtube.com/@PROJECTOCCUPYMARS/videos
GitHub :- https://github.com/PIEspace

Software Start Date :- 22-06-2024

*/

#include<Arduino.h>
#include<Wire.h>

//************************************************************************
//Define the MPU6050 Sensor 

#define MPU6050_ADDRESS 0X68 //Define the mpu6050 I2C address
#define RADIAN_TO_DEGREE 180.0/M_PI //convert factor for the radian to degrees

int16_t Ax , Ay , Az ; //define the accelerometer raw data
int16_t Gx , Gy , Gz ; //define the gyroscope raw data 
int16_t Temperature_Raw; //define the temperature raw data
float AccX , AccY , AccZ ; //define the accelerometer scaled data
float GyroX , GyroY , GyroZ ; //define the gyroscope scaled data
float Roll , Pitch , Yaw ; //define the Roll Pitch and Yaw Angles 
float Temperature ; //define the temperature 

//define the kalman filter variables 
float Roll_Estimate = 0;
float Pitch_Estimate = 0;
float Yaw_Estimate = 0;
float Roll_Bias = 0;
float Pitch_Bias = 0;
float Yaw_Bias = 0;
float Roll_Rate ;
float Pitch_Rate;
float Yaw_Rate;
float dt = 0.01; //Time step , assuming loop runs at 100Hz
float Q_angle = 0.001; //process noise variance for the accelerometer
float Q_bias = 0.003 ; //process noise variance for the gyroscope bias
float R_measure = 0.03; //measurement noise variance
float P[2][2] = {{1,0} , {0,1}};
float K[2]; //kalman gain 
float Y , S ;


//Function Prototypes 
void KalmanFilter(float Acc_Angles , float Gyro_Rate , float &angles , float &bias);


//************************************************************************
#define PUSHBUTTON 4 
#define BUZZER 2 



void setup(){
    //Define the bud rate so in this case i am using 230400 bits per second 
    Serial.begin(230400);
    Wire.begin();

    //Initialize MPU6050 Sensor 
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x6B) ; //PWR_MGMT_1 register
    Wire.write(0); //set to zero (wake up the mpu6050 sensor)
    Wire.endTransmission(true);

    Serial.println("MPU6050 initialized");
}

void loop(){

    //Calling the function
    MPU6050_DATA();

}

void MPU6050_DATA(){
    // Read accelerometer and gyroscope data
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, 14, true); // request a total of 14 registers

  // Read accelerometer data
  Ax = Wire.read() << 8 | Wire.read(); // Combine high and low bytes
  Ay = Wire.read() << 8 | Wire.read();
  Az = Wire.read() << 8 | Wire.read();

  // Read temperature
  Temperature_Raw = Wire.read() << 8 | Wire.read(); // Uncomment if temperature is needed

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

  // Calculate Roll, Pitch, Yaw angles from accelerometer data
  float roll_acc = atan2(AccY, AccZ) * RAD_TO_DEG;
  float pitch_acc = atan(-AccX / sqrt(AccY * AccY + AccZ * AccZ)) * RAD_TO_DEG;
  float yaw_acc = atan2(-GyroX, sqrt(GyroY * GyroY + GyroZ * GyroZ)) * RAD_TO_DEG;

  // Gyro rates
  Roll_Rate = GyroX;
  Pitch_Rate = GyroY;
  Yaw_Rate = GyroZ;

  // Apply Kalman filter
  KalmanFilter(roll_acc, Roll_Rate, Roll_Estimate, Roll_Bias);
  KalmanFilter(pitch_acc, Pitch_Rate, Pitch_Estimate, Pitch_Bias);
  KalmanFilter(yaw_acc, Yaw_Rate, Yaw_Estimate,Yaw_Bias);

  // Convert the raw temperature data to degree Celsius
  Temperature = (float)Temperature_Raw / 340.0 + 36.53;

  // Print the angles
  Serial.print(Roll_Estimate - 90); // Adjusted Roll
  Serial.print(",");
  Serial.print(Pitch_Estimate);
  Serial.print(",");
  Serial.print(Yaw_Estimate);
  Serial.print(",");
  Serial.print(Temperature);
  Serial.print(",");

  //Print the accelerometer
  Serial.print(AccX);
  Serial.print(",");
  Serial.print(AccY);
  Serial.print(",");
  Serial.print(AccZ);
  Serial.print(",");

  //define the Gyroscope
  Serial.print(GyroX);
  Serial.print(",");
  Serial.print(GyroY);
  Serial.print(",");
  Serial.println(GyroZ);



  delay(10); // Delay for stability, assuming loop runs at 100Hz

}

void KalmanFilter(float acc_angle, float gyro_rate, float &angle, float &bias) {
  // Predict
  angle += dt * (gyro_rate - bias);
  P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  // Update
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



