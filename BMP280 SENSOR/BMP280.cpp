#include <Wire.h>
#include <Arduino.h>

// BMP280 I2C address
#define BMP280_ADDRESS 0x76

// BMP280 registers
#define BMP280_REG_CONTROL 0xF4
#define BMP280_REG_RESULT_PRESSURE 0xF7
#define BMP280_REG_RESULT_TEMPERATURE 0xFA

// Calibration parameters
uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

int32_t t_fine;

// Kalman filter variables
float kalmanAltitude = 0.0;
float kalmanP = 1.0;  // Estimation error covariance
float kalmanQ = 0.001;  // Process noise covariance
float kalmanR = 0.1;  // Measurement noise covariance

// Function prototypes
void readCalibrationParameters();
void readSensorData(int32_t &temp, int32_t &press);
float compensateTemperature(int32_t adc_T);
float compensatePressure(int32_t adc_P);
float calculateAltitude(float pressure);
float kalmanFilter(float altitude);

void setup() {
    Serial.begin(230400);
    Wire.begin();
    
    // Start communication with BMP280
    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.write(0xD0);  // BMP280 chip ID register
    Wire.endTransmission();
    Wire.requestFrom(BMP280_ADDRESS, 1);
    if (Wire.read() != 0x58) {
        Serial.println("BMP280 not found!");
        while (1);
    }
    
    // Read calibration parameters
    readCalibrationParameters();
    
    // Configure BMP280
    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.write(BMP280_REG_CONTROL);
    Wire.write(0x3F);  // Set normal mode, temp and pressure oversampling = 1
    Wire.endTransmission();
}

void loop() {
    int32_t temp_raw, press_raw;
    
    // Read raw data from sensor
    readSensorData(temp_raw, press_raw);
    
    // Compensate temperature and pressure
    float temperature = compensateTemperature(temp_raw);
    float pressure = compensatePressure(press_raw);
    
    // Calculate altitude in cm
    float altitude = calculateAltitude(pressure);
    
    // Apply Kalman filter to the altitude
    float filteredAltitude = kalmanFilter(altitude);
    
    // Print the values
    Serial.print("Temperature: ");
    Serial.print(temperature / 100.0); // Adjusted for correct temperature
    Serial.print(" °C, Pressure: ");
    Serial.print(pressure);
    Serial.print(" Pa, Altitude: ");
    Serial.print(filteredAltitude);
    Serial.println(" cm");
    
    delay(100);
}

void readCalibrationParameters() {
    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.write(0x88);  // Start of calibration parameters
    Wire.endTransmission();
    Wire.requestFrom(BMP280_ADDRESS, 24);
    
    dig_T1 = Wire.read() | (Wire.read() << 8);
    dig_T2 = Wire.read() | (Wire.read() << 8);
    dig_T3 = Wire.read() | (Wire.read() << 8);
    dig_P1 = Wire.read() | (Wire.read() << 8);
    dig_P2 = Wire.read() | (Wire.read() << 8);
    dig_P3 = Wire.read() | (Wire.read() << 8);
    dig_P4 = Wire.read() | (Wire.read() << 8);
    dig_P5 = Wire.read() | (Wire.read() << 8);
    dig_P6 = Wire.read() | (Wire.read() << 8);
    dig_P7 = Wire.read() | (Wire.read() << 8);
    dig_P8 = Wire.read() | (Wire.read() << 8);
    dig_P9 = Wire.read() | (Wire.read() << 8);
}

void readSensorData(int32_t &temp, int32_t &press) {
    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.write(BMP280_REG_RESULT_PRESSURE);
    Wire.endTransmission();
    Wire.requestFrom(BMP280_ADDRESS, 6);
    
    press = (Wire.read() << 12) | (Wire.read() << 4) | (Wire.read() >> 4);
    temp = (Wire.read() << 12) | (Wire.read() << 4) | (Wire.read() >> 4);
}

float compensateTemperature(int32_t adc_T) {
    int32_t var1, var2;
    
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    
    t_fine = var1 + var2;
    
    return (t_fine * 5 + 128) >> 8;
}

float compensatePressure(int32_t adc_P) {
    int64_t var1, var2, p;
    
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
    
    if (var1 == 0) {
        return 0;  // Avoid division by zero
    }
    
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    
    return (float)p / 256.0;
}

float calculateAltitude(float pressure) {
    return 44330.0 * (1.0 - pow(pressure / 101325.0, 0.1903)) * 100;  // in cm
}

float kalmanFilter(float altitude) {
    // Prediction
    kalmanP += kalmanQ;

    // Kalman Gain
    float K = kalmanP / (kalmanP + kalmanR);

    // Correction
    kalmanAltitude += K * (altitude - kalmanAltitude);
    kalmanP *= (1 - K);

    return kalmanAltitude;
}
