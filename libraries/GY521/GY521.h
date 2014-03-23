#include "Arduino.h"
#include <Wire.h>
#include <Kalman.h>


#ifndef GY521_h
#define GY521_h


class GY521
{
public:
    GY521();
    ~GY521();
    void setup();
    void readIMUData();
    
    double accXangle; // Angle calculate using the accelerometer
    double accYangle;
    double gyroXangle; // Angle calculate using the gyro
    double gyroYangle;
    double compAngleX; // Calculated angle using a complementary filter
    double compAngleY;
    double kalAngleX; // Calculate the angle using a Kalman filter
    double kalAngleY;
    
    void setDebugMode(bool debug);
    bool isDebugMode();
protected:
    uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop);
    uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop);
    uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes);
    
    Kalman *kalmanX;
    Kalman *kalmanY;
    
    double accX;
    double accY;
    double accZ;
    int16_t tempRaw;
    double gyroX;
    double gyroY;
    double gyroZ;
    
    uint8_t i2cData[14];
    
    double temp;
    
    uint32_t timer;
    
    boolean debugMode;
    
    uint8_t IMUAddress;
    uint16_t I2C_TIMEOUT;
};


#endif