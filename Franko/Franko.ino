#include <Wire.h>
#include <Kalman.h>
#include <GY521.h>
#include <PID_v1.h>
#include <LMotorController.h>


//MOTOR CONTROLLER


int ENA = 3;
int IN1 = 4;
int IN2 = 2;
int IN3 = 5;
int IN4 = 7;
int ENB = 6;

LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, 1, 1);


//MPU


GY521 mpu;


//PID


double kp , ki, kd;
double prevKp, prevKi, prevKd;
double setpoint = 173.5, input, output;

PID pid(&input, &output, &setpoint, 0, 0, 0, DIRECT);


//timers


long time1Hz = 0;


//setup


void setup()
{    
    Serial.begin(115200);
    setupMPU();
    setupPID();
}


void setupMPU()
{
    mpu.setDebugMode(false);
    mpu.setup();
}


void setupPID()
{
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(1);
    pid.SetOutputLimits(-255, 255);  
}


//loop


void loop()
{
    readIMUValues();
    pid.Compute();
    motorController.move(output);

    unsigned long currentMillis = millis();
    
    if (currentMillis - time1Hz >= 1000)
    {
        loopAt1Hz();
        time1Hz = currentMillis;
    }
}


void loopAt1Hz()
{
    setPIDTuningValues();
}


//Read IMU data


void readIMUValues()
{
    mpu.readIMUData();
    input = mpu.kalAngleY+180;
}


//PID Tuning (3 potentiometers)


void setPIDTuningValues()
{
    readPIDTuningValues();
    
    if (kp != prevKp || ki != prevKi || kd != prevKd)
    {
//        Serial.print(kp);Serial.print(", ");Serial.print(ki);Serial.print(", ");Serial.println(kd);

        pid.SetTunings(kp, ki, kd);
        prevKp = kp; prevKi = ki; prevKd = kd;
    }
}


void readPIDTuningValues()
{
    int potKp = analogRead(A0);
    int potKi = analogRead(A1);
    int potKd = analogRead(A2);
        
    kp = map(potKp, 0, 1023, 0, 25000) / 100.0; //0 - 250
    ki = map(potKi, 0, 1023, 0, 25000) / 100.0; //0 - 250
    kd = map(potKd, 0, 1023, 0, 500) / 100.0; //0 - 5
}
