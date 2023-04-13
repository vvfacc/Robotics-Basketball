#ifndef Driving_h
#define Driving_h

/*This library is a module of driving the robot. This contains functions:
go straight, turn left, turn right, self rotati*/
#include "SimpleRSLK.h"

// void printTest()
// {
//     Serial.print("Test");
//     delay(100);
// }

// Function driving straight forward
void drive_foward(uint8_t motorSpeed, float time= 0)
{
    Serial.println("DRVING FOWARD...");
    enableMotor(BOTH_MOTORS);
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
    setMotorSpeed(BOTH_MOTORS, motorSpeed);
    if(time != 0)
    {
        delay(time * 1000);
        disableMotor(BOTH_MOTORS);
    }
}

void turn_left_90(uint8_t motorSpeed)
{
    //Perfrom turn left
    Serial.println("TURN LEFT...");
    disableMotor(BOTH_MOTORS);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(BOTH_MOTORS, motorSpeed);
    enableMotor(BOTH_MOTORS);
    delay(1500);
    disableMotor(BOTH_MOTORS);
}

void turn_right_90(uint8_t motorSpeed)
{
    //Perfrom turn left
    Serial.println("TURN RIGHT...");
    disableMotor(BOTH_MOTORS);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorSpeed(BOTH_MOTORS, motorSpeed);
    enableMotor(BOTH_MOTORS);
    delay(1500);
    disableMotor(BOTH_MOTORS);
}

void self_rotating(uint8_t motorSpeed)
{
    // Perform Self rotating
    Serial.println("SELF ROTATING...");
    disableMotor(BOTH_MOTORS);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorSpeed(BOTH_MOTORS, motorSpeed);
    enableMotor(BOTH_MOTORS);
    delay(5900);
    disableMotor(BOTH_MOTORS);
}

void stop()
{
    Serial.println("STOP...");
    disableMotor(BOTH_MOTORS);
}
#endif