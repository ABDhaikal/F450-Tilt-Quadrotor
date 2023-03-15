#include <Arduino.h>
#include <Servo.h>

#define MOTOR_1_PIN  6
#define MOTOR_2_PIN  5
#define MOTOR_3_PIN  4
#define MOTOR_4_PIN  3

Servo motor1, motor2, motor3, motor4;

void motor_setup() {
    pinMode(MOTOR_1_PIN, OUTPUT);
    pinMode(MOTOR_2_PIN, OUTPUT);
    pinMode(MOTOR_3_PIN, OUTPUT);
    pinMode(MOTOR_4_PIN, OUTPUT);

    motor1.attach(MOTOR_1_PIN);
    motor2.attach(MOTOR_2_PIN);
    motor3.attach(MOTOR_3_PIN);
    motor4.attach(MOTOR_4_PIN);
    
    motor1.writeMicroseconds(1000);
    motor2.writeMicroseconds(1000);
    motor3.writeMicroseconds(1000);
    motor4.writeMicroseconds(1000);
    delay(1000);

    Serial.println("Motor setup complete");
}

void motor_loop(int pwm1, int pwm2, int pwm3, int pwm4) {
    pwm1 = constrain(pwm1, 1000, 2000);                                                    //Constrain the pulse between 1000 and 2000us.
    pwm2 = constrain(pwm2, 1000, 2000);                                                  //Constrain the pulse between 1000 and 2000us.
    pwm3 = constrain(pwm3, 1000, 2000);                                                    //Constrain the pulse between 1000 and 2000us.
    pwm4 = constrain(pwm4, 1000, 2000);   
    
    motor1.writeMicroseconds(pwm1);
    motor2.writeMicroseconds(pwm2);
    motor3.writeMicroseconds(pwm3);
    motor4.writeMicroseconds(pwm4);
}

void calibrate_motors() {
    Serial.println("Calibrating motors...");
    motor_loop(2000, 2000, 2000, 2000);
    delay(1000);
    motor_loop(1000, 1000, 1000, 1000);
    delay(1000);
    Serial.println("Calibration complete");
}