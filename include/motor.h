#include <Arduino.h>
#include <Servo.h>
#include <../src/Configuration.h>

#define MOTOR_1_PIN  6
#define MOTOR_2_PIN  5
#define MOTOR_3_PIN  4
#define MOTOR_4_PIN  3

Servo motor1, motor2, motor3, motor4;
Servo servo1, servo2, servo3, servo4;

void motor_setup() {
    pinMode(motor1_pin, OUTPUT);
    pinMode(motor2_pin, OUTPUT);
    pinMode(motor3_pin, OUTPUT);
    pinMode(motor4_pin, OUTPUT);
    pinMode(servo1_pin, OUTPUT);
    pinMode(servo2_pin, OUTPUT);
    pinMode(servo3_pin, OUTPUT);
    pinMode(servo4_pin, OUTPUT);

    motor1.attach(motor1_pin);
    motor2.attach(motor2_pin);
    motor3.attach(motor3_pin);
    motor4.attach(motor4_pin);
    servo1.attach(servo1_pin);
    servo2.attach(servo2_pin);
    servo3.attach(servo3_pin);
    servo4.attach(servo4_pin);

    #ifdef PC_Debug
    Serial.println("Motor setup complete");
    #endif
    #ifdef Telem
    TELEM.println("Motor setup complete");
    #endif
}

void motor_loop(int pwm1, int pwm2, int pwm3, int pwm4) {
    pwm1 = constrain(pwm1, motor_pwm_min, motor_pwm_max);         //Constrain the pulse between 1000 and 2000us.
    pwm2 = constrain(pwm2, motor_pwm_min, motor_pwm_max);         //Constrain the pulse between 1000 and 2000us.
    pwm3 = constrain(pwm3, motor_pwm_min, motor_pwm_max);         //Constrain the pulse between 1000 and 2000us.
    pwm4 = constrain(pwm4, motor_pwm_min, motor_pwm_max);         //Constrain the pulse between 1000 and 2000us.
    
    motor1.writeMicroseconds(pwm1);
    motor2.writeMicroseconds(pwm2);
    motor3.writeMicroseconds(pwm3);
    motor4.writeMicroseconds(pwm4);
}

void servo_loop(int pwm1, int pwm2, int pwm3, int pwm4) {
    pwm1 = constrain(pwm1+servo1_offset, servo_pwm_min, servo_pwm_max);         //Constrain the pulse between 988 and 2012us.
    pwm2 = constrain(pwm2+servo2_offset, servo_pwm_min, servo_pwm_max);         //Constrain the pulse between 988 and 2012us.
    pwm3 = constrain(pwm3+servo3_offset, servo_pwm_min, servo_pwm_max);         //Constrain the pulse between 988 and 2012us.
    pwm4 = constrain(pwm4+servo4_offset, servo_pwm_min, servo_pwm_max);         //Constrain the pulse between 988 and 2012us.
    
    servo1.writeMicroseconds(pwm1);
    servo2.writeMicroseconds(pwm2);
    servo3.writeMicroseconds(pwm3);
    servo4.writeMicroseconds(pwm4);
}

void calibrate_motors() {
    Serial.println("Calibrating motors...");
    motor_loop(2000, 2000, 2000, 2000); 
    delay(1000);
    motor_loop(1000, 1000, 1000, 1000);
    delay(1000);
    Serial.println("Calibration complete");
}