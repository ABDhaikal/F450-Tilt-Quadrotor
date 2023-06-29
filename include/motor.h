#include <Arduino.h>
#include <Servo.h>
#include <../src/Configuration.h>

#define MOTOR_1_PIN  6
#define MOTOR_2_PIN  5
#define MOTOR_3_PIN  4
#define MOTOR_4_PIN  3


Servo motor1, motor2, motor3, motor4;
Servo servo1, servo2, servo3, servo4;

void servo_loop(int pwm1, int pwm2, int pwm3, int pwm4);
void motor_loop(int pwm1, int pwm2, int pwm3, int pwm4);
void calibrate_motors();

void motor_setup() {

    //set pinmode servo and motor
    pinMode(MOTOR_1_PIN, OUTPUT);
    pinMode(MOTOR_2_PIN, OUTPUT);
    pinMode(MOTOR_3_PIN, OUTPUT);
    pinMode(MOTOR_4_PIN, OUTPUT);
    pinMode(servo1_pin, OUTPUT);
    pinMode(servo2_pin, OUTPUT);
    pinMode(servo3_pin, OUTPUT);
    pinMode(servo4_pin, OUTPUT);

    //attach servo and motor to pin
    motor1.attach(MOTOR_1_PIN);
    motor2.attach(MOTOR_2_PIN);
    motor3.attach(MOTOR_3_PIN);
    motor4.attach(MOTOR_4_PIN);
    servo1.attach(servo1_pin);
    servo2.attach(servo2_pin);
    servo3.attach(servo3_pin);
    servo4.attach(servo4_pin);
    #ifdef Calibrate_motor
    calibrate_motors();
    #endif
    //recenter servo
    servo_loop(1500,1500,1500,1500);
    motor_loop(MOTOR_PWM_MIN,MOTOR_PWM_MIN,MOTOR_PWM_MIN,MOTOR_PWM_MIN);
    #ifdef USB_DEBUG
    Serial.println("Motor setup complete");
    #endif
    #ifdef TELEM_DEBUG
    TELEM.println("Motor setup complete");
    #endif
    
}

void motor_loop(int pwm1, int pwm2, int pwm3, int pwm4) {
    pwm1 = constrain(pwm1, MOTOR_PWM_MIN, MOTOR_PWM_MAX);         //Constrain the pulse between 1000 and 2000us.
    pwm2 = constrain(pwm2, MOTOR_PWM_MIN, MOTOR_PWM_MAX);         //Constrain the pulse between 1000 and 2000us.
    pwm3 = constrain(pwm3, MOTOR_PWM_MIN, MOTOR_PWM_MAX);         //Constrain the pulse between 1000 and 2000us.
    pwm4 = constrain(pwm4, MOTOR_PWM_MIN, MOTOR_PWM_MAX);         //Constrain the pulse between 1000 and 2000us.
    
    motor1.writeMicroseconds(pwm1);
    motor2.writeMicroseconds(pwm2);
    motor3.writeMicroseconds(pwm3);
    motor4.writeMicroseconds(pwm4);
}

void servo_loop(int pwm1, int pwm2, int pwm3, int pwm4) {
    pwm1 = constrain(pwm1+servo1_offset, SERVO_PWM_MIN, SERVO_PWM_MAX);         //Constrain the pulse between 988 and 2012us.
    pwm2 = constrain(pwm2+servo2_offset, SERVO_PWM_MIN, SERVO_PWM_MAX);         //Constrain the pulse between 988 and 2012us.
    pwm3 = constrain(pwm3+servo3_offset, SERVO_PWM_MIN, SERVO_PWM_MAX);         //Constrain the pulse between 988 and 2012us.
    pwm4 = constrain(pwm4+servo4_offset, SERVO_PWM_MIN, SERVO_PWM_MAX);         //Constrain the pulse between 988 and 2012us.
    
    servo1.writeMicroseconds(pwm1);
    servo2.writeMicroseconds(pwm2);
    servo3.writeMicroseconds(pwm3);
    servo4.writeMicroseconds(pwm4);
}

void calibrate_motors() {
    Serial.println("Calibrating motors...");
    motor_loop(MOTOR_PWM_MAX, MOTOR_PWM_MAX, MOTOR_PWM_MAX, MOTOR_PWM_MAX); 
    delay(1000);
    motor_loop(MOTOR_PWM_MIN, MOTOR_PWM_MIN, MOTOR_PWM_MIN, MOTOR_PWM_MIN);
    delay(1000);
    Serial.println("Calibration complete");
}