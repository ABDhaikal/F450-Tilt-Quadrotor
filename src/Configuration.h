/**
 * @file Configuration.h
 * @author Haikal Abdurrahman(haikalabdurrahman95@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-04-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once
/**
 * @brief Timing
 * 
 */
uint32_t loop_timer = 0;
uint32_t debug_timer = 0;



/**
 * @brief 
 * 
 */
#define tilt
#define integrator

/**
 * @brief Computer Debug
 * 
 */
#define PC_Debug //computer debugging
#ifdef PC_Debug
#define PC Serial
#define PC_Baudrate 115200
#endif

/**
 * @brief GPS Configuration
 * 
 */
//#define GPS_ //using gps for location detection
#ifdef GPS_
    #include <TinyGPSPlus.h>
    #define GPS Serial3
    #define GPS_Bautrate 9600
    TinyGPSPlus gps;
#endif

/**
 * @brief Telemetry monitoring
 * 
 */
#define Telem
#ifdef Telem
#define TELEM Serial5
#define TELEM_Baudrate 57600
#endif

/**
 * @brief quad motor
 *  
 */
// #define Calibrate_motor

int motor1_pin = 0;
int motor2_pin = 2;
int motor3_pin = 4;
int motor4_pin = 6;
int motor_pwm_min = 988;
int motor_pwm_max = 2012;

/**
 * @brief motor servo 
 * 
 */
int servo1_pin = 1;
int servo2_pin = 3;
int servo3_pin = 9;
int servo4_pin = 5;
int servo1_offset = 70;
int servo2_offset = 60;
int servo3_offset = -10;
int servo4_offset = -10;

int servo_pwm_min = 988;
int servo_pwm_max = 2012;


//for tunning servos
// int pwms1 = 1500;
// int pwms2 = 1500;
// int pwms3 = 1500;
// int pwms4 = 1500;

// measurment 
float armleght = 0.23; //m
float mass = 1.76; //kg

#define dmp

#define update_rate 2000 //on milisecond