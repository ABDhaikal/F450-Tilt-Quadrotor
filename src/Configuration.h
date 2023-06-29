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
#define UPDATE_RATE 2000 //on microsecond


/**
 * @brief type of quadrotor
 * 
 */
#define TILT_QUADROTOR

/**************** COMUNICATION CONFIGURATION ****************/

/**
 * @brief Computer Debug
 * 
 */
#define USB_DEBUG //computer debuggings
#ifdef USB_DEBUG
#define USB Serial
#define USB_BAUDRATE 57600
#define USBPRINT(x) USB.print(x)
#define USBPRINTLN(x) USB.println(x)
#define USBENDL USB.println()
#define USBPRINTTAB USB.print(" ")
#define USBPRINT2(x,y) USB.print(x);USB.print(y)
#endif

/**
 * @brief GPS Configuration
 * 
 */
//#define GPS_ //using gps for location detection
#ifdef GPS_
    #include <TinyGPSPlus.h>
    #define GPS Serial3
    #define GPS_BAUDRATE 9600
    TinyGPSPlus gps;
#endif

/**
 * @brief Telemetry monitoring
 * 
 */
#define TELEM_DEBUG
#ifdef TELEM_DEBUG
#define TELEM Serial3
#define TELEM_BAUDRATE 57600
#define TELEM_PRINT(x) TELEM.print(x)
#define TELEM_PRINTLN(x) TELEM.println(x)
#define TELEM_ENDL TELEM.println()
#define TELEM_PRINTTAB TELEM.print(" ")
#define TELEM_PRINT2(x,y) TELEM.print(x);TELEM.print(y)
#endif

#ifdef USB_DEBUG&&TELEM_DEBUG
#define DEBUG(x) TELEM.print(x);USB.print(x)
#define DEBUGLN(x) TELEM.println(x);USB.println(x)
#define DEBUGENDL TELEM.println();USB.println()
#define DEBUGTAB TELEM.print(" ");USB.print(" ")
#define DEBUG2(x,y) TELEM.print(x);TELEM.print(y);USB.print(x);USB.print(y)
#else
#ifdef TELEM_DEBUG &! USB_DEBUG
#define DEBUG(x) USB.print(x)
#define DEBUGLN(x) USB.println(x)
#define DEBUGENDL USB.println()
#define DEBUGTAB USB.print(" ")
#define DEBUG2(x,y) USB.print(x);USB.print(y)
#else
#ifdef USB_DEBUG&!TELEM_DEBUG
#define DEBUG(x) TELEM.print(x)
#define DEBUGLN(x) TELEM.println(x)
#define DEBUGENDL TELEM.println()
#define DEBUGTAB TELEM.print(" ")
#define DEBUG2(x,y) TELEM.print(x);TELEM.print(y)
#endif
#endif
#endif



/**************** ACTUATOR CONFIGURATION ****************/
/**
 * @brief brushless motor
 *  
 */
// #define Calibrate_motor

#define MOTOR_1_PIN  0
#define MOTOR_2_PIN  2
#define MOTOR_3_PIN  4
#define MOTOR_4_PIN  6
#define MOTOR_PWM_MIN  988
#define MOTOR_PWM_MAX  2012

/**
 * @brief motor servo 
 * 
 */

#define servo1_pin  1
#define servo2_pin  3
#define servo3_pin  9
#define servo4_pin  5

#define servo1_offset  70
#define servo2_offset  60
#define servo3_offset  -10
#define servo4_offset  -10

#define SERVO_PWM_MIN  988
#define SERVO_PWM_MAX  2012



/**************** MEASUREMENT CONFIGURATION ****************/ 

/**
 * @brief typical quadrotor
 * 
 */
#define ARM_LEGHT 0.23 //m
#define VEHICLE_MASS 1.7 //kg

/**
 * @brief INERTIAL SENSOR CONFIGURATION
 * 
 */
#define dmp
#ifdef dmp
#define DMP_INTERRUPT_PIN 22 
#else 
#define LPF_ACCEL

#ifdef LPF_ACCEL
    float DEFAULT_ACCEL_FILTER_FREQ =25;
    float DEFAULT_ACCEL_FILTER_SAMPLE_FREQ =1000000.0f/UPDATE_RATE;
#endif
#define LPF_GYRO
#ifdef LPF_GYRO
    float DEFAULT_GYRO_FILTER_FREQ =25;
    float DEFAULT_GYRO_FILTER_SAMPLE_FREQ =1000000.0f/UPDATE_RATE;
#endif
#endif 

// #define kalman_imu
#define SCALE_FACTOR_GYRO 16.4
#define SCALE_FACTOR_ACCEL 16384
#define dmp_update_boost // upgrade data from imu when use dmp to get higher update rate
#define calibrate_imu


/**************** CONTROL CONFIGURATION ****************/ 
#define CONTROL_INTEGRATOR
float LIMIT_ROLL=10;
float LIMIT_PITCH=10; 

float roll_ref_off=0;
float pitch_ref_off=2;