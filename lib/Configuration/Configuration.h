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

#include <Arduino.h>
// #ifndef CONFIGURATION_H
// #define CONFIGURATION_H

/**
 * @brief Timing
 * 
 */

#define UPDATE_RATE 2000 //on microsecond
#define UPDATE_RATE_S (UPDATE_RATE/1000000.0) //on second


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
#define USBPRINTTAB USB.print(" ")
#define USBPRINT2(x,y) USB.print(x);USB.print(y)
#define USBPRINT2TAB(x,y) USB.print(x);USB.print(y);USB.print("\t")
#endif

/**
 * @brief Telemetry monitoring
 * 
 */
#define TELEM_DEBUG
#ifdef TELEM_DEBUG
#define TELEM Serial5
#define TELEM_BAUDRATE 57600
#define TELEMPRINT(x) TELEM.print(x)
#define TELEMPRINTLN(x) TELEM.println(x)
#define TELEMPRINTTAB TELEM.print(" ")
#define TELEMPRINT2(x,y) TELEM.print(x);TELEM.print(y)
#define TELEMPRINT2TAB(x,y) TELEM.print(x);TELEM.print(y);TELEM.print("\t")
#endif

/**
 * @brief DEBUG SETTINGS
 * 
 */
#define DEBUG_FREQ 25 //in Hz
#define DEBUG_CYCLE (1000000/DEBUG_FREQ) //in microsecond

#if defined(USB_DEBUG) && defined(TELEM_DEBUG)
    #define USEDEBUG
    #define DEBUG(x) TELEM.print(x);USB.print(x)
    #define DEBUG2(x,y) TELEM.print(x);TELEM.print(y);USB.print(x);USB.print(y)
    #define DEBUGLN(x) TELEM.println(x);USB.println(x)
    #define DEBUGTAB() TELEM.print(" ");USB.print(" ")
    #define DEBUG2TAB(x,y) TELEM.print(x);TELEM.print(y);TELEM.print("\t");USB.print(x);USB.print(y),USB.print("\t")
#elif (!defined(TELEM_DEBUG) && defined(USB_DEBUG))
    #define USEDEBUG
    #define DEBUG(x) USB.print(x)
    #define DEBUGLN(x) USB.println(x)
    #define DEBUGTAB(x) USB.print(x);USB.print("\t")
    #define DEBUG2(x,y) USB.print(x);USB.print(y)
    #define DEBUG2TAB(x,y) USB.print(x);USB.print(y);USB.print("\t")

#elif !defined(USB_DEBUG) && (defined(TELEM_DEBUG))
    #define USEDEBUG
    #define DEBUG(x) TELEM.print(x)
    #define DEBUGLN(x) TELEM.println(x)
    #define DEBUGTAB TELEM.print(" ")
    #define DEBUG2(x,y) TELEM.print(x);TELEM.print(y)
    #define DEBUG2T(x,y) TELEM.print(x);TELEM.print(y);TELEM.print("\t")
#endif

#define RADIO_PWM_MAX 2012
#define RADIO_PWM_MIN 988
#define RADIO_PWM_CENTER 1500


/**
 * @brief GPS Configuration
 * 
 */
// #define GPS_ //using gps for location detection
#ifdef GPS_
    #define GPS Serial3
    #define GPS_BAUDRATE 9600
#endif

/**************** ACTUATOR CONFIGURATION ****************/
/**
 * @brief brushless motor
 *  
 */

#define force_constant 0.000132f
#define torque_constant 0.00000683f
#define MAX_THRUST_MOTOR 1185.0f
// #define MAX_THRUST_MOTOR 1360.0f

// #define Calibrate_motor

#define MOTOR_1_PIN  0
#define MOTOR_2_PIN  2
#define MOTOR_3_PIN  4
#define MOTOR_4_PIN  6
#define MOTOR_PWM_MIN  988
#define MOTOR_PWM_CENTER  1500
#define MOTOR_PWM_MAX  2012

/**
 * @brief motor servo 
 * 
 */

#define servo1_pin  1
#define servo2_pin  3
#define servo3_pin  5
#define servo4_pin  9

#define servo1_offset  90
#define servo2_offset  168
#define servo3_offset  5
#define servo4_offset  35

#define SERVO_PWM_MIN  988
#define SERVO_PWM_CENTER  1500
#define SERVO_PWM_MAX  2012

#define SERVO_ANGLE_MIN  -30
#define SERVO_ANGLE_MAX  30


/**************** MEASUREMENT CONFIGURATION ****************/ 

/**
 * @brief typical quadrotor
 * 
 */
#define ARM_LEGHT 0.23 //m
#define VEHICLE_MASS 1.7 //kg
#define RAD2DEG 180/PI
#define DEG2RAD PI/180
/**
 * @brief INERTIAL SENSOR CONFIGURATION
 * 
 */
// #define dmp
#define DMP_INTERRUPT_PIN 22 
#ifndef dmp
// #define LPF_ACCEL
// #define LPF_GYRO
#define KALMAN_IMU
#define MADGWICK_IMU
#ifdef LPF_ACCEL
    #define DEFAULT_ACCEL_FILTER_FREQ 25
    #define DEFAULT_ACCEL_FILTER_SAMPLE_FREQ 1000000.0f/UPDATE_RATE
#endif
#ifdef LPF_GYRO
    #define DEFAULT_GYRO_FILTER_FREQ 25
    #define DEFAULT_GYRO_FILTER_SAMPLE_FREQ 1000000.0f/UPDATE_RATE
#endif

#if defined(LPF_ACCEL)||defined(LPF_GYRO)
    #define LPF_IMU
#endif
#endif 

//for embeded magnetometer
#define USE_BYPASS_IMU


#define SCALE_FACTOR_GYRO 16.4
#define SCALE_FACTOR_ACCEL 16384
#define GRAVITY 9.80665
#define dmp_update_boost // upgrade data from imu when use dmp to get higher update rate
#define calibrate_imu


/**
 * @brief BAROMETER CONFIGURATION
 * 
 */
// #define USE_BARO

/**************** CONTROL CONFIGURATION ****************/ 
#define CONTROL_INTEGRATOR

#ifdef CONTROL_INTEGRATOR
#define INTEGRATOR_LIMIT 2000
#define roll_integrator 0 
#define pitch_integrator  0
#define yaw_integrator 0
#define k_roll_integrator  23.5/2
#define k_pitch_integrator  23.5/2
#define k_yaw_integrator  0

#endif

#define dt 0.002
#define LIMIT_ROLL  20
#define LIMIT_PITCH 20 
#define LIMIT_YAW_RATE 15 

// #define roll_offset  0
// #define pitch_offset 0
// #define yaw_offset   0

#define AUX_SETPOINT

#ifndef AUX_SETPOINT
#define X_POSITION_CONTROLLER
// #define Y_POSITION_CONTROLLER
#define ROLL_CONTROLLER
// #define PITCH_CONTROLLER
#endif
#define Z_POSITION_CONTROLLER
#define YAW_CONTROLLER

// #endif