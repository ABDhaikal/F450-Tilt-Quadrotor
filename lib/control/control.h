#include <Arduino.h>
#include <Configuration.h>
#include "parameter.h"

#ifndef CONTROL_H
#define CONTROL_H



float k_x_position= 0;
float k_x_velocity     = 0;

float k_y_position= 0;
float k_y_velocity  = 0;

float k_z_position    = 0;
float k_z_velocity = 0;


float k_roll        = 23.5;
float k_roll_rate   = 5.5;

float k_pitch       = k_roll; 
float k_pitch_rate  = k_roll_rate;

float k_yaw         = 29.1f;
float k_yaw_rate    = 7.6f;


float _roll_integrator      =0; 
float _pitch_integrator     =0;
float _yaw_integrator       =0;
float _k_roll_integrator    =0;
float _k_pitch_integrator   =0;
float _k_yaw_integrator     =0;
float _dt = dt;


float u1,u2,u3,u4, u5, u6;
int m1,m2,m3,m4;
int s1,s2,s3,s4;


void integrator(float *integrator, float *k_integrator, float error)
{
    #ifdef CONTROL_INTEGRATOR
    *integrator += *k_integrator * error * _dt;
    if(*integrator > INTEGRATOR_LIMIT)
    {
        *integrator = INTEGRATOR_LIMIT;
    }
    else if(*integrator < -INTEGRATOR_LIMIT)
    {
        *integrator = -INTEGRATOR_LIMIT;
    }
    #endif
}

// equation for the pwm to thrust
int thrust_to_pwm(float input_thrust)
{
    return input_thrust/abs(input_thrust)*(0.1704*pow(abs(input_thrust),0.6735))*10;
}

void control_setup()
{
    #ifdef CONTROL_INTEGRATOR
    _k_roll_integrator    =k_roll_integrator ;
    _k_pitch_integrator   =k_pitch_integrator;
    _k_yaw_integrator     =k_yaw_integrator  ;
    _roll_integrator = 0;
    _pitch_integrator = 0;
    _yaw_integrator = 0;
    #endif
}

void control(attitude_parameter *attitude,attitude_parameter *attitude_ref 
            ,radio_parameter *data_radio)
{
    #ifdef TILT_QUADROTOR
    u1 = 0;
    u2 = 0;
    u3 = (-k_z_position * (attitude->z_position-attitude_ref->z_position) 
        +(-k_z_velocity * (attitude->z_velocity-attitude_ref->z_velocity)));
    u4 = (-k_roll * (attitude->roll-attitude_ref->roll) 
        +(-k_roll_rate * (attitude->roll_rate-attitude_ref->roll_rate)));
    u5 = (-k_pitch * (attitude->pitch-attitude_ref->pitch) ) 
        +(-k_pitch_rate * (attitude->pitch_rate-attitude_ref->pitch_rate));
    u6 = (-k_yaw * (attitude->yaw-attitude_ref->yaw)) 
        +(-k_yaw_rate * (attitude_ref->yaw_rate-attitude_ref->yaw_rate));
    
    //calculate servo angle
    s1 = atan(2*u1/u5)*180/PI;
    s2 = atan(2*u4/u5)*180/PI;
    s3 = atan(2*u5/u1)*180/PI;
    s4 = atan(2*u6/u1)*180/PI;
    #endif

    #ifdef CONTROL_INTEGRATOR
    _roll_integrator  += -_k_roll_integrator*(attitude->roll-attitude_ref->roll)*_dt;
    _pitch_integrator += -_k_pitch_integrator*(attitude->pitch-attitude_ref->pitch)*_dt;
    _yaw_integrator   += -_k_yaw_integrator*(attitude->yaw-attitude_ref->pitch)*_dt;
    _roll_integrator = constrain(roll_integrator, -INTEGRATOR_LIMIT, INTEGRATOR_LIMIT);
    _pitch_integrator = constrain(pitch_integrator, -INTEGRATOR_LIMIT, INTEGRATOR_LIMIT);

    //integral reset system
    if(data_radio->ch_throttle<1100||!data_radio->armed)
    {
        _roll_integrator = 0;
        _pitch_integrator = 0;
        _yaw_integrator = 0;
    }
    if(attitude->roll-attitude_ref->roll < 0.2 && attitude->roll-attitude_ref->roll >-0.2)
    {
        _roll_integrator = 0;
    }
    if(attitude->pitch-attitude_ref->pitch <  0.2 && attitude->pitch-attitude_ref->pitch > 0.2)
    { 
        _pitch_integrator = 0;
    }
    if(attitude->yaw-attitude_ref->pitch <  0.2 && attitude->yaw-attitude_ref->pitch > 0.2)
    {
        _pitch_integrator = 0;
    }

    u4 = u4 + _roll_integrator;
    u5 = u5 + _pitch_integrator;
    u6 = u6 + _yaw_integrator;
    #endif

    if (data_radio->armed)
    {

        m1 = 1000 + thrust_to_pwm((data_radio->ch_throttle-MOTOR_PWM_MIN)*10
            +(u5/2)/ARM_LEGHT + (u6/2) ) + (data_radio->ch_yaw-MOTOR_PWM_CENTER);
        m2 = 1000 + thrust_to_pwm((data_radio->ch_throttle-MOTOR_PWM_MIN)*10
            +(u4/2)/ARM_LEGHT - (u6/2) ) - (data_radio->ch_yaw-MOTOR_PWM_CENTER);
        m3 = 1000 + thrust_to_pwm((data_radio->ch_throttle-MOTOR_PWM_MIN)*10
            -(u5/2)/ARM_LEGHT + (u6/2) ) + (data_radio->ch_yaw-MOTOR_PWM_CENTER);
        m4 = 1000 + thrust_to_pwm((data_radio->ch_throttle-MOTOR_PWM_MIN)*10
            -(u4/2)/ARM_LEGHT - (u6/2) ) - (data_radio->ch_yaw-MOTOR_PWM_CENTER);

    }
    else
    {
        m1 = MOTOR_PWM_MIN;
        m2 = MOTOR_PWM_MIN;
        m3 = MOTOR_PWM_MIN;
        m4 = MOTOR_PWM_MIN;
    }
}

void get_setpoint(attitude_parameter *attitude_ref,radio_parameter *data_radio)
{
    if(data_radio->ch_roll>(RADIO_PWM_CENTER+10)||data_radio->ch_roll<(RADIO_PWM_CENTER+10))
    {
      attitude_ref->roll =  (abs((float)data_radio->ch_roll-RADIO_PWM_CENTER)/500*LIMIT_ROLL)
                            *(data_radio->ch_roll-RADIO_PWM_CENTER)/abs((float)data_radio->ch_roll-RADIO_PWM_CENTER);
    }
    else
    {
      attitude_ref->roll =0;
    }

    if(data_radio->ch_pitch>(RADIO_PWM_CENTER+10)||data_radio->ch_pitch<(RADIO_PWM_CENTER+10))
    {
      attitude_ref->pitch =  -(abs((float)data_radio->ch_pitch-RADIO_PWM_CENTER)/500*LIMIT_PITCH)
                            *(data_radio->ch_pitch-RADIO_PWM_CENTER)/abs((float)data_radio->ch_pitch-RADIO_PWM_CENTER) ;
    }
    else
    {
      attitude_ref->pitch = 0;
    }
  
}

#endif