#include <Arduino.h>
#include <Configuration.h>

float k_roll        = 23.5;
// float k_roll        = 0;
float k_roll_rate   = 5.5;

float k_pitch       = k_roll; 
float k_pitch_rate  = k_roll_rate;

float k_yaw         = 29.1f;
float k_yaw_rate    = 7.6f;

float alt_ref = 0, alt_sensor, alt_rate_sensor;
float roll_ref =0, pitch_ref = 0, yaw_ref = 0;
float roll_sensor, pitch_sensor, yaw_sensor, roll_rate_sensor, pitch_rate_sensor, yaw_rate_sensor;
float x_ref, y_ref, z_ref;
float x_position, y_position, z_position, x_velocity, y_velocity, z_velocity;
float heading_now, last_heading;
float u1, u2, u3, u4;
int m1,m2,m3,m4;

#ifdef CONTROL_INTEGRATOR
#define INTEGRATOR_LIMIT 2000
float roll_integrator = 0, pitch_integrator = 0,yaw_integrator = 0;
float k_roll_integrator = 23.5;
float k_pitch_integrator = 23.5;
float k_yaw_integrator = 23.5;
float dt = 0.002;

#endif



// using linear equation for the pwm to thrust
int thrust_to_pwm(float input_thrust)
{
    return input_thrust/abs(input_thrust)*(0.1704*pow(abs(input_thrust),0.6735))*10;
}


void control(int16_t chan_roll, int16_t chan_pitch, int16_t chan_yaw, int16_t chan_throttle, bool arm)
{
    #ifdef TILT_QUADROTOR
    u1 = 0;
    u2 = (-k_roll * (roll_sensor-roll_ref) + (-k_roll_rate * (roll_rate_sensor)));
    u3 = (-k_pitch * (pitch_sensor-pitch_ref) ) + (-k_pitch_rate * (pitch_rate_sensor));
    u4 = (-k_yaw * (yaw_sensor-yaw_ref)) + (-k_yaw_rate * (yaw_rate_sensor));
    #endif

    #ifdef CONTROL_INTEGRATOR
    roll_integrator  += -k_roll_integrator*(roll_sensor-roll_ref)*dt;
    pitch_integrator += -k_pitch_integrator*(pitch_sensor-pitch_ref)*dt;
    yaw_integrator   += -k_yaw_integrator*(yaw_sensor-yaw_ref)*dt;
    roll_integrator = constrain(roll_integrator, -INTEGRATOR_LIMIT, INTEGRATOR_LIMIT);
    pitch_integrator = constrain(pitch_integrator, -INTEGRATOR_LIMIT, INTEGRATOR_LIMIT);

    //integral reset system
    if(chan_throttle<1100||!arm)
    {
        roll_integrator = 0;
        pitch_integrator = 0;
        yaw_integrator = 0;
    }
    if(roll_sensor-roll_ref < 0.2 && roll_sensor-roll_ref >-0.2)
    {
        roll_integrator = 0;
    }
    if(pitch_sensor-pitch_ref <  0.2 && pitch_sensor-pitch_ref > 0.2)
    { 
        pitch_integrator = 0;
    }
    if(yaw_sensor-yaw_ref <  0.2 && yaw_sensor-yaw_ref > 0.2)
    {
        pitch_integrator = 0;
    }

    u2 = u2 + roll_integrator;
    u3 = u3 + pitch_integrator;
    u4 = u4 + yaw_integrator;
    #endif

    if (arming)
    {

        m1 = 1000 + thrust_to_pwm((ch_throttle-1000)*10+(u3/2)/ARM_LEGHT + (u4/2) ) + (ch_yaw-1500);
        m2 = 1000 + thrust_to_pwm((ch_throttle-1000)*10+(u2/2)/ARM_LEGHT - (u4/2) ) - (ch_yaw-1500);
        m3 = 1000 + thrust_to_pwm((ch_throttle-1000)*10-(u3/2)/ARM_LEGHT + (u4/2) ) + (ch_yaw-1500);
        m4 = 1000 + thrust_to_pwm((ch_throttle-1000)*10-(u2/2)/ARM_LEGHT - (u4/2) ) - (ch_yaw-1500);

    }
    else
    {
        m1 = 988;
        m2 = 988;
        m3 = 988;
        m4 = 988;
    }
}