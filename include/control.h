#include <Arduino.h>
#include <Configuration.h>

float k_roll        = 23.5;
// float k_roll        = 0;
float k_roll_rate   = 5.5;

float k_pitch       = k_roll; 
float k_pitch_rate  = k_roll_rate;

float k_yaw         = 20.0f;
float k_yaw_rate    = 1.5f;

float alt_ref = 0, alt_sensor, alt_rate_sensor;
float roll_ref =0, pitch_ref = 0, yaw_ref = 0;
float roll_sensor, pitch_sensor, yaw_sensor, roll_rate_sensor, pitch_rate_sensor, yaw_rate_sensor;
float heading_now, last_heading;
float u1, u2, u3, u4;

#ifdef integrator
float roll_integrator = 0, pitch_integrator = 0;
#define INTEGRATOR_LIMIT 1400
float k_roll_integrator = 0.0;
float k_pitch_integrator = 0.0;
int dt =0.002;

#endif



// using linear equation for the pwm to thrust
int thrust_to_pwm(float input_thrust)
{
    return ((0.0091*input_thrust))*10;
}


void control()
{
    #ifdef tilt
    u1 =0;
    u2 = (-k_roll * (roll_sensor-roll_ref) + (-k_roll_rate * (roll_rate_sensor)));
    u3 = (-k_pitch * (pitch_sensor-pitch_ref) ) + (-k_pitch_rate * (pitch_rate_sensor));
    u4 = (-k_yaw * (yaw_sensor-yaw_ref)) + (-k_yaw_rate * (yaw_rate_sensor));
    #endif

    #ifdef integrator
        roll_integrator  += k_roll_integrator*(roll_sensor-roll_ref * dt);
        pitch_integrator += k_pitch_integrator*(pitch_sensor-pitch_ref * dt);
        roll_integrator = constrain(roll_integrator, -INTEGRATOR_LIMIT, INTEGRATOR_LIMIT);
        pitch_integrator = constrain(pitch_integrator, -INTEGRATOR_LIMIT, INTEGRATOR_LIMIT);
        u2 = u2 + roll_integrator;
        u3 = u3 + pitch_integrator;
    #endif
}