#include <Arduino.h>
#include <Configuration.h>


// float k_alt         = 15.0;
// float k_alt_rate    = 6.0f;

// float k_roll        = 35.0f;
// float k_roll_rate   = 40.0;

// float k_pitch       = k_roll; 
// float k_pitch_rate  = k_roll_rate;

// float k_yaw         = 42.0f;
// float k_yaw_rate    = 9.0f;

float k_roll        = 0;
// float k_roll        = 0;
float k_roll_rate   = 0;

float k_pitch       = k_roll; 
float k_pitch_rate  = k_roll_rate;

float alt_ref = 0, alt_sensor, alt_rate_sensor;
float roll_ref =0, pitch_ref = 0, yaw_ref = 0;
float roll_sensor, pitch_sensor, yaw_sensor, roll_rate_sensor, pitch_rate_sensor, yaw_rate_sensor;
float heading_now, last_heading;
float u1, u2, u3, u4;


// using linear equation for the pwm to thrust
int thrust_to_pwm(float input_thrust)
{
    //int output = (((0.1036*input_thrust)+10)/100*1000)+1000;
    // return ((0.1036*input_thrust)+10)*10;
    return ((0.0871*input_thrust))*10;
}


void control()
{
#ifdef tilt

//u1 = (-k_alt * (alt_sensor-alt_ref)) + (-k_alt_rate * (alt_rate_sensor) / 1'000'000.0f);
u1 =0;
u2 = (-k_roll * (roll_sensor-roll_ref) + (-k_roll_rate * (roll_rate_sensor)));
u3 = (-k_pitch * (pitch_sensor-pitch_ref) ) + (-k_pitch_rate * (pitch_rate_sensor));
//u4 = (-k_yaw * (yaw_sensor-yaw_ref)) + (-k_yaw_rate * (yaw_rate_sensor));
u4=0;


#else

const double A_invers[4][4] = {{292600, -1300300,  1300300,  6283300},
                               {292600, -1300300, -1300300, -6283300},
                               {292600,  1300300, -1300300,  6283300},
                               {292600,  1300300,  1300300, -6283300}};

float omega2[4];

void control_fsfb() {

    u1 = (-k_alt * (alt_sensor-alt_ref) / 1'000'000.0f) + (-k_alt_rate * (alt_rate_sensor) / 1'000'000.0f);
    u2 = (-k_roll * (roll_sensor-roll_ref) / 10'000'000.0f) + (-k_roll_rate * (roll_rate_sensor) / 10'000'000.0f);
    u3 = (-k_pitch * (pitch_sensor-pitch_ref) / 10'000'000.0f) + (-k_pitch_rate * (pitch_rate_sensor) / 10'000'000.0f);
    u4 = (-k_yaw * (yaw_sensor-yaw_ref) / 10'000'000.0f) + (-k_yaw_rate * (yaw_rate_sensor) / 10'000'000.0f);

    omega2[0] = (A_invers[0][0]*u1 + A_invers[0][1]*u2 + A_invers[0][2]*u3 + A_invers[0][3]*u4);
    omega2[1] = (A_invers[1][0]*u1 + A_invers[1][1]*u2 + A_invers[1][2]*u3 + A_invers[1][3]*u4);
    omega2[2] = (A_invers[2][0]*u1 + A_invers[2][1]*u2 + A_invers[2][2]*u3 + A_invers[2][3]*u4);
    omega2[3] = (A_invers[3][0]*u1 + A_invers[3][1]*u2 + A_invers[3][2]*u3 + A_invers[3][3]*u4);
}

#endif
}