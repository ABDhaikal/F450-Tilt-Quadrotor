#include <Arduino.h>

float k_alt         = 15.0;
float k_alt_rate    = 6.0f;

float k_roll        = 35.0f;
float k_roll_rate   = 40.0;

float k_pitch       = k_roll; 
float k_pitch_rate  = k_roll_rate;

float k_yaw         = 42.0f;
float k_yaw_rate    = 9.0f;

float alt_ref = 75, alt_sensor, alt_rate_sensor;
float roll_ref, pitch_ref, yaw_ref;
float roll_sensor, pitch_sensor, yaw_sensor, roll_rate_sensor, pitch_rate_sensor, yaw_rate_sensor;
float heading_now, last_heading;
float u1, u2, u3, u4;

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