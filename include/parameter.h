#include <Arduino.h>
#include <Configuration.h>

#ifndef USE_PARAMETER
#define USE_PARAMETER
 struct attitude_parameter {
    float roll=0;
    float pitch=0;
    float yaw=0;
    float roll_rate=0;
    float pitch_rate=0;
    float yaw_rate=0;
    float x_position=0;
    float y_position=0;
    float z_position=0;
    float x_velocity=0;
    float y_velocity=0;
    float z_velocity=0;
};

struct radio_parameter{
    int16_t ch_roll;
    int16_t ch_pitch;
    int16_t ch_yaw;
    int16_t ch_throttle;
    bool armed;
    int16_t aux1;
    int16_t aux2;
};

#endif