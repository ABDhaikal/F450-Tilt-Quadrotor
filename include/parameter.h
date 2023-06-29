#include <Arduino.h>
#include <../src/Configuration.h>

static struct {
    float roll;
    float pitch;
    float yaw;
    float altitude;
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
    float altitude_rate;
    
    int16_t channel_roll;
    int16_t channel_pitch;
    int16_t channel_yaw;
    int16_t channel_throttle;
} rc;