#include <Arduino.h>

#define SONAR_TRIG 1
#define SONAR_ECHO 0
#define SOUND_SPEED 344.0f // m/s

uint32_t srf_timer = 0;
float echo_signal = 0;

float alt, alt_rate;
float prev_alt = 0;
const float sonar_offset = 5.06;
uint32_t sonar_time = 0;

void echo_rising();
void echo_falling();

void sonar_setup() {
    pinMode(SONAR_ECHO, INPUT);
    pinMode(SONAR_TRIG, OUTPUT);

    srf_timer = micros();

    attachInterrupt(SONAR_ECHO, echo_rising, RISING);

    digitalWrite(SONAR_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(SONAR_TRIG, HIGH);
    delayMicroseconds(10);

    alt = (SOUND_SPEED * echo_signal * 0.0001) / 2.0f; // cm
}

void sonar_loop() {
    digitalWrite(SONAR_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(SONAR_TRIG, HIGH);
    delayMicroseconds(10);
    
    alt = (SOUND_SPEED * echo_signal * 0.0001) / 2.0f; // cm

    if (abs(alt - prev_alt) < 50) {
        alt = (alt + prev_alt) / 2;
    }

    if (alt < 0) {
        alt = 0;
    } else if (alt > 200) {
        alt = 200;
    }

    // Velocity
    if (millis() - sonar_time > 100) {
        sonar_time = millis();
        alt_rate = (alt - prev_alt) / 0.1f;
        prev_alt = alt;
    }
}

void echo_rising()
{
  detachInterrupt(SONAR_ECHO);
  srf_timer = micros();
  attachInterrupt(SONAR_ECHO, echo_falling, FALLING);
}

void echo_falling()
{
  detachInterrupt(SONAR_ECHO);
  echo_signal = micros() - srf_timer;
  attachInterrupt(SONAR_ECHO, echo_rising, RISING);
}