#include "Arduino.h"
#include "remote.h"
#include "motor.h"
#include "imu.h"
#include "sonar.h"
#include "control.h"

#define UART Serial
#define BAUD_RATE 57600

uint32_t loop_timer = 0, debug_timer = 0;

int pwm1, pwm2, pwm3, pwm4;

void debug();
void tuning();

void setup() {
  UART.begin(BAUD_RATE);
  delay(250);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  remote_setup();
  motor_setup();
  imu_setup();
  sonar_setup();

  digitalWrite(LED_BUILTIN, LOW);
  UART.println("Starting loop...");

  loop_timer = micros();
}

void loop() {
  imu_loop();
  sonar_loop();

  remote_loop();

  roll_ref = (ch_roll - 1500) / 10.0f; // limit roll_ref to 50 deg
  pitch_ref = (ch_pitch - 1500) / 10.0f; // limit pitch_ref to 50 deg
  yaw_ref = (ch_yaw - 1500) / 250.0f; // limit yaw_ref to 10 deg

  alt_sensor = alt;
  roll_sensor = roll;
  pitch_sensor = pitch;
  yaw_sensor = yaw - yaw_offset;

  alt_rate_sensor = alt_rate;
  roll_rate_sensor = gyroX;
  pitch_rate_sensor = gyroY;
  yaw_rate_sensor = gyroZ;
  
  control_fsfb();

  if (arming) {
    if (ch_throttle > 1800) ch_throttle = 1800; //Limit the throttle to 1800us.
      pwm1 = (int) omega2[0] + ch_throttle;                                                  //Calculate the pulse for esc 1 (front-right - CCW).
      pwm2 = (int) omega2[1] + ch_throttle;                                                  //Calculate the pulse for esc 2 (rear-right - CW).
      pwm3 = (int) omega2[2] + ch_throttle;                                                  //Calculate the pulse for esc 3 (rear-left - CCW).
      pwm4 = (int) omega2[3] + ch_throttle;                                                  //Calculate the pulse for esc 4 (front-left - CW).
  }
  else {
    yaw_offset = yaw;

    pwm1 = 1000;
    pwm2 = 1000;
    pwm3 = 1000;
    pwm4 = 1000;
  }

  motor_loop(pwm1, pwm2+3, pwm3+1, pwm4+2);

  if (micros() - debug_timer > 100000) {
    debug();
    debug_timer = micros();
  }
}

void debug() {
  String str = "";
  str += "althold:";
  str += alt_hold_mode;
  str += " Alt:";
  str += alt_sensor;
  str += " ";
  str += alt_rate_sensor;
  str += " Roll:";
  str += roll_sensor;
  str += " ";
  str += roll_rate_sensor;
  str += " Pitch:";
  str += pitch_sensor;
  str += " ";
  str += pitch_rate_sensor;
  str += " Yaw:";
  str += yaw_sensor;
  str += " ";
  str += yaw_rate_sensor;
  // str += " PWM:";
  // str += pwm1;
  // str += " ";
  // str += pwm2;
  // str += " ";
  // str += pwm3;
  // str += " ";
  // str += pwm4;
  
  UART.println(str);
}