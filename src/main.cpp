#include "Arduino.h"
#include "Configuration.h"
#include "remote.h"
#include "motor.h"
#include "imu.h"
#include "sonar.h"
#include "control.h"


//input pwm permotor
int m1,m2,m3,m4;
void debug();

void setup() {
  
  #ifdef Telem
  TELEM.begin(TELEM_Baudrate);
  #endif 

  #ifdef PC_Debug
  PC.begin(PC_Baudrate);
  #endif

  #ifdef GPS_
  GPS.begin(GPS_Bautrate);
  delay(250);
  #endif

  remote_setup();
  motor_setup();
  #ifdef Calibrate_motor
    calibrate_motors();
  #endif
  imu_setup();
  //sonar_setup();

  digitalWrite(LED_BUILTIN, LOW);
  loop_timer = micros();
}

void loop() {
  imu_loop();
  remote_loop();
  control();


  roll_sensor = roll;
  yaw_sensor = yaw;
  pitch_sensor = pitch;
  roll_rate_sensor = gyro.x/4;
  yaw_rate_sensor = -gyro.z/4;
  pitch_rate_sensor = -gyro.y/4;

  if (arming)
  {
    m1 = (ch_throttle-988)+(thrust_to_pwm((u1/2)+(u2/2)+(u4/4)) - (ch_pitch-1500));
    m2 = (ch_throttle-988)+(thrust_to_pwm((u1/2)-(u3/2)-(u4/4)) + (ch_roll-1500));
    m3 = (ch_throttle-988)+(thrust_to_pwm((u1/2)-(u2/2)+(u4/4)) + (ch_pitch-1500));
    m4 = (ch_throttle-988)+(thrust_to_pwm((u1/2)+(u3/2)-(u4/4)) - (ch_roll-1500));
    // m1 = (ch_throttle) - (ch_pitch-1500);
    // m2 = (ch_throttle) + (ch_roll-1500);
    // m3 = (ch_throttle) + (ch_pitch-1500);
    // m4 = (ch_throttle) - (ch_roll-1500);
  }
  else
  {
    m1 = 988;
    m2 = 988;
    m3 = 988;
    m4 = 988;
  }
  motor_loop(m1, m2, m3,m4);

  //debug();

  while (micros() - loop_timer < 2000);
  // UART.println(micros() - loop_timer);
  loop_timer = micros();
}

void debug() {
  String str = "";

  // str += " roll:";
  // str += roll;
  // str += " ";
//    str += " roll_rate:";
//   str += roll_rate_sensor;
//   str += " ";

// //  str += " factor:";
// //   str += mpu.getFullScaleGyroRange();
// //   str += " ";
//   //  str += " pitchrate:";
//   // str += pitch_rate_sensor;
//   // str += " ";

// str += " roll_rate_raw :";
//   str += gyroX;
//   str += " ";

  // str += " pitch rate raw:";
  // str += gyroY;
  // str += " ";

  
 
  PC.println(str);
}