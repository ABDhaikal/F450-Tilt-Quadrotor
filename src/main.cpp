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
  imu_setup();
  //sonar_setup();

  digitalWrite(LED_BUILTIN, LOW);
  loop_timer = micros();
}

void loop() {
  imu_loop();
  remote_loop();
  control();

  if (arming)
  {
    m1 = (int)thrust_to_pwm(u1); // Calculate the pulse for esc 1 (front-right - CCW).
    m2 = (int)thrust_to_pwm(u2); // Calculate the pulse for esc 1 (front-right - CCW).
    m3 = (int)thrust_to_pwm(u3); // Calculate the pulse for esc 1 (front-right - CCW).
    m4 = (int)thrust_to_pwm(u4); // Calculate the pulse for esc 1 (front-right - CCW).

  }
  else
  {
    m1 = 1000;
    m2 = 1000;
    m3 = 1000;
    m4 = 1000;
  }

  motor_loop(m1, m2, m3,m4);

  while (micros() - loop_timer < 2000)
    ;
  // UART.println(micros() - loop_timer);
  loop_timer = micros();
}

// void debug() {
//   String str = "";
//   str += " pwm1:";
//   str += pwm1;
//   str += " ";
//   str += " pwm2:";
//   str += pwm2;
//   str += " ";
//   str += " pwm3:";
//   str += pwm3;
//   str += " ";
//   str += " pwm4:";'
//   str += pwm4;
//   str += " ";
//   // str += " Roll2:";
//   // str += roll2;
//   // str += " ";
//   // str += " Pitch2:";
//   // str += pitch2;
//   // str += " ";
//   // str += " Yaw2:";
//   // str += yaw2;
//   // str += " ";
//   PC.println(str);
// }