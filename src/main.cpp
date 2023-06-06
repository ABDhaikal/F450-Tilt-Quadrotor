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

float x_diff;
float y_diff;
float z_diff;

float x_last;
float y_last;
float z_last;


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
  // loop_timer = micros();




  // imu_loop();
  // x_last = roll;
  // y_last = pitch;
  // z_last = yaw;
  // loop_timer = micros();
}

void loop() {
  imu_loop();
  
  roll_sensor = roll;
  yaw_sensor = yaw;
  pitch_sensor = pitch;
  roll_rate_sensor = roll_rate;
  yaw_rate_sensor = yaw_rate;
  pitch_rate_sensor = pitch_rate;
  remote_loop();
  control();
  if (arming)
  {
    m1 = (ch_throttle) + thrust_to_pwm((u3/2)/armleght) - (ch_pitch-1500)+ (ch_yaw-1500);
    m2 = (ch_throttle) - thrust_to_pwm((u2/2)/armleght) + (ch_roll-1500)- (ch_yaw-1500);
    m3 = (ch_throttle) - thrust_to_pwm((u3/2)/armleght) + (ch_pitch-1500)+ (ch_yaw-1500);
    m4 = (ch_throttle) + thrust_to_pwm((u2/2)/armleght) - (ch_roll-1500)-  (ch_yaw-1500);
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

}

void debug() {
  int incomingByte = 0;
  // if (TELEM.available() > 0) {
  //    // for incoming serial data
  //   // read the incoming byte:
  //   incomingByte = TELEM.read();
  //  };
  //  if(incomingByte == 'a')
  //  {
  //   k_roll += 0.1;
  //  };
  //  if(incomingByte == 's')
  //  {
  //   k_roll_rate += 0.1;
  //  };
  //  if(incomingByte == 'd')
  //  {
  //   k_pitch += 0.1;
  //  };
  //  if(incomingByte == 'f')
  //  {
  //   k_pitch_rate += 0.1;
  //  };
  //  if(incomingByte == 'A')
  //  {
  //   k_roll -= 0.1;
  //  };
  //  if(incomingByte == 'S')
  //  {
  //   k_roll_rate -= 0.1;
  //  };
  //  if(incomingByte == 'D')
  //  {
  //   k_pitch -= 0.1;
  //  };
  //  if(incomingByte == 'F')
  //  {
  //   k_pitch_rate -= 0.1;
  //  };
  
  String str = "";

  // str += " P_roll:";
  // str += k_roll;
  // str += " ";
   
  // str += " D_roll:";
  // str += k_roll_rate;
  // str += " ";

  // str += " P_roll:";
  // str += k_pitch;
  // str += " ";
   
  // str += " D_roll:";
  // str += k_pitch_rate;
  // str += " ";
  str += " cht:";
  str += ch_throttle;
  str += " ";
  str += " chr:";
  str += ch_roll;
  str += " ";
  str += " chp:";
  str += ch_pitch;
  str += " ";
  str += " chy:";
  str += ch_yaw;
  str += " ";

  PC.println(str);
}