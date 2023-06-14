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
void tune();
void get_setpoint();
int incomingByte = 0; // for incoming serial data

float roll_last = 0;
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
   imu_loop();
   roll_last = roll;
}

void loop() {
  imu_loop();
  
  roll_sensor = roll;
  yaw_sensor = yaw;
  pitch_sensor = pitch;
  // roll_rate_sensor = gyroX;
  // yaw_rate_sensor = gyroZ;
  // pitch_rate_sensor = gyroY;


  roll_rate_sensor = roll_rate;
  yaw_rate_sensor = yaw_rate;
  pitch_rate_sensor = pitch_rate;
  remote_loop();
  get_setpoint();
  control();
  if (arming)
  {
    m1 = (ch_throttle) + thrust_to_pwm((u3/2)/armleght) - (ch_pitch-1500)/3 + thrust_to_pwm((u4/2)) + (ch_yaw-1500);
    m2 = (ch_throttle) + thrust_to_pwm((u2/2)/armleght) + (ch_roll-1500) /3 - thrust_to_pwm((u4/2)) - (ch_yaw-1500);
    m3 = (ch_throttle) - thrust_to_pwm((u3/2)/armleght) + (ch_pitch-1500)/3 + thrust_to_pwm((u4/2)) + (ch_yaw-1500);
    m4 = (ch_throttle) - thrust_to_pwm((u2/2)/armleght) - (ch_roll-1500) /3 - thrust_to_pwm((u4/2)) - (ch_yaw-1500);

  }
  else
  {
    m1 = 988;
    m2 = 988;
    m3 = 988;
    m4 = 988;
  }
  
  motor_loop(m1, m2, m3,m4);
  debug();
  tune();
  // PC.println((micros()-loop_timer));
  //  TELEM.println(micros()-loop_timer);
  while ((micros()-loop_timer)<update_rate);
  // PC.println((micros()-loop_timer));
  loop_timer = micros();
}

void get_setpoint()
{
  if (arming)
  {
    if(ch_roll>1600||ch_roll<1400)
    {
      roll_ref = roll_sensor;
    }
    else
    {
      roll_ref = 0;
    }

    if(ch_pitch>1600||ch_pitch<1400)
    {
      pitch_ref = pitch_sensor;
    }
    else
    {
      pitch_ref = 0;
    }

    if(ch_yaw>1600||ch_yaw<1400)
    {
      yaw_ref = yaw_sensor;
    }
  }
}

void debug() {
  String str = "";
  // str += " K roll:";
  // str += k_roll;
  // str += " ";
  // str += "  Kroll rate:";
  // str += k_roll_rate;
  // str += " ";

  // str += " K yaw:";
  // str += k_yaw;
  // str += " ";
  // str += "  Kyaw rate:";
  // str += k_yaw_rate;
  // str += " ";

  // #ifdef integrator
  // str += " Ki_roll:";
  // str += k_roll_integrator;
  // str += " ";
  // str += " Ki_pith:";
  // str += k_pitch_integrator;
  // str += " ";
  // #endif

  // str += " roll:";
  // str += roll;
  // str += " ";
  // str += " pitch:";
  // str += pitch;
  // str += " ";
  //   str += " yaw:";
  // str += yaw;
  // str += " ";
  // str += " roll rate:";
  // str += roll_rate_sensor;
  // str += " ";
  // str += " pitch rate:";
  // str += pitch_rate_sensor;
  // str += " ";

  str += " roll_dmp:";
  str += roll;
  str += " ";
  // if(roll_rate!=-1)
  // {
  // roll_last += roll_rate*0.002;
  // }
  str += " roll_raw:";
  str += x_hat;
  str += " ";
  str += " gyroAngleX:";
  str += (float)(gyro32[1]);
  str += " ";

  // #ifdef Telem
  //   TELEM.println(str);
  // #endif
  #ifdef PC_Debug
    PC.println(str);
  #endif
}

void tune()
{
  if (TELEM.available() > 0) 
  {
    
    // read the incoming byte:
    incomingByte = TELEM.read();
   
   if(incomingByte == 'a')
   {
    k_roll +=0.1;
    k_pitch +=0.1;
   };
   if(incomingByte == 's')
   {
    k_roll_rate +=0.1;
    k_pitch_rate +=0.1;
   };
   if(incomingByte == 'd')
   {
    k_yaw +=0.1;
   };
   if(incomingByte == 'f')
   {
    k_yaw_rate +=0.1;
   };
   if(incomingByte == 'A')
   {
    k_roll -=0.1;
    k_pitch -=0.1;
   };
   if(incomingByte == 'S')
   {
     k_roll_rate -=0.1;
    k_pitch_rate -=0.1;
   };
   if(incomingByte == 'D')
   {
    k_yaw -=0.1;
   };
   if(incomingByte == 'F')
   {
    k_yaw_rate -=0.1;
   };

   #ifdef integrator
    if(incomingByte == 'g')
    {
      k_roll_integrator  +=0.1;
      k_pitch_integrator +=0.1;
    };
    if(incomingByte == 'G')
    {
      k_roll_integrator  -=0.1;
      k_pitch_integrator -=0.1;
    };
   #endif

    incomingByte=0;
  }
}