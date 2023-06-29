#include "Arduino.h"
#include "Configuration.h"
#include "radio.h"
#include "motor.h"
#ifdef dmp
#include "imu.h"
#else
#include "imu_raw.h"
#endif
#include "sonar.h"
#include "control.h"


void debug();

void tune();
int incomingByte = 0;

void get_setpoint();

void setup() {
  
  #ifdef TELEM_DEBUG
  TELEM.begin(TELEM_BAUDRATE);
  #endif 

  #ifdef USB_DEBUG
  USB.begin(USB_BAUDRATE);
  #endif

  #ifdef GPS_
  GPS.begin(GPS_BAUDRATE);
  delay(250);
  #endif

  radio_setup();
  motor_setup();
 
  imu_setup();
  digitalWrite(LED_BUILTIN, LOW);
   loop_timer = micros();
   debug_timer = micros();
}

void loop() {
  imu_loop();
  
  roll_sensor = roll;
  yaw_sensor = yaw;
  pitch_sensor = pitch;
  roll_rate_sensor = roll_rate;
  yaw_rate_sensor = yaw_rate;
  pitch_rate_sensor = pitch_rate;

  radio_loop();
  get_setpoint();

  control(ch_roll,ch_pitch,ch_yaw,ch_throttle,arming);
  motor_loop(m1, m2, m3,m4);

  // tune();
  if(micros()-debug_timer>10000)
  {
    USB.print("debug");
      debug();
      debug_timer = micros();
  }


  // USB.print(" ");
  USB.println((micros()-loop_timer));
  while ((micros()-loop_timer)<UPDATE_RATE);
  // USB.println((micros()-loop_timer));
  loop_timer = micros();
}

void get_setpoint()
{
    if(ch_roll>1500||ch_roll<1500)
    {
      roll_ref =  (abs((float)ch_roll-1500)/500*LIMIT_ROLL)*(ch_roll-1500)/abs((float)ch_roll-1500)+ roll_ref_off;
    }
    else
    {
      roll_ref =roll_ref_off;
    }

    if(ch_pitch>1500||ch_pitch<1500)
    {
      pitch_ref =  -(abs((float)ch_pitch-1500)/500*LIMIT_PITCH)*(ch_pitch-1500)/abs((float)ch_pitch-1500)+pitch_ref_off ;
    }
    else
    {
      pitch_ref = pitch_ref_off;
    }
  
}

void debug() {

}

void tune()
{
  // #ifdef TELEM_DEBUG
  // if (TELEM.available() > 0) 
  // {
    
  //   // read the incoming byte:
  //   incomingByte = TELEM.read();
   
  //  if(incomingByte == 'a')
  //  {
  //   k_roll +=0.1;
  //   k_pitch +=0.1;
  //  };
  //  if(incomingByte == 's')
  //  {
  //   k_roll_rate +=0.1;
  //   k_pitch_rate +=0.1;
  //  };
  //  if(incomingByte == 'd')
  //  {
  //   k_yaw +=0.1;
  //  };
  //  if(incomingByte == 'f')
  //  {
  //   k_yaw_rate +=0.1;
  //  };
  //  if(incomingByte == 'A')
  //  {
  //   k_roll -=0.1;
  //   k_pitch -=0.1;
  //  };
  //  if(incomingByte == 'S')
  //  {
  //    k_roll_rate -=0.1;
  //   k_pitch_rate -=0.1;
  //  };
  //  if(incomingByte == 'D')
  //  {
  //   k_yaw -=0.1;
  //  };
  //  if(incomingByte == 'F')
  //  {
  //   k_yaw_rate -=0.1;
  //  };

  //  #ifdef CONTROL_INTEGRATOR
  //   if(incomingByte == 'g')
  //   {
  //     k_roll_integrator  +=0.1;
  //     k_pitch_integrator +=0.1;
  //   };
  //   if(incomingByte == 'G')
  //   {
  //     k_roll_integrator  -=0.1;
  //     k_pitch_integrator -=0.1;
  //   };
  //   if(incomingByte == 'j')
  //   {
  //     k_yaw_integrator  +=0.1;
  //   };
  //   if(incomingByte == 'J')
  //   {
  //     k_yaw_integrator  -=0.1;
  //   };
  //  #endif
   
  //    if(incomingByte == 'h')
  //   {
  //     roll_ref_off  +=0.1;
  //   };
  //   if(incomingByte == 'H')
  //   {
  //     roll_ref_off  -=0.1;
  //   };

  //    if(incomingByte == 'i')
  //   {
  //     pitch_ref_off  +=0.1;
  //   };
  //   if(incomingByte == 'I')
  //   {
  //     pitch_ref_off  -=0.1;
  //   };
  //   incomingByte=0;
  // }
}