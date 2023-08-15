#include "Arduino.h"
#include "Configuration.h"
#include "radio.h"
#include "motor.h"
#ifdef dmp
#include "imu.h"
#else
#include "imu_raw.h"
#endif
#ifdef USE_BARO
#include "barometer.h"
#endif
#ifdef GPS_
#include "GPS.h"
#endif
// #include "sonar.h"
#include "control.h"

// declare parameter
attitude_parameter attitude;
attitude_parameter attitude_ref;
radio_parameter radio_data;
uint32_t loop_timer = 0;
uint32_t debug_timer = 0;
void debug();
void tune();
int incomingByte = 0;
bool init = false;

float inc;


void setup() {

  #ifdef TELEM_DEBUG
  TELEM.begin(TELEM_BAUDRATE);
   delay(250);
  
  #endif 
  
  #ifdef USB_DEBUG
  USB.begin(USB_BAUDRATE);
  delay(250);
  #endif

  #ifdef GPS_
  GPS.begin(GPS_BAUDRATE);
  delay(250);
  #endif
  #ifdef USE_BARO
  baro_setup();
  #endif

  radio_setup(&radio_data);
  motor_setup();
 
  imu_setup();
  control_setup();
  digitalWrite(LED_BUILTIN, LOW);
   loop_timer = micros();
   debug_timer = micros();


}

void loop() {
  imu_loop(&attitude);
  #ifdef USE_BARO
  baro_loop();
  #endif

  #ifdef GPS_
  gps_loop();
  #endif

  radio_loop(&radio_data);
  get_setpoint(&attitude_ref, &radio_data,&attitude);

  control(&attitude, &attitude_ref, &radio_data);
  motor_loop(m1, m2, m3,m4);
  servo_loop(s1, s2, s3,s4);
  tune();

  




  // tune();
  if((micros()-debug_timer)>DEBUG_CYCLE)
  {
      debug();
      debug_timer = micros();
  }
  if((micros()-loop_timer)>UPDATE_RATE)
  {
    DEBUGLN();
    DEBUG(" WARNING OVERTIME :");
    DEBUG((micros()-loop_timer));
    DEBUGLN();
  }

  // USBPRINT((micros()-loop_timer));
  // DEBUGLN();
  
  
  while ((micros()-loop_timer)<UPDATE_RATE);
  // USB.println((micros()-loop_timer));
  loop_timer = micros();
}



void debug() {
  DEBUG2(" roll ", attitude.roll);
  DEBUG2(" pitch ", attitude.pitch);
  DEBUG2(" yaw ", attitude.yaw);
  DEBUG2(" roll_rate ", attitude.roll_rate);
  DEBUG2(" pitch_rate ", attitude.pitch_rate);
  DEBUG2(" yaw_rate ", attitude.yaw_rate);

  DEBUG2(" roll_ref ", attitude_ref.roll);
  DEBUG2(" pitch_ref ", attitude_ref.pitch);
  DEBUG2(" yaw_ref ", attitude_ref.yaw);

  // DEBUG2(" x_ref ", attitude_ref.x_position);
  // DEBUG2(" y_ref ", attitude_ref.y_position);
  // DEBUG2(" z_ref ", attitude_ref.z_position);
// 

  // DEBUG2(" u1 ",u3);
  // DEBUG2(" u2 ",u4);
  // DEBUG2(" u3 ",u5);
  // DEBUG2(" u4 ",u6);

  // DEBUG2(" ch_roll ", radio_data.ch_roll);
  // DEBUG2(" ch_pitch ", radio_data.ch_pitch);
  // DEBUG2(" ch_yaw ", radio_data.ch_yaw);
  DEBUG2(" ch_throttle ", radio_data.ch_throttle);
  // DEBUG2(" armed ", radio_data.armed);
  // DEBUG2(" lost", data.lost_frame);
  // DEBUG2(" signal_lost", signal_lost);


  // DEBUG2(" u1 ", u1);
  // DEBUG2(" u2 ", u2);
  // DEBUG2(" u3 ", u3);
  // DEBUG2(" u4 ", u4);
  // DEBUG2(" u5 ", u5);
  // DEBUG2(" u6 ", u6);

  DEBUG2(" roll_off ",roll_offset);
  DEBUG2(" pitch_off ",pitch_offset);

// 
  // DEBUG2(" servo1_off ",servo1_off);
  // DEBUG2(" servo2_off ",servo2_off);
  // DEBUG2(" s3off ",servo3_off);
  // DEBUG2(" s4off ",servo4_off);
  DEBUG2(" inc ",inc);

  // DEBUG2(" s1 ",s1);
  // DEBUG2(" s2 ",s2);
  // DEBUG2(" s3 ",s3);
  // DEBUG2(" s4 ",s4);
  // DEBUG2(" u1should angle ",tan(((float)radio_data.ch_pitch-1500)/500*SERVO_ANGLE_MAX*DEG2RAD));

  // DEBUG2(" INC ",inc);




  // DEBUG2(" m1 ", m1);
  // DEBUG2(" m2 ", m2);
  // DEBUG2(" m3 ", m3);
  // DEBUG2(" m4 ", m4);

  // DEBUG2(" o1 ", omega2_to_pwm(omega2_m1));
  // DEBUG2(" o2 ", omega2_to_pwm(omega2_m2));
  // DEBUG2(" o3 ", omega2_to_pwm(omega2_m3));
  // DEBUG2(" o4 ", omega2_to_pwm(omega2_m4));

  // DEBUG2(" ch_roll ", radio_data.ch_roll);
  // DEBUG2(" ch_pitch ", radio_data.ch_pitch);
  // DEBUG2(" ch_yaw ", radio_data.ch_yaw);
  // DEBUG2(" ch_throttle ", radio_data.ch_throttle);
  // DEBUG2(" armed ", radio_data.armed);
  
  // DEBUG2(" accX= ",raw_accel_x);
  // DEBUG2(" accY= ",raw_accel_y);
  // DEBUG2(" accZ= ",raw_accel_z);
  
  // DEBUG2(" acc= ",mpu.getAccelXSelfTestFactoryTrim());
  // DEBUG2(" acc= ",mpu.getAccelYSelfTestFactoryTrim());
  // DEBUG2(" acc= ",mpu.getAccelZSelfTestFactoryTrim());






  // DEBUG2(" ff= ",mpu.setSlaveWordGroupOffset());




  // DEBUG2(" acc= ",world_accel.x);

  
  //  DEBUG2(" mag_heading= ",mag_heading);
  // DEBUG2(" rawmx= ",raw_mag_x);
  // DEBUG2(" rawmy= ",raw_mag_y);
  // DEBUG2(" rawmz= ",raw_mag_z);

  // DEBUG2(" mx= ",mag_x);
  // DEBUG2(" my= ",mag_y);
  // DEBUG2(" mz= ",mag_z);

  // DEBUG2(" pressurem= ",baro.raw_pres);
  // DEBUG2(" temperaturem= ",baro.raw_temp);

  // DEBUG2(" pressurem= ",pressurem);
  // DEBUG2(" temperaturem= ",temperaturem);

  // DEBUG2(" baro_altitude= ",baro_altitude);
  // DEBUG2(" baro_altitude_rate= ",baro_altitude_rate);

  // DEBUG2(" sat= ",gps_sat);
  // DEBUG2(" lat= ",gps_lat);
  // DEBUG2(" lon= ",gps_lon);
  // DEBUG2(" hdop= ",gps_hdop);
  // DEBUG2(" age= ",gps_age);

  


  // DEBUG2(" RK= ",acc_roll);
  // DEBUG2(" PK= ",acc_pitch);
  // DEBUG2(" RK= ",madgwick_roll);
  // DEBUG2(" PK= ",madgwick_pitch);
  // DEBUG2(" Py= ",madgwick_yaw);

 

  DEBUGLN();
}

void tune()
{
  // #ifdef TELEM_DEBUG
  if (TELEM.available() > 0) 
  {
    
  //   // read the incoming byte:
    incomingByte = TELEM.read();
   
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
   
     if(incomingByte == 'a')
    {
      roll_offset  +=inc;
    };
    if(incomingByte == 's')
    {
      pitch_offset +=inc;
    };
// 
     if(incomingByte == 'A')
    {
      roll_offset -=inc;
    };
    if(incomingByte == 'S')
    {
      pitch_offset  -=inc;
    };


  //    if(incomingByte == 'a')
  //  {
  //   servo1_off +=inc;
  //  };
  //  if(incomingByte == 's')
  //  {
  //   servo2_off +=inc;
  //  };
  //  if(incomingByte == 'd')
  //  {
  //   servo3_off +=inc;
  //  };
  //  if(incomingByte == 'f')
  //  {
  //   servo4_off +=inc;
  //  };
  //  if(incomingByte == 'A')
  //  {
  //   servo1_off -=inc;
  //  };
  //  if(incomingByte == 'S')
  //  {
  //   servo2_off -=inc;
  //  };
  //  if(incomingByte == 'D')
  //  {
  //   servo3_off -=inc;
  //  };
  //  if(incomingByte == 'F')
  //  {
  //   servo4_off -=inc;
  //  };
   if(incomingByte == 'g')
   {
    inc +=0.1;
   };
   if(incomingByte == 'G')
   {
    inc -=0.1;
   };

  
  //    if(incomingByte == 'a')
  //  {
  //   roll_offset +=inc;
  //  };
  //  if(incomingByte == 's')
  //  {
  //   pitch_offset +=inc;
  //  };

   
  //    if(incomingByte == 'A')
  //  {
  //   roll_offset -=inc;
  //  };
  //  if(incomingByte == 'S')
  //  {
  //   pitch_offset -=inc;
  //  };

    incomingByte=0;
  }
}