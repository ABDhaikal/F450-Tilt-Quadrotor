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



void setup() {

  #ifdef TELEM_DEBUG
  TELEM.begin(TELEM_BAUDRATE);
   delay(250);
  
  #endif 
  
  // #ifdef USB_DEBUG
  // USB.begin(USB_BAUDRATE);
  // delay(250);
  // #endif

  #ifdef GPS_
  GPS.begin(GPS_BAUDRATE);
  delay(250);
  #endif

  radio_setup(&radio_data);
  motor_setup();
 
  imu_setup();
  digitalWrite(LED_BUILTIN, LOW);
   loop_timer = micros();
   debug_timer = micros();


}

void loop() {
  imu_loop(&attitude);
  position_x += kalman_roll*UPDATE_RATE_S;
  radio_loop(&radio_data);
  get_setpoint(&attitude_ref, &radio_data);

  control(&attitude, &attitude_ref, &radio_data);
  motor_loop(m1, m2, m3,m4);




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
  
  
  while ((micros()-loop_timer)<UPDATE_RATE);
  // USB.println((micros()-loop_timer));
  loop_timer = micros();
}



void debug() {
  // DEBUG2(" roll ", attitude.roll);
  // DEBUG2(" pitch ", attitude.pitch);
  // DEBUG2(" yaw ", attitude.yaw);
  // DEBUG2(" roll_rate ", attitude.roll_rate);
  // DEBUG2(" pitch_rate ", attitude.pitch_rate);
  // DEBUG2(" yaw_rate ", attitude.yaw_rate);
  // DEBUG2(" ch_roll ", radio_data.ch_roll);
  // DEBUG2(" ch_pitch ", radio_data.ch_pitch);
  // DEBUG2(" ch_yaw ", radio_data.ch_yaw);
  // DEBUG2(" ch_throttle ", radio_data.ch_throttle);
  // DEBUG2(" armed ", radio_data.armed);

  // DEBUG2(" dmp= ",((float)aa.x)/SCALEs_FACTOR_ACCEL*GRAVITY);
  // DEBUG2(" dmp= ",((float)ax/SCALE_FACTOR_ACCEL)*GRAVITY);

  // DEBUG2(" linx= ",raw_accel_x);
  // DEBUG2(" liny= ",raw_accel_y);
  // DEBUG2(" linz= ",raw_accel_z);
 

  // DEBUG2(" accx= ",(int)kalman_out[0]);
  // DEBUG2(" accy= ",(int)kalman_out[1]);
  // DEBUG2(" accz= ",(int)kalman_out[2]);

  // DEBUG2(" ax= ",ax);
  // DEBUG2(" ay= ",ay);
  // DEBUG2(" az= ",az);


  // DEBUGTAB(mag_x);
  // DEBUGTAB(mag_y);
  // DEBUGTAB(mag_z);

  // DEBUG2(" roll= ",mag_x);
  // DEBUG2(" pitch= ",mag_y);
  // DEBUG2(" yaw= ",mag_z);
  // DEBUG2(" roll= ", gyro_roll);
  // DEBUG2(" pitch= ",gyro_pitch);
  // DEBUG2(" yaw= ",  gyro_yaw);
  //   DEBUG2(" roll= ", acc_roll);
  // DEBUG2(" pitch= ",acc_pitch);
  // // DEBUG2(" yaw2= ",  mag_heading);
  

  DEBUG2(" linacc= ",lin_accel.x);
    // DEBUG2(" vacc= ",raw_accel_y);
    // DEBUG2(" pos= ",position_x);


  // DEBUG2(" ax= ",accel_x);
  // DEBUG2(" ay= ",accel_y);
  // DEBUG2(" az= ",accel_z);
  // DEBUG2(" magacc= ",sqrt(accel_x*accel_x+accel_y*accel_y+accel_z*accel_z)/GRAVITY*100);
  // DEBUG2(" velx= ",lin_accel.x);
  // DEBUG2(" vely= ",lin_accel.y);
  // DEBUG2(" velx= ",lin_accel.z);

  // DEBUG2(" velx= ",world_accel.x);
  // DEBUG2(" vely= ",world_accel.y);
  // DEBUG2(" velx= ",world_accel.z);


  // DEBUG2(" roll= ", roll);
  // DEBUG2(" pitch= ",pitch);
  // DEBUG2(" roll= ", kalman_roll);
  // DEBUG2(" pitch= ",kalman_pitch);

  
  // DEBUG2(" velx= ",lin_accel_x);
  // DEBUG2(" vely= ",lin_accel_y);
  // DEBUG2(" velx2= ",accel_x*cos(pitch*RAD2DEG)+accel_y*sin(pitch*RAD2DEG)*sin(roll*RAD2DEG)+accel_z*sin(pitch*RAD2DEG)*cos(roll*RAD2DEG));
  // linearax = accelx*cos(pitch)+accely*sin(pitch)*sin(roll)+accelz*sin(pitch)*cos(roll);
  // linearay = accely*cos(roll)-accelz*sin(roll);
  // linearaz = -accelx*sin(pitch)+accely*sin(roll)*cos(pitch)+accelz*cos(pitch)*cos(roll);

  // DEBUG2(" velz= ",lin_accel_z);

  // DEBUG2(" velx= ",position_x);
  // DEBUG2(" vely= ",position_y);
  // DEBUG2(" velz= ",position_z);

  // DEBUG2(" liny= ",sqrt(lin_accel_x*lin_accel_x+lin_accel_y*lin_accel_y+lin_accel_z*lin_accel_z	)/GRAVITY);

  // DEBUG2(" liny= ",raw_gyro_x);
  // DEBUG2(" linz= ",raw_gyro_y);
  // DEBUG2(" linz= ",raw_gyro_z);

  

  // DEBUG2(" roll= ", madgwick_roll);
  // DEBUG2(" pitch= ",madgwick_pitch);
  // DEBUG2(" yaw= ",  madgwick_yaw);
  // DEBUG2(" magx= ",mag_x);
  // DEBUG2(" magy= ",mag_y);
  // DEBUG2(" magz= ",mag_z);

  // USBPRINT2(" ",(micros()-loop_timer));

  DEBUGLN();
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
  //     attitude_ref->roll_off  +=0.1;
  //   };
  //   if(incomingByte == 'H')
  //   {
  //     attitude_ref->roll_off  -=0.1;
  //   };

  //    if(incomingByte == 'i')
  //   {
  //     attitude_ref->pitch_off  +=0.1;
  //   };
  //   if(incomingByte == 'I')
  //   {
  //     attitude_ref->pitch_off  -=0.1;
  //   };
  //   incomingByte=0;
  // }
}