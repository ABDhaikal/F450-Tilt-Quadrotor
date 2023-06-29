#include <Arduino.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include "Configuration.h"

// inertial parameter
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float roll = 0;
float pitch = 0;
float yaw = 0;
float roll_rate = 0;
float pitch_rate = 0;
float yaw_rate = 0;
float offset_gyro_roll =0;
float offset_gyro_pitch =0;
float offset_gyro_yaw =0;
float offset_accel_x =0;
float offset_accel_y =0;
float offset_accel_z =0;
float yaw_offset = 0;
bool apply_offset_imu = true;

// for raw data
int hehe;
float raw_gyro_x;
float raw_gyro_y;
float raw_gyro_z;
float raw_accel_x;
float raw_accel_y;
float raw_accel_z;
float gyro_roll;
float gyro_pitch;
float gyro_yaw;
int16_t gx,gy,gz,ax,ay,az;
MPU6050 mpu;

void get_imu_offset();
void imu_loop() ;
void YawOffsetFuntion();
void GetRawData();
void madgwick_filter();
#define DMP_INTERRUPT_PIN 22 

#ifdef LPF_ACCEL
 struct LPF_param {
        float sample;
        float cutoff_freq;
        float sample_freq;
        float a1;
        float a2;
        float b0;
        float b1;
        float b2;
        float delay_element_1;
        float delay_element_2;
        float output;
    };
    struct LPF_param LPF_accel_param[3];
 float LPF_accel_x;
 float LPF_accel_y;
 float LPF_accel_z;
#endif

#ifdef LPF_GYRO
 struct LPF_param LPF_gyro_param[3];
 float LPF_gyro_x;
 float LPF_gyro_y;
 float LPF_gyro_z;
#endif


void imu_setup() 
{
    Wire.begin();
    Wire.setClock(400000); // 400 kHz I2C clock.

    // initialize device
    // mpu.reset();
    // delay(30);
    mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); 
    mpu.setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!
	pinMode(DMP_INTERRUPT_PIN, INPUT);
    mpu.setRate(0);
	mpu.setXGyroOffset(6);
	mpu.setYGyroOffset(202);
	mpu.setZGyroOffset(-18);
	mpu.setXAccelOffset(-1149);
	mpu.setYAccelOffset(-1038);
	mpu.setZAccelOffset(1849);
    

    #ifdef LPF_ACCEL
     for (int i = 0; i < 3; i++)
    {
    LPF_accel_param[i].cutoff_freq = DEFAULT_ACCEL_FILTER_FREQ;
    LPF_accel_param[i].sample_freq = DEFAULT_ACCEL_FILTER_SAMPLE_FREQ;
    float fr = LPF_accel_param[i].sample_freq/LPF_accel_param[i].cutoff_freq;
    float ohm = tanf(M_PI/fr);
    float c = 1.0f+2.0f*cosf(M_PI/4.0f)*ohm + ohm*ohm;

    LPF_accel_param[i].b0 = ohm*ohm/c;
    LPF_accel_param[i].b1 = 2.0f*LPF_accel_param[i].b0;
    LPF_accel_param[i].b2 = LPF_accel_param[i].b0;
    LPF_accel_param[i].a1 = 2.0f*(ohm*ohm-1.0f)/c;
    LPF_accel_param[i].a2 = (1.0f-2.0f*cosf(M_PI/4.0f)*ohm+ohm*ohm)/c;
    }
    #endif

    #ifdef LPF_GYRO
     for (int i = 0; i < 3; i++)
    {
    LPF_gyro_param[i].cutoff_freq = DEFAULT_GYRO_FILTER_FREQ;
    LPF_gyro_param[i].sample_freq = DEFAULT_GYRO_FILTER_SAMPLE_FREQ;
    float fr = LPF_gyro_param[i].sample_freq/LPF_gyro_param[i].cutoff_freq;
    float ohm = tanf(M_PI/fr);
    float c = 1.0f+2.0f*cosf(M_PI/4.0f)*ohm + ohm*ohm;

    LPF_gyro_param[i].b0 = ohm*ohm/c;
    LPF_gyro_param[i].b1 = 2.0f*LPF_gyro_param[i].b0;
    LPF_gyro_param[i].b2 = LPF_gyro_param[i].b0;
    LPF_gyro_param[i].a1 = 2.0f*(ohm*ohm-1.0f)/c;
    LPF_gyro_param[i].a2 = (1.0f-2.0f*cosf(M_PI/4.0f)*ohm+ohm*ohm)/c;
    }
    #endif

    #ifdef calibrate_imu
    get_imu_offset();
    #endif
}
void imu_loop() {
    GetRawData();
    roll_rate    = ((float)LPF_gyro_x)/SCALE_FACTOR_GYRO;
    pitch_rate   = ((float)LPF_gyro_y)/SCALE_FACTOR_GYRO;
    yaw_rate     = ((float)LPF_gyro_z)/SCALE_FACTOR_GYRO;
    gyro_roll   += roll_rate*0.001;
    gyro_pitch  += roll_rate*0.001;
    gyro_yaw    += roll_rate*0.001;
   

    roll = gyro_roll;
    pitch = gyro_pitch;
    yaw = gyro_yaw;
	YawOffsetFuntion();
}

void get_imu_offset()
{
 apply_offset_imu = false;
 for(int16_t i=0;i<5000;i++)
  {
     GetRawData();
  }
  delay(100);
  for(int16_t i=0;i<10000;i++)
  {
   GetRawData();
   offset_accel_x     += LPF_accel_x;
   offset_accel_y     += LPF_accel_y;
   offset_accel_z     += LPF_accel_z;
   offset_gyro_roll   += LPF_gyro_x;
   offset_gyro_pitch  += LPF_gyro_y;
   offset_gyro_yaw    += LPF_gyro_z;
  }
    offset_accel_x     = offset_accel_x/10000;
    offset_accel_y     = offset_accel_y/10000;
    offset_accel_z     = offset_accel_z/10000;
    offset_gyro_roll   = offset_gyro_roll/10000;
    offset_gyro_pitch  = offset_gyro_pitch/10000;
    offset_gyro_yaw    = offset_gyro_yaw/10000;
    apply_offset_imu = true;
    #ifdef USB_DEBUG
    USB.print("offset_accel_x: ");
    USB.print(offset_accel_x);
    USB.print(" offset_accel_y: ");
    USB.print(offset_accel_y);
    USB.print(" offset_accel_z: ");
    USB.print(offset_accel_z);
    USB.print(" offset_gyro_roll: ");
    USB.print(offset_gyro_roll);
    USB.print(" offset_gyro_pitch: ");
    USB.print(offset_gyro_pitch);
    USB.print(" offset_gyro_yaw: ");
    USB.println(offset_gyro_yaw);
    #endif
    #ifdef TELEM_DEBUG
    TELEM.print("offset_accel_x: ");
    TELEM.print(offset_accel_x);
    TELEM.print(" offset_accel_y: ");
    TELEM.print(offset_accel_y);
    TELEM.print(" offset_accel_z: ");
    TELEM.print(offset_accel_z);
    TELEM.print(" offset_gyro_roll: ");
    TELEM.print(offset_gyro_roll);
    TELEM.print(" offset_gyro_pitch: ");
    TELEM.print(offset_gyro_pitch);
    TELEM.print(" offset_gyro_yaw: ");
    TELEM.println(offset_gyro_yaw);
    #endif

}

void YawOffsetFuntion()
{
 	yaw = yaw +180+yaw_offset;
	if(yaw<0)
	{
		yaw = yaw + 360;
	}
	if(yaw>360)
	{
		yaw = yaw - 360;
	}
	yaw = yaw - 180;
}

void GetRawData()
{
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if(apply_offset_imu == 1)
    {
    raw_gyro_x  =  ((float)gy-offset_gyro_roll);
	raw_gyro_y  =  ((float)gx-offset_gyro_pitch);
	raw_gyro_z  =  ((float)-gz-offset_gyro_yaw);
    raw_accel_x = ax- offset_accel_x;
    raw_accel_y = ay- offset_accel_y;
    raw_accel_z = az- offset_accel_z;
    }
    else
    {
    raw_gyro_x  = gy;
	raw_gyro_y  = gx;
	raw_gyro_z  = -gz;
    raw_accel_x = ax;
    raw_accel_y = ay;
    raw_accel_z = az;
    }

    #ifdef LPF_ACCEL
    LPF_accel_param[0].sample = raw_accel_x;
    LPF_accel_param[1].sample = raw_accel_y;
    LPF_accel_param[2].sample = raw_accel_z;
    for (int i = 0; i < 3; i++)
    {
        float delay_element_0 = LPF_accel_param[i].sample - LPF_accel_param[i].delay_element_1 * LPF_accel_param[i].a1 - LPF_accel_param[i].delay_element_2 * LPF_accel_param[i].a2;
        LPF_accel_param[i].output = delay_element_0 * LPF_accel_param[i].b0 + LPF_accel_param[i].delay_element_1 * LPF_accel_param[i].b1 + LPF_accel_param[i].delay_element_2 * LPF_accel_param[i].b2;

        LPF_accel_param[i].delay_element_2 = LPF_accel_param[i].delay_element_1;
        LPF_accel_param[i].delay_element_1 = delay_element_0;
    }
    LPF_accel_x=LPF_accel_param[0].output;
    LPF_accel_y=LPF_accel_param[1].output;
    LPF_accel_z=LPF_accel_param[2].output;
    #endif

    #ifdef LPF_GYRO
    LPF_gyro_param[0].sample = raw_gyro_x;
    LPF_gyro_param[1].sample = raw_gyro_y;
    LPF_gyro_param[2].sample = raw_gyro_z;
    for (int i = 0; i < 3; i++)
    {
        float delay_element_0 = LPF_gyro_param[i].sample - LPF_gyro_param[i].delay_element_1 * LPF_gyro_param[i].a1 - LPF_gyro_param[i].delay_element_2 * LPF_gyro_param[i].a2;
        LPF_gyro_param[i].output = delay_element_0 * LPF_gyro_param[i].b0 + LPF_gyro_param[i].delay_element_1 * LPF_gyro_param[i].b1 + LPF_gyro_param[i].delay_element_2 * LPF_gyro_param[i].b2;

        LPF_gyro_param[i].delay_element_2 = LPF_gyro_param[i].delay_element_1;
        LPF_gyro_param[i].delay_element_1 = delay_element_0;
    }
    LPF_gyro_x=((LPF_gyro_param[0].sample*LPF_gyro_param[0].cutoff_freq)+LPF_gyro_param[0].output)/(LPF_gyro_param[0].cutoff_freq+1);
    LPF_gyro_y=((LPF_gyro_param[1].sample*LPF_gyro_param[1].cutoff_freq)+LPF_gyro_param[1].output)/(LPF_gyro_param[1].cutoff_freq+1);
    LPF_gyro_z=((LPF_gyro_param[2].sample*LPF_gyro_param[2].cutoff_freq)+LPF_gyro_param[2].output)/(LPF_gyro_param[2].cutoff_freq+1);
    #endif
}

void madgwick_filter()
{

}