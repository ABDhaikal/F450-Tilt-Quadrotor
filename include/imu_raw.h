#include <Arduino.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include "Configuration.h"
#include "parameter.h"

#ifdef SCALE_FACTOR_ACCEL
#undef SCALE_FACTOR_ACCEL 
#endif
#define SCALE_FACTOR_ACCEL 4096
#ifdef SCALE_FACTOR_GYRO
#undef SCALE_FACTOR_GYRO 
#endif
#define SCALE_FACTOR_GYRO 65.5

#define DMP_INTERRUPT_PIN 22 

#ifdef LPF_IMU
#include <PassFilter.h>

#ifdef LPF_ACCEL
    Lowpass2p LPF_accel_x;
    Lowpass2p LPF_accel_y;
    Lowpass2p LPF_accel_z;
    VectorFloat lpf_accel;
#endif

#ifdef LPF_GYRO
    Lowpass2p LPF_gyro_x;
    Lowpass2p LPF_gyro_y;
    Lowpass2p LPF_gyro_z;
    VectorFloat lpf_gyro;
#endif
void LPF_Update();
void LPF_init();
// void apply_LPF();
#endif


#ifdef KALMAN_IMU
float kalman_roll;
float kalman_pitch;
float yaw_kalman;
float kalman_roll_uncertainty=4;
float kalman_pitch_uncertainty=4;
float kalman_yaw_uncertainty;
float GyroUncertainty=4; //can get value from standard deviation of raw data 
float AccelUncertainty=3;//can get value from standard deviation of raw data 
void KalmanUpdate();
void applyKalman(float &KalmanState, float &KalmanUncertainty, float KalmanInput, float KalmanMeasurement
                ,float inputUncertainty,float MeasurementUncertainty);
#endif

#ifdef MADGWICK_IMU
#include <MadgwickAHRS.h>
Madgwick _madgwick;
float madgwick_roll = 0;
float madgwick_pitch=0;
float madgwick_yaw  =0;
void MadgwickUpdate();
Quaternion q_madgwick;
#endif


float roll_offset  =3.5;
float pitch_offset =1;
float yaw_offset   =0;

// for offset data
bool apply_offset_imu = true;
int offset_gyro_roll =0;
int offset_gyro_pitch =0;
int offset_gyro_yaw =0;
float offset_accel_x =-265  *(4096/SCALE_FACTOR_ACCEL);
float offset_accel_y =-51   *(4096/SCALE_FACTOR_ACCEL);
float offset_accel_z =-50   *(4096/SCALE_FACTOR_ACCEL);
// float offset_accel_x =-263;
// float offset_accel_y =-48;
// float offset_accel_z =25;



// for raw data
float raw_gyro_x;
float raw_gyro_y;
float raw_gyro_z;
float raw_accel_x;
float raw_accel_y;
float raw_accel_z;
float acc_roll;
float acc_pitch;
float acc_yaw;
float gyro_roll;
float gyro_pitch;
float gyro_yaw;
// for final data
float roll = 0;
float pitch = 0;
float yaw = 0;

VectorFloat gyro;
VectorFloat accel;
VectorFloat gravity;
VectorFloat lin_accel;
VectorFloat world_accel;
float velocity_x=0;
float velocity_y=0;
float velocity_z=0;
float position_x=0;
float position_y=0;
float position_z=0;
float mag_x;
float mag_y;
float mag_z;
int16_t raw_mag_x;
int16_t raw_mag_y;
int16_t raw_mag_z;
Quaternion q_imu;
Quaternion q_accel;
float ekf_ax;
float ekf_ay;
float ekf_az;




#ifdef USE_BYPASS_IMU
// #include "Compass.h"
#include <HMC5883L.h>	
HMC5883L mag;
float mag_heading;
void init_mag();
float LSB_TO_mG = 0.92f;
float mag_offset_x=77.200594;
float mag_offset_y=-147.628290;
float mag_offset_z=-35.250081;
float mag_scale[3][3]={
    {0.929955,0.001777,-0.004077},
    {0.001777,0.913090,0.010228},
    {-0.004077,0.010228,1.142049}
};
int compass_last_update=0;
int compass_update_period=15000;
#endif


int16_t gx,gy,gz,ax,ay,az;
MPU6050 mpu;    
void get_imu_offset();
void imu_loop();
void imu_loop(attitude_parameter *data);
void YawOffsetFuntion();
void GetRawData();
void ScaleIMUData();
void CalculateGravity();
void CalculateLinearAcceleration();
void CalculateWorldAcceleration();
void getoffsetaccel();
void velocityCalculation();



void imu_setup() 
{
    
    Wire.begin();
    Wire.setClock(400000); // 400 kHz I2C clock.
    // initialize device
    mpu.reset();
    delay(30);
    mpu.setSleepEnabled(false);
    mpu.setMemoryBank(0x10, true, true);
	// mpu.setMemoryStartAddress(0x06);
    mpu.setMemoryBank(0, false, false);
    mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
    mpu.setRate(0);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8); 
    mpu.setDMPEnabled(false);
    mpu.setDLPFMode(MPU6050_DLPF_BW_20);
    mpu.setDHPFMode(MPU6050_DHPF_HOLD);
    // mpu.setFreefallDetectionThreshold(0);
    // mpu.setYGyroOffsetTC;
    // mpu.resetAccelerometerPath();
    // mpu.setMotionDetectionThreshold(100);
    // mpu.setZeroMotionDetectionThreshold(0);
    mpu.setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!
    mpu.setFIFOEnabled(true);
    mpu.resetFIFO();
    mpu.setI2CMasterModeEnabled(false);
    

    #ifdef USE_BYPASS_IMU
    mpu.setI2CBypassEnabled(true);
    mag.initialize();
    mag.setMode(HMC5883L_MODE_CONTINUOUS);
    mag.setDataRate(HMC5883L_RATE_75);
    mag.setGain(HMC5883L_GAIN_1090);

    
    #endif
	pinMode(DMP_INTERRUPT_PIN, INPUT);

	mpu.setXGyroOffset(-1);
	mpu.setYGyroOffset(194);
	mpu.setZGyroOffset(-20);
	mpu.setXAccelOffset(-1199);
	mpu.setYAccelOffset(-1001);
	mpu.setZAccelOffset(1863);
    // mpu.resetFIFO();
    delay(100);
    #ifdef LPF_IMU
    LPF_init();
    #endif

    #ifdef calibrate_imu
    delay(1000);
    get_imu_offset();
    #endif

    #ifdef MADGWICK_IMU
    _madgwick.begin(500);
    #endif

    // for(int i=0;i<10000;i++)
    // {
    // imu_loop();
    // }
    velocity_x=0;
    velocity_y=0;
    velocity_z=0;
    position_x=0;
    position_y=0;
    position_z=0;
}
void imu_loop(attitude_parameter *data)
{
    imu_loop();

    data->roll = roll-roll_offset;
    data->pitch = pitch-pitch_offset;
    data->yaw = yaw-yaw_offset;
    data->roll_rate = gyro.x;
    data->pitch_rate = gyro.y;
    data->yaw_rate = gyro.z;

}
void imu_loop() {
    GetRawData();

    #ifdef LPF_IMU
    LPF_Update();
    #endif

    // gyro_roll += gyro.x*UPDATE_RATE_S/SCALE_FACTOR_GYRO;
    // gyro_pitch += gyro.y*UPDATE_RATE_S/SCALE_FACTOR_GYRO;
    // gyro_yaw += gyro.z*UPDATE_RATE_S/SCALE_FACTOR_GYRO;
    

    ScaleIMUData();
    #ifdef KALMAN_IMU
    KalmanUpdate();
    #endif
    #ifdef MADGWICK_IMU
    MadgwickUpdate();
    #endif
    CalculateGravity();
    CalculateLinearAcceleration();
    CalculateWorldAcceleration();
    velocityCalculation();
	YawOffsetFuntion();
}

void GetRawData()
{
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if(apply_offset_imu == 1)
    {
    raw_gyro_x  =  ((float)gx-offset_gyro_roll);
	raw_gyro_y  =  ((float)gy-offset_gyro_pitch);
	raw_gyro_z  =  ((float)gz-offset_gyro_yaw);
    ax = (ax- offset_accel_x);
    ay = (ay- offset_accel_y);
    az = (az- offset_accel_z);
    // raw_accel_x = 0.993785*ax+0.005884*ay-0.000562*az;
    // raw_accel_y = 0.005884*ax+1.001000*ay+0.001827*az;
    // raw_accel_z = -0.000562*ax+0.001827*ay+0.996989*az;
    raw_accel_x = (int)(0.993785*ax);
    raw_accel_y = (int)(1.001000*ay);
    raw_accel_z = (int)(0.99609375*az);
    }
    else
    {
    raw_gyro_x  = gx;
	raw_gyro_y  = gy;
	raw_gyro_z  = gz;
    raw_accel_x = ax;
    raw_accel_y = ay;
    raw_accel_z = az;
    }
    // gyro.x   = raw_gyro_x  ;
    // gyro.y  = raw_gyro_y  ;
    // gyro.z    = raw_gyro_z ;
    // accel.x = raw_accel_x ;
    // accel.y = raw_accel_y ;
    // accel.z = raw_accel_z ;

    gyro.x   = raw_gyro_y  ;
    gyro.y  = raw_gyro_x  ;
    gyro.z    = -raw_gyro_z ;
    accel.x = -raw_accel_y ;
    accel.y = -raw_accel_x ;
    accel.z = raw_accel_z ;



    #ifdef USE_BYPASS_IMU
    if((micros()-compass_last_update)>compass_update_period)
    {
    compass_last_update=micros();
    mag.getHeading(&raw_mag_x, &raw_mag_y, &raw_mag_z);
    
    //normalize the raw data

    //apply offset and convert to mili gauss
    raw_mag_x = (float)(raw_mag_x - mag_offset_x) * LSB_TO_mG;
    raw_mag_y = (float)(raw_mag_y - mag_offset_y) * LSB_TO_mG;
    raw_mag_z = (float)(raw_mag_z - mag_offset_z) * LSB_TO_mG;

    // scale mag data from calibrate method
    float temp[3] = {raw_mag_x,raw_mag_y,raw_mag_z};
    float temp2[3] = {0,0,0};
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            temp2[i] += mag_scale[i][j]*temp[j];
        }
    }

    mag_x = temp2[0];
    mag_y = temp2[1];
    mag_z = temp2[2];
    // DEBUG2("mag time",micros()-compass_last_update);
    

    }
    #endif
    
}

void ScaleIMUData()
{
    gyro.x   = gyro.x/SCALE_FACTOR_GYRO;
    gyro.y  = gyro.y/SCALE_FACTOR_GYRO;
    gyro.z    = gyro.z/SCALE_FACTOR_GYRO;
    accel.x     = accel.x/SCALE_FACTOR_ACCEL*GRAVITY;
    accel.y     = accel.y/SCALE_FACTOR_ACCEL*GRAVITY;
    accel.z     = accel.z/SCALE_FACTOR_ACCEL*GRAVITY;

};

void get_imu_offset()
{
 apply_offset_imu = false;
 for(int16_t i=0;i<5000;i++)
  {
     mpu.getRotation(&gx, &gy, &gz);
     
  }
  delay(100);
  for(int16_t i=0;i<10000;i++)
  {
   mpu.getRotation(&gx, &gy, &gz);
   #ifdef LPF_IMU
   gyro.x = gx;
   gyro.y = gy;
   gyro.z = gz;
   LPF_Update();
   #endif
   #ifdef LPF_GYRO
   offset_gyro_roll   += lpf_gyro.x;
   offset_gyro_pitch  += lpf_gyro.y;
   offset_gyro_yaw    += lpf_gyro.z;
   #else
   offset_gyro_roll   += gx;
   offset_gyro_pitch  += gy;
   offset_gyro_yaw    += gz;
   #endif
  }
    offset_gyro_roll   = offset_gyro_roll/10000;
    offset_gyro_pitch  = offset_gyro_pitch/10000;
    offset_gyro_yaw    = offset_gyro_yaw/10000;
    apply_offset_imu = true;
    // getoffsetaccel();
    

    #ifdef USEDEBUG
    DEBUG("offset_accel_x: ");
    DEBUG(offset_accel_x);
    DEBUG(" offset_accel_y: ");
    DEBUG(offset_accel_y);
    DEBUG(" offset_accel_z: ");
    DEBUG(offset_accel_z);
    DEBUG(" offset_gyro_roll: ");
    DEBUG(offset_gyro_roll);
    DEBUG(" offset_gyro_pitch: ");
    DEBUG(offset_gyro_pitch);
    DEBUG(" offset_gyro_yaw: ");
    DEBUGLN(offset_gyro_yaw);
    #endif
}

void YawOffsetFuntion()
{
 	yaw = yaw +yaw_offset;
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

void CalculateGravity()
{
    gravity.x = 2 * (q_imu.x*q_imu.z - q_imu.w*q_imu.y);
    gravity.y = 2 * (q_imu.w*q_imu.x + q_imu.y*q_imu.z);
    gravity.z = q_imu.w*q_imu.w - q_imu.x*q_imu.x - q_imu.y*q_imu.y + q_imu. z*q_imu.z;
    gravity.x *=GRAVITY;
    gravity.y *=GRAVITY;
    gravity.z *=GRAVITY;
    
};

void CalculateLinearAcceleration()
{
    lin_accel.x = accel.x-gravity.x;
    lin_accel.y = accel.y-gravity.y;
    lin_accel.z = accel.z-gravity.z; 
}

void CalculateWorldAcceleration()
{
    world_accel = lin_accel.getRotated(&q_imu);
}

void getoffsetaccel()
{
    float offset_accel_temp_x = -263;
    float offset_accel_temp_y = -48;
    float offset_accel_temp_z = 26;
    float avg_lin_accel_x = 0;
    float avg_lin_accel_y = 0;
    float avg_lin_accel_z = 0;
    float sum_lin_accel   = 0;
    float min_lin_sum[4];
    min_lin_sum[0] = 100000;
    for (int i = 0; i < 4; i++)
    {
        
         for (int j = 0; j < 4; j++)
        {
             for (int k = 0; k < 26; k++)
            {   
                for(int l = 0; l<1000;l++)
                {
                    GetRawData();
                    #ifdef LPF_IMU
                    LPF_Update();
                    #endif
                    ScaleIMUData();
                    #ifdef MADGWICK_IMU
                    MadgwickUpdate();
                    #endif
                }
                for(int l = 0; l<500;l++)
                {
                    offset_accel_x = offset_accel_temp_x-i;
                    offset_accel_y = offset_accel_temp_y-j;
                    offset_accel_z = offset_accel_temp_z-k;
                    GetRawData();
                    #ifdef LPF_IMU
                    LPF_Update();
                    #endif
                    ScaleIMUData();
                    #ifdef MADGWICK_IMU
                    MadgwickUpdate();
                    #endif
                    CalculateLinearAcceleration();
                    avg_lin_accel_x += lin_accel.x;
                    avg_lin_accel_y += lin_accel.x;
                    avg_lin_accel_z += lin_accel.z;
                }
                avg_lin_accel_x = abs(avg_lin_accel_x)/300;
                avg_lin_accel_y = abs(avg_lin_accel_y)/300;
                avg_lin_accel_z = abs(avg_lin_accel_z)/300;
                sum_lin_accel   = avg_lin_accel_x+avg_lin_accel_y+avg_lin_accel_z;
                if(sum_lin_accel<min_lin_sum[0])
                {
                    min_lin_sum[0] = sum_lin_accel;
                    min_lin_sum[1] = offset_accel_x;
                    min_lin_sum[2] = offset_accel_y;
                    min_lin_sum[3] = offset_accel_z;
                    DEBUGLN("GET MIN");
                }
                DEBUG("sum_lin_accel: ");
                DEBUG(sum_lin_accel);
                DEBUG(" offset_accel_x: ");
                DEBUG(offset_accel_x);
                DEBUG(" offset_accel_y: ");
                DEBUG(offset_accel_y);
                DEBUG(" offset_accel_z: ");
                DEBUG(offset_accel_z);
                DEBUG(" avg_lin_accel_x: ");
                DEBUG(avg_lin_accel_x);
                DEBUG(" avg_lin_accel_y: ");
                DEBUG(avg_lin_accel_y);
                DEBUG(" avg_lin_accel_z: ");
                DEBUG(avg_lin_accel_z);
                DEBUG2(" roll: ",roll);
                DEBUG2(" pitch: ",pitch);
                DEBUGLN(" ");
                avg_lin_accel_x = 0;
                avg_lin_accel_y = 0;
                avg_lin_accel_z = 0;
            }
        }
    }


}
void velocityCalculation()
{
    
}

#ifdef LPF_IMU
void LPF_init()
{
    #ifdef LPF_ACCEL
    LPF_accel_x.init(DEFAULT_ACCEL_FILTER_FREQ,DEFAULT_ACCEL_FILTER_SAMPLE_FREQ);
    LPF_accel_y.init(DEFAULT_ACCEL_FILTER_FREQ,DEFAULT_ACCEL_FILTER_SAMPLE_FREQ);
    LPF_accel_z.init(DEFAULT_ACCEL_FILTER_FREQ,DEFAULT_ACCEL_FILTER_SAMPLE_FREQ);
    #endif

    #ifdef LPF_GYRO
    LPF_gyro_x.init(DEFAULT_GYRO_FILTER_FREQ,DEFAULT_GYRO_FILTER_SAMPLE_FREQ);
    LPF_gyro_y.init(DEFAULT_GYRO_FILTER_FREQ,DEFAULT_GYRO_FILTER_SAMPLE_FREQ);
    LPF_gyro_z.init(DEFAULT_GYRO_FILTER_FREQ,DEFAULT_GYRO_FILTER_SAMPLE_FREQ);
    #endif
}


void LPF_Update()
{
    #ifdef LPF_ACCEL

    lpf_accel.x=LPF_accel_x.update(accel.x);
    lpf_accel.y=LPF_accel_y.update(accel.y);
    lpf_accel.z=LPF_accel_z.update(accel.z);
    accel.x = lpf_accel.x;
    accel.y = lpf_accel.y;
    accel.z = lpf_accel.z;
    #endif

    #ifdef LPF_GYRO
    lpf_gyro.x=LPF_gyro_x.update(gyro.x);
    lpf_gyro.y=LPF_gyro_y.update(gyro.y);
    lpf_gyro.z=LPF_gyro_z.update(gyro.z);
    gyro.x = lpf_gyro.x;
    gyro.y = lpf_gyro.y;
    gyro.z = lpf_gyro.z;
    #endif 
}


#endif

#ifdef KALMAN_IMU
void KalmanUpdate()
{
    //calculate accel angle
  acc_roll=atan(accel.y/sqrt(accel.x*accel.x+accel.z*accel.z))*RAD2DEG;
  acc_pitch=-atan(accel.x/sqrt(accel.y*accel.y+accel.z*accel.z))*RAD2DEG;
  acc_yaw=-atan(accel.x/sqrt(accel.y*accel.y+accel.x*accel.x))*RAD2DEG;
  applyKalman(kalman_roll,kalman_roll_uncertainty,lin_accel.x,lin_accel.x*UPDATE_RATE_S,5,1);
  applyKalman(kalman_pitch,kalman_pitch_uncertainty,gyro.x,0,1,0.5);
}
void applyKalman(float &KalmanState, float &KalmanUncertainty, float KalmanInput, float KalmanMeasurement
                ,float inputUncertainty,float MeasurementUncertainty) 
{
  KalmanState=KalmanState+UPDATE_RATE_S*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + UPDATE_RATE_S * UPDATE_RATE_S * inputUncertainty * inputUncertainty;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + MeasurementUncertainty * MeasurementUncertainty);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState); 
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
}
#endif

#ifdef MADGWICK_IMU
void MadgwickUpdate()
{
    // _madgwick.UpdateTiming((int)micros());
    _madgwick.update(gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z, mag_y, mag_x, -mag_z);
    // _madgwick.updateIMU(gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z);

    
    q_madgwick.w = _madgwick.getW();
    q_madgwick.x = _madgwick.getX();
    q_madgwick.y = _madgwick.getY();
    q_madgwick.z = _madgwick.getZ();
    q_imu = q_madgwick;

    madgwick_roll = _madgwick.getRoll();
    madgwick_pitch = _madgwick.getPitch();
    madgwick_yaw = _madgwick.getYaw();
    roll = madgwick_roll;
    pitch = madgwick_pitch;
    yaw = madgwick_yaw;
    
};
#endif



