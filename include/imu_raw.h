#include <Arduino.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include "Configuration.h"
#include "parameter.h"
#include "EKF.h"

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

#ifdef LPF_ACCEL
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
void LPF_Update();
void LPF_init();
void apply_LPF();
#endif


#ifdef KALMAN_IMU
float roll_kalman;
float pitch_kalman;
float yaw_kalman;
float kalman_roll_uncertainty=4;
float kalman_pitch_uncertainty=4;
float kalman_yaw_uncertainty;
float GyroUncertainty=4; //can get value from standard deviation of raw data 
float AccelUncertainty=3;//can get value from standard deviation of raw data 
void KalmanUpdate();
void applyKalman(float &KalmanState, float &KalmanUncertainty, float KalmanInput, float KalmanMeasurement);
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


// for offset data
bool apply_offset_imu = true;
int offset_gyro_roll =0;
int offset_gyro_pitch =0;
int offset_gyro_yaw =0;
float offset_accel_x =-265;
float offset_accel_y =-51;
float offset_accel_z =40;
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
float roll_rate = 0;
float pitch_rate = 0;
float yaw_rate = 0;
float accel_x=0;
float accel_y=0;
float accel_z=0;
float lin_accel_x=0;
float lin_accel_y=0;
float lin_accel_z=0;
float velocity_x=0;
float velocity_y=0;
float velocity_z=0;
float position_x=0;
float position_y=0;
float position_z=0;
float mag_x;
float mag_y;
float mag_z;
Quaternion q_imu;
Quaternion q_accel;
float ekf_ax;
float ekf_ay;
float ekf_az;


#ifdef USE_BYPASS_IMU
#include "Compass.h"
Compass mag;
float mag_heading;
void init_mag();
void calucalte_mag_heading();
#endif


int16_t gx,gy,gz,ax,ay,az;
MPU6050 mpu;    
void get_imu_offset();
void imu_loop();
void imu_loop(attitude_parameter *data);
void YawOffsetFuntion();
void GetRawData();
void ScaleIMUData();
void LinearAcceleration();
void getoffsetaccel();
void velocityCalculation();



void imu_setup() 
{
    
    Wire.begin();
    Wire.setClock(400000); // 400 kHz I2C clock.
    // initialize device
    mpu.reset();
    delay(30);
    mpu.setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!
    mpu.setMemoryBank(0x10, true, true);
	mpu.setMemoryStartAddress(0x06);
    mpu.setMemoryBank(0, false, false);
    mpu.setRate(0);
    mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8); 
    mpu.setDLPFMode(MPU6050_DLPF_BW_5);
    mpu.setFIFOEnabled(true);
    mpu.resetFIFO();
    mpu.setI2CMasterModeEnabled(false);
    mpu.setI2CBypassEnabled(true);

    #ifdef USE_BYPASS_IMU
    init_mag();
    #endif
	pinMode(DMP_INTERRUPT_PIN, INPUT);

	mpu.setXGyroOffset(-1);
	mpu.setYGyroOffset(194);
	mpu.setZGyroOffset(-20);
	mpu.setXAccelOffset(-1199);
	mpu.setYAccelOffset(-1001);
	mpu.setZAccelOffset(1863);
    mpu.resetFIFO();
    delay(100);
    #ifdef LPF_IMU
    LPF_init();
    #endif

    #ifdef calibrate_imu
    get_imu_offset();
    #endif

    #ifdef MADGWICK_IMU
    _madgwick.begin(1/UPDATE_RATE_S);
    #endif

    for(int i=0;i<10000;i++)
    {
    imu_loop();
    }
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
}
void imu_loop() {
    GetRawData();
    #ifdef USE_BYPASS_IMU
    calucalte_mag_heading();
    #endif

    #ifdef LPF_IMU
    LPF_Update();
    #endif

    gyro_roll += raw_gyro_x*0.002/SCALE_FACTOR_GYRO;
    gyro_pitch += raw_gyro_y*0.002/SCALE_FACTOR_GYRO;
    gyro_yaw += raw_gyro_z*0.002/SCALE_FACTOR_GYRO;
    

    ScaleIMUData();
    #ifdef KALMAN_IMU
    KalmanUpdate();
    #endif
    #ifdef MADGWICK_IMU
    MadgwickUpdate();
    #endif
    LinearAcceleration();
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
    // raw_accel_z = -0.000562*ax+0.001827*ay+0.993785*az;
    raw_accel_x = 0.99380226*ax;
    raw_accel_y = 1.001018961*ay;
    raw_accel_z = 0.996990516*az;
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
    roll_rate   = raw_gyro_x  ;
    pitch_rate  = raw_gyro_y  ;
    yaw_rate    = raw_gyro_z ;
    accel_x = raw_accel_x ;
    accel_y = raw_accel_y ;
    accel_z = raw_accel_z ;

    #ifdef USE_BYPASS_IMU
    mag_x = mag.getMagX();
    mag_y = mag.getMagY();
    mag_z = mag.getMagZ();
    #endif
}

void ScaleIMUData()
{
    roll_rate   = roll_rate/SCALE_FACTOR_GYRO;
    pitch_rate  = pitch_rate/SCALE_FACTOR_GYRO;
    yaw_rate    = yaw_rate/SCALE_FACTOR_GYRO;
    accel_x     = accel_x/SCALE_FACTOR_ACCEL*GRAVITY;
    accel_y     = accel_y/SCALE_FACTOR_ACCEL*GRAVITY;
    accel_z     = accel_z/SCALE_FACTOR_ACCEL*GRAVITY;

};

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
   #ifdef LPF_IMU
   roll_rate = gx;
   pitch_rate = gy;
   yaw_rate = gz;
   LPF_Update();
   #endif
   #ifdef LPF_GYRO
   offset_gyro_roll   += LPF_gyro_x;
   offset_gyro_pitch  += LPF_gyro_y;
   offset_gyro_yaw    += LPF_gyro_z;
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
 	yaw = -yaw +yaw_offset;
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


void LinearAcceleration()
{
   q_accel.w = 0;
    q_accel.x = accel_x;
    q_accel.y = accel_y;
    q_accel.z = accel_z;
    q_accel = q_imu.getProduct(q_accel);
    q_accel = q_accel.getProduct(q_imu.getConjugate());
    lin_accel_x = q_accel.x;
    lin_accel_y = q_accel.y;
    lin_accel_z = q_accel.z-GRAVITY;
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
                    LinearAcceleration();
                    avg_lin_accel_x += lin_accel_x;
                    avg_lin_accel_y += lin_accel_y;
                    avg_lin_accel_z += lin_accel_z;
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
}

void apply_LPF(LPF_param *lpf)
{
    float delay_element_0 = lpf->sample - lpf->delay_element_1 * lpf->a1 - lpf->delay_element_2 * lpf->a2;
    lpf->output = delay_element_0 * lpf->b0 + lpf->delay_element_1 * lpf->b1 + lpf->delay_element_2 * lpf->b2;
    lpf->delay_element_2 = lpf->delay_element_1;
    lpf->delay_element_1 = delay_element_0;
}

void LPF_Update()
{
    #ifdef LPF_ACCEL
    LPF_accel_param[0].sample = accel_x;
    LPF_accel_param[1].sample = accel_y;
    LPF_accel_param[2].sample = accel_z;
    for (int i = 0; i < 3; i++)
    {
         apply_LPF(&LPF_accel_param[i]);
    }
    LPF_accel_x=LPF_accel_param[0].output;
    LPF_accel_y=LPF_accel_param[1].output;
    LPF_accel_z=LPF_accel_param[2].output;
    accel_x = LPF_accel_x;
    accel_y = LPF_accel_y;
    accel_z = LPF_accel_z;
    #endif

    #ifdef LPF_GYRO
    LPF_gyro_param[0].sample = roll_rate;
    LPF_gyro_param[1].sample = pitch_rate;
    LPF_gyro_param[2].sample = yaw_rate;
    for (int i = 0; i < 3; i++)
    {
        apply_LPF(&LPF_gyro_param[i]);
    }
    LPF_gyro_x=LPF_gyro_param[0].output;
    LPF_gyro_y=LPF_gyro_param[1].output;
    LPF_gyro_z=LPF_gyro_param[2].output;
    roll_rate = LPF_gyro_x;
    pitch_rate = LPF_gyro_y;
    yaw_rate = LPF_gyro_z;
    #endif 
}


#endif

#ifdef KALMAN_IMU
void KalmanUpdate()
{
    //calculate accel angle
  acc_roll=atan(accel_y/sqrt(accel_x*accel_x+accel_z*accel_z))*RAD2DEG;
  acc_pitch=-atan(accel_x/sqrt(accel_y*accel_y+accel_z*accel_z))*RAD2DEG;
  acc_yaw=-atan(accel_x/sqrt(accel_y*accel_y+accel_x*accel_x))*RAD2DEG;
  applyKalman(roll,kalman_roll_uncertainty,roll_rate,acc_roll);
  applyKalman(pitch,kalman_pitch_uncertainty,pitch_rate,acc_pitch);
}
void applyKalman(float &KalmanState, float &KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+UPDATE_RATE_S*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + UPDATE_RATE_S * UPDATE_RATE_S * GyroUncertainty * GyroUncertainty;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + AccelUncertainty * AccelUncertainty);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
}
#endif

#ifdef MADGWICK_IMU
void MadgwickUpdate()
{
    _madgwick.update(roll_rate, pitch_rate, yaw_rate, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z);
    
    q_madgwick.w = _madgwick.getW();
    q_madgwick.x = _madgwick.getX();
    q_madgwick.y = _madgwick.getY();
    q_madgwick.z = _madgwick.getZ();
    q_imu = q_madgwick.getNormalized();

    madgwick_roll = _madgwick.getRoll();
    madgwick_pitch = _madgwick.getPitch();
    madgwick_yaw = _madgwick.getYaw();
    roll = madgwick_roll;
    pitch = madgwick_pitch;
    yaw = madgwick_yaw;
    
};
#endif

#ifdef USE_BYPASS_IMU
void init_mag()
{
    mag.begin();
    mag.setRange(HMC5883L_RANGE_2_5GA);
    mag.setMeasurementMode(HMC5883L_CONTINOUS);
    mag.setDataRate(HMC5883L_DATARATE_75HZ);
    mag.setSamples(HMC5883L_SAMPLES_1);
    delay(26);
}
void calucalte_mag_heading()    
{
    mag_heading = atan2(mag_y, mag_x);
    if(mag_heading < 0)
      mag_heading += 2 * M_PI;
    
    mag_heading = mag_heading * 180 / M_PI;
};
#endif

