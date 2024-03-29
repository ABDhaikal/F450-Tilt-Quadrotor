#include <Arduino.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include "Configuration.h"
#include "parameter.h"

// inertial parameter
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// for raw data
float raw_gyro_x;
float raw_gyro_y;
float raw_gyro_z;
float raw_accel_x;
float raw_accel_y;
float raw_accel_z;
int16_t gx,gy,gz,ax,ay,az;

#ifdef dmp
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
VectorInt16 gyro;    // [x, y, z]            gyro

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() 
{
    mpuInterrupt = true;
}


#ifdef dmp_update_boost
int loop_validator = 6;
int loop_validator_count = 0;
bool dmp_on_update = false;
float last_roll = 0;
float last_pitch = 0;
float last_yaw = 0;
float diff_roll = 0;
float diff_pitch =0;
float diff_yaw = 0;

float last_roll_rate = 0;
float last_pitch_rate = 0;
float last_yaw_rate = 0;
float diff_roll_rate = 0;
float diff_pitch_rate =0;
float diff_yaw_rate = 0;
float last_diff_roll_rate = 0;
float last_diff_pitch_rate =0;
float last_diff_yaw_rate = 0;


#endif

#endif




// kalman filter
#ifdef kalman_imu
	float kalman_K [3];
	float kalman_A [3] = {1,1,1};
	float kalman_C [3] = {0.8,1,1};
	float kalman_P [3];
	float kalman_Q [3] ={0.01,1,0.1};
	float kalman_R [3] ={2,1,1};
	float kalman_out[3];
	float kalman_in[3];
#endif



void imu_loop() ;

void imu_setup() {
    Wire.begin();
    Wire.setClock(400000); // 400 kHz I2C clock.

	#ifdef dmp
	mpu.initialize();
	pinMode(DMP_INTERRUPT_PIN, INPUT);

	devStatus = mpu.dmpInitialize();
	mpu.setXGyroOffset(4);
	mpu.setYGyroOffset(202);
	mpu.setZGyroOffset(-19);
	mpu.setXAccelOffset(-1147);
	mpu.setYAccelOffset(-1038);
	mpu.setZAccelOffset(1831);

	
	if (devStatus == 0) {
		#ifdef calibrate_imu
		mpu.CalibrateAccel(100);
    	mpu.CalibrateGyro(100);
    	mpu.PrintActiveOffsets();
		#endif
		mpu.setDMPEnabled(true);

		attachInterrupt(digitalPinToInterrupt(DMP_INTERRUPT_PIN), dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		dmpReady = true;

		packetSize = mpu.dmpGetFIFOPacketSize();
	} else {
		#ifdef USB_DEBUG
		Serial.print("DMP Initialization failed ");
		Serial.print(devStatus);
		Serial.println("");
		#endif
	}

	

	#endif
}


void imu_loop(attitude_parameter *_attitude) {
	#ifdef dmp
		if (!dmpReady)return;
		if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
		{
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			// get roll roll yaw
			_attitude->yaw = ypr[0] * RAD_TO_DEG;
			_attitude->roll = -ypr[1] * RAD_TO_DEG ;
			_attitude->pitch = ypr[2] * RAD_TO_DEG ;
			mpu.dmpGetAccel(&aa, fifoBuffer);
    		mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
			mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q); 
			mpu.dmpGetGyro(&gyro,fifoBuffer);
			_attitude->roll_rate 	=  gyro.y;
  			_attitude->yaw_rate 	= -gyro.z;
  			_attitude->pitch_rate 	=  gyro.x;
			if(_attitude->roll_rate==-1)
			{
				_attitude->roll_rate=0;
			}
			if(_attitude->yaw_rate==1)
			{
				_attitude->yaw_rate=0;
			}
			if(_attitude->pitch_rate==-1)
			{
				_attitude->pitch_rate=0;
			}
			#ifdef dmp_update_boost
			dmp_on_update = true;
			loop_validator_count =0;
			diff_roll = _attitude->roll-last_roll;
			diff_pitch	= _attitude->pitch-last_pitch;
			diff_yaw = _attitude->yaw-last_yaw;
			last_roll = _attitude->roll;
			last_pitch = _attitude->pitch;
			last_yaw = _attitude->yaw;
			
			diff_roll_rate = _attitude->roll_rate-last_roll_rate;
			diff_pitch_rate	= _attitude->pitch_rate-last_pitch_rate;
			diff_yaw_rate = _attitude->yaw_rate-last_yaw_rate;
			last_roll_rate = _attitude->roll_rate;
			last_pitch_rate = _attitude->pitch_rate;
			last_yaw_rate = _attitude->yaw_rate;
			#endif
		}
		#ifdef dmp_update_boost
		if(loop_validator_count!=0)
		{
			dmp_on_update = false;
			_attitude->roll_rate += diff_roll_rate/loop_validator;
			_attitude->pitch_rate += diff_pitch_rate/loop_validator;
			_attitude->yaw_rate +=   diff_yaw_rate/loop_validator;
			_attitude->roll += diff_roll/loop_validator;
			_attitude->pitch += diff_pitch/loop_validator;
			_attitude->yaw += _attitude->yaw + diff_yaw/loop_validator;
		}
		loop_validator_count++;
		#endif
	#endif

	#ifdef kalman_imu
		//add kalman filter
		kalman_in[0] = _attitude->roll_rate;
		kalman_in[1] = _attitude->pitch_rate;
		kalman_in[2] = _attitude->yaw_rate;
		for(int i = 0; i<3; i++)
		{
			double x_pred = kalman_A[i] * kalman_out[i];
        	double P_pred = kalman_A[i] * kalman_P[i] * kalman_A[i] + kalman_Q[i];

        	// Calculate the Kalman gain
        	kalman_K[i] = P_pred * kalman_C[i] / (kalman_C[i] * P_pred * kalman_C[i] + kalman_R[i]);

        	// Update the state estimate
        	kalman_out[i] = x_pred + kalman_K[i] * (kalman_in[i] - kalman_C[i] * x_pred);

        	// Update the error covariance
        	kalman_P[i] = (1 - kalman_K[i] * kalman_C[i]) * P_pred;
		}
	#endif
	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    raw_gyro_x  =  ((float)gy)/SCALE_FACTOR_GYRO;
	raw_gyro_y  =  ((float)gx)/SCALE_FACTOR_GYRO;
	raw_gyro_z  =  ((float)-gz)/SCALE_FACTOR_GYRO;
    raw_accel_x = ax;
    raw_accel_y = ay;
    raw_accel_z = az;

	_attitude->yaw = _attitude->yaw +180+yaw_offset;
	if(_attitude->yaw<0)
	{
		_attitude->yaw = _attitude->yaw + 360;
	}
	if(_attitude->yaw>360)
	{
		_attitude->yaw = _attitude->yaw - 360;
	}
	_attitude->yaw = _attitude->yaw - 180;
}
