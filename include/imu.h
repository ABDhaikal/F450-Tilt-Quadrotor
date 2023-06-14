#include <Arduino.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include "Configuration.h"


MPU6050 raw;
float accelX, accelY, accelZ, gyroX, gyroY=0, gyroZ;
float accelAngleX, accelAngleY, accelAngleZ;
float gyroAngleX=0,gyroAngleY=0,gyroAngleZ=0;
int16_t ax, ay, az, gx, gy, gz;
float roll_comple = 0;
float pitch_comple = 0;
float yaw_comple = 0;
float roll_rate_comple = 0;
float pitch_rate_comple = 0;
float yaw_rate_comple = 0;
float scaleFactorGyro = 16.4;

#ifdef dmp
MPU6050 mpu;


#define INTERRUPT_PIN 22  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

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
int32_t gyro32[3];

#endif

float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float roll = 0;
float pitch = 0;
float yaw = 0;
uint32_t roll_rate = 0;
float pitch_rate = 0;
float yaw_rate = 0;
float offset_gyro_roll =0;

// kalman filter
float kalman_K = 0;
float kalman_A = 1;
float kalman_C = 1;
float kalman_P = 0;
float kalman_Q = 0.0001;
float kalman_R = 0.2;
float x_hat = 0;

// validator
int loop_validator = 4;
int loop_validator_count = 0;






volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void imu_loop() ;

void imu_setup() {
    Wire.begin();
    Wire.setClock(400000); // 400 kHz I2C clock.
	raw.initialize();
	raw.setXGyroOffset(6);
	raw.setYGyroOffset(198);
	raw.setZGyroOffset(-12);
	raw.setXAccelOffset(-1178);
	raw.setYAccelOffset(-1000);
	raw.setZAccelOffset(1863);
	raw.setRate(2);

	#ifdef dmp
	mpu.initialize();
	pinMode(INTERRUPT_PIN, INPUT);

	devStatus = mpu.dmpInitialize();
	mpu.setXGyroOffset(9);
	mpu.setYGyroOffset(201);
	mpu.setZGyroOffset(-6);
	mpu.setXAccelOffset(-1134);
	mpu.setYAccelOffset(-1044);
	mpu.setZAccelOffset(1866);

	
	if (devStatus == 0) {
		mpu.setDMPEnabled(true);

		attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		dmpReady = true;

		packetSize = mpu.dmpGetFIFOPacketSize();
	} else {
		#ifdef PC_Debug
		Serial.print("DMP Initialization failed ");
		Serial.print(devStatus);
		Serial.println("");
		#endif
	}
	#endif
	int i = 0;
	while(i<1000) {
		raw.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		i++;
		offset_gyro_roll += (float)gy/scaleFactorGyro;
		delay(2);
	}
	offset_gyro_roll = offset_gyro_roll/1000;
	imu_loop();
	delay(5);
	gyroAngleX = roll;
}

void imu_loop() {
	#ifdef dmp
	loop_validator_count++;
	if (!dmpReady) return;


	 if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
	 {
		loop_validator_count =0;
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

		// get roll pitch yaw
		yaw = ypr[0] * RAD_TO_DEG;
		roll = -ypr[1] * RAD_TO_DEG;
		pitch = ypr[2] * RAD_TO_DEG;

		// mpu.dmpGetAccel(&aa, fifoBuffer);
        // mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
		// mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q); 
		
		mpu.dmpGetGyro(gyro32,fifoBuffer);

		roll_rate 	= gyro32[1];
  		yaw_rate 	= -gyro32[2];
  		pitch_rate 	=  gyro32[0];

	 }
	 	#endif

		raw.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		roll_rate_comple = (gy /scaleFactorGyro)+offset_gyro_roll;
		pitch_rate_comple = gx / scaleFactorGyro;
		yaw_rate_comple = -gz / scaleFactorGyro;

		//add kalman filter
		double x_pred = kalman_A * x_hat;
        double P_pred = kalman_A * kalman_P * kalman_A + kalman_Q;

        // Calculate the Kalman gain
        kalman_K = P_pred * kalman_C / (kalman_C * P_pred * kalman_C + kalman_R);

        // Update the state estimate
        x_hat = x_pred + kalman_K * (roll_rate_comple - kalman_C * x_pred);

        // Update the error covariance
        kalman_P = (1 - kalman_K * kalman_C) * P_pred;


		if(x_hat>0.05||x_hat<-0.05)
		{
			gyroAngleX = gyroAngleX + (x_hat  * update_rate/1000000);
		}
		
 		gyroAngleY = gyroAngleY + (pitch_rate_comple * update_rate/1000000);
		gyroAngleZ = gyroAngleZ + (yaw_rate_comple   * update_rate/1000000);
		accelAngleY = (atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * 180 / PI); // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  		accelAngleX = (atan(-1 * ax / sqrt(pow(ay, 2) + pow(az, 2))) * 180 / PI); // AccErrorY ~(-1.5\
		
		 // Complementary filter - combine acceleromter and gyro angle values
 		roll_comple = 0.96 * gyroAngleX + 0.04 * accelAngleX;
 		pitch_comple = 0.96 * gyroAngleY + 0.04 * accelAngleY;
		yaw_comple =  yaw_comple + gyroAngleZ ;
		// gyroAngleX = roll_comple;
		// gyroAngleX = 0.1*gyroAngleX+0.9*roll;
		// if(loop_validator_count>loop_validator){
		// 	loop_validator_count = 0;
		// 	gyroAngleX = 0.8*roll+0.2*gyroAngleX;
		// }

}