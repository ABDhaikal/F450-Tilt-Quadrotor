#include <Arduino.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include "Configuration.h"

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
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float roll = 0;
float pitch = 0;
float yaw = 0;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void quaternion_to_rotation_matrix();

void imu_setup() {
    Wire.begin();
    Wire.setClock(400000); // 400 kHz I2C clock.

	mpu.initialize();
	pinMode(INTERRUPT_PIN, INPUT);

	devStatus = mpu.dmpInitialize();

	mpu.setXGyroOffset(187);
    mpu.setYGyroOffset(9);
    mpu.setZGyroOffset(19);
	mpu.setXAccelOffset(-862);
	mpu.setYAccelOffset(-495);
    mpu.setZAccelOffset(1076);

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
}

void imu_loop() {
	if (!dmpReady) return;

	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	fifoCount = mpu.getFIFOCount();

	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		mpu.resetFIFO();
	} else if (mpuIntStatus & 0x02) {
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		mpu.getFIFOBytes(fifoBuffer, packetSize);
		fifoCount -= packetSize;

		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

		// get roll pitch yaw
		roll = ypr[0] * RAD_TO_DEG;
		pitch = ypr[1] * RAD_TO_DEG;
		yaw = ypr[2] * RAD_TO_DEG;
		if (yaw < 0) yaw += 360; // yaw stays between 0 and 360

		mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
		mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
		mpu.dmpGetGyro(&gyro,fifoBuffer);
		// TODO: pose estimation
	}
}