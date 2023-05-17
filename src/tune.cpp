//   #include <Arduino.h>
// #include <Servo.h>
// #include <../src/Configuration.h>

// int incomingByte = 0; // for incoming serial data
//   void tune()
//   {
//   if (PC.available() > 0) {
//     // read the incoming byte:
//     incomingByte = Serial.read();
//    };
//    if(incomingByte == 'a')
//    {
//     pwm1 +=10;
//    };
//    if(incomingByte == 's')
//    {
//     pwm2 +=10;
//    };
//    if(incomingByte == 'd')
//    {
//     pwm3 +=10;
//    };
//    if(incomingByte == 'f')
//    {
//     pwm4 +=10;
//    };
//    if(incomingByte == 'A')
//    {
//     pwm1 -=10;
//    };
//    if(incomingByte == 'S')
//    {
//     pwm2 -=10;
//    };
//    if(incomingByte == 'D')
//    {
//     pwm3 -=10;
//    };
//    if(incomingByte == 'F')
//    {
//     pwm4 -=10;
//    };
//   debug();
//   delay(100);
  
//   servo1.writeMicroseconds(pwm1);
//     servo2.writeMicroseconds(pwm2);
//     servo3.writeMicroseconds(pwm3);
//     servo4.writeMicroseconds(pwm4);
//     incomingByte=0;

//   };