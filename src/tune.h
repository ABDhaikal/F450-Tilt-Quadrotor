//   #include <Arduino.h>
// #include <Servo.h>
// #include <../src/Configuration.h>
// #include <motor.h>

//   void tune()
//   {
//     int incomingByte = 0; // for incoming serial data
//   if (TELEM.available() > 0) {
    
//     // read the incoming byte:
//     incomingByte = TELEM.read();
//    };
//    if(incomingByte == 'a')
//    {
//     pwms1 +=10;
//    };
//    if(incomingByte == 's')
//    {
//     pwms2 +=10;
//    };
//    if(incomingByte == 'd')
//    {
//     pwms3 +=10;
//    };
//    if(incomingByte == 'f')
//    {
//     pwms4 +=10;
//    };
//    if(incomingByte == 'A')
//    {
//     pwms1 -=10;
//    };
//    if(incomingByte == 'S')
//    {
//     pwms2 -=10;
//    };
//    if(incomingByte == 'D')
//    {
//     pwms3 -=10;
//    };
//    if(incomingByte == 'F')
//    {
//     pwms4 -=10;
//    };
  
//   servo1.writeMicroseconds(pwms1);
//     servo2.writeMicroseconds(pwms2);
//     servo3.writeMicroseconds(pwms3);
//     servo4.writeMicroseconds(pwms4);
//     incomingByte=0;

//   };