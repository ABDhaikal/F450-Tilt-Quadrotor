#include <Arduino.h>
#include <sbus.h>
#include <Configuration.h>

bfs::SbusRx sbus_rx(&Serial2);

bfs::SbusData data;

bool signal_lost = false;
int16_t ch_roll, ch_pitch, ch_throttle, ch_yaw;
bool arming;
bool alt_hold_mode = false;
bool lost_state = 0;
int lost_timeout= 50; //in milisecond
int lost_timer = 0;

void failsafe() 
{  
    if(signal_lost)
    { 
        if(lost_state==0)
        {
          lost_state = 1; 
          lost_timer = millis();
        }
        if((millis()-lost_timer)>lost_timeout)
        {
            #ifdef USB_DEBUG
            USB.println("Signal Lost");
            #endif
            #ifdef TELEM_DEBUG
            TELEM.println("Signal Lost");
            #endif
            ch_roll = 1500;
            ch_pitch = 1500;
            ch_throttle = 1000;
            ch_yaw = 1500;
            arming = false;
        }
    }
  else
  {
    lost_timer = 0;
    lost_state = 0;
  }
}

void radio_setup() {
    sbus_rx.Begin();

    if (sbus_rx.Read()) {
        data = sbus_rx.data();

        arming = data.ch[4] > 1500 ? 1 : 0;
        signal_lost = data.lost_frame;

        if (arming) {
            while (arming) {
                if (sbus_rx.Read()) {
                    data = sbus_rx.data();
                    arming = data.ch[4] > 1500 ? 1 : 0;
                    signal_lost = data.lost_frame;
                }
            }
        }

        if (signal_lost) {
            while (signal_lost) {
                if (sbus_rx.Read()) {
                    data = sbus_rx.data();
                    arming = data.ch[4] > 1500 ? 1 : 0;
                    signal_lost = data.lost_frame;
                }
            }
        }
    }
}

void radio_loop() {
  if (sbus_rx.Read()) {
    data = sbus_rx.data();

    ch_roll = data.ch[0] * 0.615f + 890;
    ch_pitch = data.ch[1] * 0.615f + 890;
    ch_throttle = data.ch[2] * 0.615f + 890;
    ch_yaw = data.ch[3] * 0.615f + 890;

    ch_roll = constrain(ch_roll, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
    ch_pitch = constrain(ch_pitch, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
    ch_throttle = constrain(ch_throttle, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
    ch_yaw = constrain(ch_yaw, MOTOR_PWM_MIN, MOTOR_PWM_MAX);

    arming = data.ch[4] > 1500 ? true : false;
    alt_hold_mode = data.ch[5] > 1500 ? true : false;
    signal_lost = data.lost_frame;
  }

  failsafe();
}

