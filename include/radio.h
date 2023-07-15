#include <Arduino.h>
#include <sbus.h>
#include <Configuration.h>
#include "parameter.h"

bfs::SbusRx sbus_rx(&Serial2);

bfs::SbusData data;

bool signal_lost = false;
bool alt_hold_mode = false;
bool lost_state = 0;
int lost_timeout= 50; //in milisecond
int lost_timer = 0;

void failsafe(radio_parameter *radio_data) 
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
            radio_data->ch_roll = 1500;
            radio_data->ch_pitch = 1500;
            radio_data->ch_throttle = 1000;
            radio_data->ch_yaw = 1500;
            radio_data->armed = false;
        }
    }
  else
  {
    lost_timer = 0;
    lost_state = 0;
  }
}

void radio_setup(radio_parameter *radio_data) {
    sbus_rx.Begin();

    if (sbus_rx.Read()) {
        data = sbus_rx.data();

        radio_data->armed = data.ch[4] > 1500 ? 1 : 0;
        signal_lost = data.lost_frame;

        if (radio_data->armed) {
            while (radio_data->armed) {
                if (sbus_rx.Read()) {
                    data = sbus_rx.data();
                    radio_data->armed = data.ch[4] > 1500 ? 1 : 0;
                    signal_lost = data.lost_frame;
                }
            }
        }

        if (signal_lost) {
            while (signal_lost) {
                if (sbus_rx.Read()) {
                    data = sbus_rx.data();
                    radio_data->armed = data.ch[4] > 1500 ? 1 : 0;
                    signal_lost = data.lost_frame;
                }
            }
        }
    }
}

void radio_loop(radio_parameter *radio_data) {
  if (sbus_rx.Read()) {
    data = sbus_rx.data();

    radio_data->ch_roll = data.ch[0] * 0.615f + 890;
    radio_data->ch_pitch = data.ch[1] * 0.615f + 890;
    radio_data->ch_throttle = data.ch[2] * 0.615f + 890;
    radio_data->ch_yaw = data.ch[3] * 0.615f + 890;

    radio_data->ch_roll = constrain(radio_data->ch_roll, RADIO_PWM_MIN, RADIO_PWM_MAX);
    radio_data->ch_pitch = constrain(radio_data->ch_pitch, RADIO_PWM_MIN, RADIO_PWM_MAX);
    radio_data->ch_throttle = constrain(radio_data->ch_throttle, RADIO_PWM_MIN, RADIO_PWM_MAX);
    radio_data->ch_yaw = constrain(radio_data->ch_yaw, RADIO_PWM_MIN, RADIO_PWM_MAX);

    radio_data->armed = data.ch[4] > 1500 ? true : false;
    alt_hold_mode = data.ch[5] > 1500 ? true : false;
    signal_lost = data.lost_frame;
  }

  failsafe(radio_data);
}

