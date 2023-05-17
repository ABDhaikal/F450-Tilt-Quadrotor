#include <Arduino.h>
#include <sbus.h>

bfs::SbusRx sbus_rx(&Serial2);

bfs::SbusData data;

bool signal_lost = false;
int16_t ch_roll, ch_pitch, ch_throttle, ch_yaw;
bool arming;
bool alt_hold_mode = false;

void failsafe() {
    if (signal_lost) {
        ch_roll = 1500;
        ch_pitch = 1500;
        ch_throttle = 1000;
        ch_yaw = 1500;
        arming = false;
    }
}

void remote_setup() {
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

void remote_loop() {
  if (sbus_rx.Read()) {
    data = sbus_rx.data();

    ch_roll = data.ch[0] * 0.615f + 890;
    ch_pitch = data.ch[1] * 0.615f + 890;
    ch_throttle = data.ch[2] * 0.615f + 890;
    ch_yaw = data.ch[3] * 0.615f + 890;

    ch_roll = constrain(ch_roll, 1000, 2000);
    ch_pitch = constrain(ch_pitch, 1000, 2000);
    ch_throttle = constrain(ch_throttle, 1000, 2000);
    ch_yaw = constrain(ch_yaw, 1000, 2000);

    arming = data.ch[4] > 1500 ? true : false;
    alt_hold_mode = data.ch[5] > 1500 ? true : false;
    signal_lost = data.lost_frame;
  }

  failsafe();
}

