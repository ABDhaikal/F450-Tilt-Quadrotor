#include "Wire.h"
#include "I2Cdev.h"
#include "MS5611.h"
#include "Configuration.h"
#include "passfilter.h"
MS5611 baro(0x77);
#define DegreeToKevin 273.15
float reference_pressure = 0;
float refrence_temperature = 0; 
float pressurem;
float temperaturem;
float raw_baro_altitude;
float baro_altitude;
float baro_altitude_offset = 0;

float last_baro_altitude;
float raw_baro_altitude_rate;
float baro_altitude_rate;

int32_t baro_timer = 0;
int32_t baro_start_timer = 0;
Lowpass baroFilter(0.2, 0.0);
Lowpass baroRateFilter(0.2, 0.0);
bool baro_calibrated = false;

void baro_loop();
void baro_setup() {
    Wire.begin();
    baro.initialize();
    DEBUG(baro.testConnection() ? "baro connection successful" : "baro connection failed");
    // this gives false positive value on the due I think due to the problems with wire ?
    baro.setOverSampleRate(MS561101BA_OSR_256);

    float temp_counter = 0;
    for(int i=0;i<100;i++)
    {   
        float temp_press;
        float temp_temp;
        if(baro.readValues(&temp_press, &temp_temp))
        {
            pressurem += temp_press;
            temperaturem += temp_temp;
            temp_counter++;
        }
        delay(10);
    }
    refrence_temperature = (temperaturem/temp_counter)+DegreeToKevin;
    reference_pressure = pressurem/temp_counter;
    baro_timer = micros();
    baro_start_timer = millis();
}

void baro_loop() {
    if(baro.readValues(&pressurem, &temperaturem))
    {
        raw_baro_altitude = 153.8462f * refrence_temperature * (1.0f - expf(0.190259f * logf(pressurem/reference_pressure)));
        baro_altitude = baroFilter.update(raw_baro_altitude,(micros()-baro_timer)*0.000001f)-baro_altitude_offset;
        raw_baro_altitude_rate= (baro_altitude - last_baro_altitude)/(micros()-baro_timer)*1000000;
        last_baro_altitude = (baro_altitude);
        baro_altitude_rate = baroRateFilter.update(raw_baro_altitude_rate,(micros()-baro_timer)*0.000001f);
        // Serial.println((micros()-baro_timer));
        baro_timer = micros();
    }
    if((millis()-baro_start_timer>5000)&&(!baro_calibrated))
    {
        baro_calibrated = true;
        baro_altitude_offset = baro_altitude;
    }
    
}