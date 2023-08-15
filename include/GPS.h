#include <TinyGPSPlus.h>
#include <configuration.h>


TinyGPSPlus gps;
uint32_t gps_time = 0;
double gps_lat = 0;
double gps_lon = 0;
uint32_t gps_alt = 0;
uint32_t gps_sat = 0;
double gps_speed = 0;
double gps_hdop =100;
double gps_course;
uint32_t gps_age = 0;


void gps_loop()
{
    while (GPS.available() > 0)
    if (gps.encode(GPS.read()))
    {
      if (gps.location.isValid())
        {
        gps_lat = gps.location.lat()*1000000;
        gps_lon = gps.location.lng()*1000000;
        gps_age = gps.location.age();
        }
        if (gps.altitude.isValid())
        {
        gps_alt = gps.altitude.meters();
        }
        if (gps.satellites.isValid())
        {
        gps_sat = gps.satellites.value();
        }
        if (gps.hdop.isValid())
        {
        gps_hdop = gps.hdop.hdop();
        }
        if (gps.speed.isValid())
        {
        gps_speed = gps.speed.mps();
        }
        if (gps.course.isValid())
        {
        gps_course = gps.course.deg();
        }
    }
    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
      Serial.println(F("No GPS detected: check wiring."));
      while(true);
    }
    

}