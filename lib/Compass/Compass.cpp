/*
HMC5883L.cpp - Class file for the HMC5883L Triple Axis Digital Compass Arduino Library.

Version: 1.1.0
(c) 2014 Korneliusz Jarzebski
www.jarzebski.pl

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/




#include "Compass.h"

bool Compass::begin()
{
    Wire.begin();

    if ((fastRegister8(HMC5883L_REG_IDENT_A) != 0x48)
    || (fastRegister8(HMC5883L_REG_IDENT_B) != 0x34)
    || (fastRegister8(HMC5883L_REG_IDENT_C) != 0x33))
    {
	return false;
    }

    setRange(HMC5883L_RANGE_1_3GA);
    setMeasurementMode(HMC5883L_CONTINOUS);
    setDataRate(HMC5883L_DATARATE_15HZ);
    setSamples(HMC5883L_SAMPLES_1);

    mgPerDigit = 0.92f;

    return true;
}


Vector Compass::readRaw(void)
{
    v.XAxis = (readRegister16(HMC5883L_REG_OUT_X_M) - xOffset);
    v.YAxis = (readRegister16(HMC5883L_REG_OUT_Y_M) - yOffset);
    v.ZAxis = (readRegister16(HMC5883L_REG_OUT_Z_M) - zOffset);
    //scale data
    float temp[3] = {v.XAxis, v.YAxis, v.ZAxis};
    float temp2[3] = {0,0,0};
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            temp2[i] += Scale[i][j]*temp[j];
        }
    }
    v.XAxis = temp2[0];
    v.YAxis = temp2[1];
    v.ZAxis = temp2[2];
    return v;
}

Vector Compass::readNormalize(void)
{
    v.XAxis = (((float)readRegister16(HMC5883L_REG_OUT_X_M) - xOffset))* mgPerDigit;
    v.YAxis = (((float)readRegister16(HMC5883L_REG_OUT_Y_M) - yOffset))* mgPerDigit;
    v.ZAxis = (((float)readRegister16(HMC5883L_REG_OUT_Z_M) - zOffset))* mgPerDigit;
    //scale data
    float temp[3] = {v.XAxis, v.YAxis, v.ZAxis};
    float temp2[3] = {0,0,0};
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            temp2[i] += Scale[i][j]*temp[j];
        }
    }
    v.XAxis = temp2[0];
    v.YAxis = temp2[1];
    v.ZAxis = temp2[2];


    return v;
}

void Compass::setOffset(float xo, float yo, float zo)
{
    xOffset = xo;
    yOffset = yo;
    zOffset = zo;

}

void Compass::setOffset(float xo, float yo, float zo, float _sc[3][3])
{
    xOffset = xo;
    yOffset = yo;
    zOffset = zo;
     for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            Scale[i][j] = _sc[i][j];
        }
    }
}

void Compass::setScale(float _sc[3][3])
{
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            Scale[i][j] = _sc[i][j];
        }
    }
}


void Compass::setRange(hmc5883l_range_t range)
{
    switch(range)
    {
	case HMC5883L_RANGE_0_88GA:
	    mgPerDigit = 0.073f;
	    break;

	case HMC5883L_RANGE_1_3GA:
	    mgPerDigit = 0.92f;
	    break;

	case HMC5883L_RANGE_1_9GA:
	    mgPerDigit = 1.22f;
	    break;

	case HMC5883L_RANGE_2_5GA:
	    mgPerDigit = 1.52f;
	    break;

	case HMC5883L_RANGE_4GA:
	    mgPerDigit = 2.27f;
	    break;

	case HMC5883L_RANGE_4_7GA:
	    mgPerDigit = 2.56f;
	    break;

	case HMC5883L_RANGE_5_6GA:
	    mgPerDigit = 3.03f;
	    break;

	case HMC5883L_RANGE_8_1GA:
	    mgPerDigit = 4.35f;
	    break;

	default:
	    break;
    }

    writeRegister8(HMC5883L_REG_CONFIG_B, range << 5);
}

hmc5883l_range_t Compass::getRange(void)
{
    return (hmc5883l_range_t)((readRegister8(HMC5883L_REG_CONFIG_B) >> 5));
}

void Compass::setMeasurementMode(hmc5883l_mode_t mode)
{
    uint8_t value;

    value = readRegister8(HMC5883L_REG_MODE);
    value &= 0b11111100;
    value |= mode;

    writeRegister8(HMC5883L_REG_MODE, value);
}

hmc5883l_mode_t Compass::getMeasurementMode(void)
{
    uint8_t value;

    value = readRegister8(HMC5883L_REG_MODE);
    value &= 0b00000011;

    return (hmc5883l_mode_t)value;
}

void Compass::setDataRate(hmc5883l_dataRate_t dataRate)
{
    uint8_t value;

    value = readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b11100011;
    value |= (dataRate << 2);

    writeRegister8(HMC5883L_REG_CONFIG_A, value);
}

hmc5883l_dataRate_t Compass::getDataRate(void)
{
    uint8_t value;

    value = readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b00011100;
    value >>= 2;

    return (hmc5883l_dataRate_t)value;
}

void Compass::setSamples(hmc5883l_samples_t samples)
{
    uint8_t value;

    value = readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b10011111;
    value |= (samples << 5);

    writeRegister8(HMC5883L_REG_CONFIG_A, value);
}

hmc5883l_samples_t Compass::getSamples(void)
{
    uint8_t value;

    value = readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b01100000;
    value >>= 5;

    return (hmc5883l_samples_t)value;
}

Vector Compass::selfTest()
{
	Vector value;
	
	setMeasurementMode(HMC5883L_SINGLE);
	writeRegister8(HMC5883L_REG_CONFIG_A, 0x00);
	writeRegister8(HMC5883L_REG_CONFIG_A, 0x01);
	value = readRaw();
	writeRegister8(HMC5883L_REG_CONFIG_A, 0x00);
	
	return value;
}

// Write byte to register
void Compass::writeRegister8(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(HMC5883L_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(reg);
        Wire.write(value);
    #else
        Wire.send(reg);
        Wire.send(value);
    #endif
    Wire.endTransmission();
}

// Read byte to register
uint8_t Compass::fastRegister8(uint8_t reg)
{
    uint8_t value;
    Wire.beginTransmission(HMC5883L_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif
    Wire.endTransmission();

    Wire.requestFrom(HMC5883L_ADDRESS, 1);
    #if ARDUINO >= 100
        value = Wire.read();
    #else
        value = Wire.receive();
    #endif;
    Wire.endTransmission();

    return value;
}

// Read byte from register
uint8_t Compass::readRegister8(uint8_t reg)
{
    uint8_t value;
    Wire.beginTransmission(HMC5883L_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif
    Wire.endTransmission();

    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.requestFrom(HMC5883L_ADDRESS, 1);
    while(!Wire.available()) {};
    #if ARDUINO >= 100
        value = Wire.read();
    #else
        value = Wire.receive();
    #endif;
    Wire.endTransmission();

    return value;
}

// Read word from register
int16_t Compass::readRegister16(uint8_t reg)
{
    int16_t value;
    Wire.beginTransmission(HMC5883L_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif
    Wire.endTransmission();

    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.requestFrom(HMC5883L_ADDRESS, 2);
    while(!Wire.available()) {};
    #if ARDUINO >= 100
        uint8_t vha = Wire.read();
        uint8_t vla = Wire.read();
    #else
        uint8_t vha = Wire.receive();
        uint8_t vla = Wire.receive();
    #endif;
    Wire.endTransmission();

    value = vha << 8 | vla;

    return value;
}

// void Compass::calibrate(int time_calibrate)
// {
//     int timing_period = 1000000 / getDataRate();
//     int start_time = micros();
//     int timing_calib = micros();
//     float total_scale=0;
//     float maxX=0, minX=0, maxY=0, minY=0, maxZ=0, minZ=0;
//     float offX=0, offY=0, offZ=0;

//     while(micros() - start_time < time_calibrate * 1000000)
//     {  while(micros() - timing_calib < timing_period)
//         {
//         v.XAxis = (float)readRegister16(HMC5883L_REG_OUT_X_M);
//         v.YAxis = (float)readRegister16(HMC5883L_REG_OUT_Y_M);
//         v.ZAxis = (float)readRegister16(HMC5883L_REG_OUT_Z_M);

//         if (v.XAxis < minX) minX = v.XAxis;
//         if (v.XAxis > maxX) maxX = v.XAxis;
//         if (v.YAxis < minY) minY = v.YAxis;
//         if (v.YAxis > maxY) maxY = v.YAxis;
//         if (v.ZAxis < minZ) minZ = v.ZAxis;
//         if (v.ZAxis > maxZ) maxZ = v.ZAxis; 

//         // Calculate offsets
//         offX = (maxX + minX)/2;
//         offY = (maxY + minY)/2;
//         offZ = (maxZ + minZ)/2;

//         Scale[0][0] = (maxX - minX)/2;
//         Scale[1][1] = (maxY - minY)/2;
//         Scale[2][2] = (maxZ - minZ)/2;

//         total_scale = Scale[0][0] + Scale[1][1] + Scale[2][2];
//         Scale[0][0] = total_scale / Scale[0][0];
//         Scale[1][1] = total_scale / Scale[1][1];
//         Scale[2][2] = total_scale / Scale[2][2];

//         #ifdef USEDEBUG
//         // Serial.print("X: "); Serial.print(v.XAxis); Serial.print(" ");
//         // Serial.print("Y: "); Serial.print(v.YAxis); Serial.print(" ");
//         // Serial.print("Z: "); Serial.print(v.ZAxis); Serial.print(" ");
//         // Serial.print("Xoff: "); Serial.print(offX); Serial.print(" ");
//         // Serial.print("Yoff: "); Serial.print(offY); Serial.print(" ");
//         // Serial.print("Zoff: "); Serial.print(offZ); Serial.print(" ");
//         // Serial.print("Xscale: "); Serial.print(xScale); Serial.print(" ");
//         // Serial.print("Yscale: "); Serial.print(yScale); Serial.print(" ");
//         // Serial.print("Zscale: "); Serial.print(zScale); Serial.print(" ");
//         // Serial.println();

//         // DEBUG2("x", v.XAxis);
//         // DEBUG2("y", v.YAxis);
//         // DEBUG2("z", v.ZAxis);
//         // DEBUG2("xoff", offX);
//         // DEBUG2("yoff", offY);
//         // DEBUG2("zoff", offZ);
//         // DEBUG2("xscale", xScale);
//         // DEBUG2("yscale", yScale);
//         // DEBUG2("zscale", zScale);
//         // DEBUGLN();
//         #endif
//         timing_calib = micros();
//         }
//         // Serial.println("Calibration done");
//         // DEBUGLN("Calibration done");
//         this->setOffset(offX, offY, offZ);
//     }
    

// }
