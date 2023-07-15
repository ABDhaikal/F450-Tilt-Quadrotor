#include "LowPassFilter.h"
#include "math.h"

void Lowpass :: init(float _cutoff_freq,float _sample_freq)
{
    cutoff_freq = _cutoff_freq;
    sample_freq = _sample_freq;
    float fr = sample_freq/cutoff_freq;
    float ohm = tanf(M_PI/fr);
    float c = 1.0f+2.0f*cosf(M_PI/4.0f)*ohm + ohm*ohm;

    b0 = ohm*ohm/c;
    b1 = 2.0f*b0;
    b2 = b0;
    a1 = 2.0f*(ohm*ohm-1.0f)/c;
    a2 = (1.0f-2.0f*cosf(M_PI/4.0f)*ohm+ohm*ohm)/c;
}

float Lowpass :: update(float _sample)
{   sample = _sample;
    float delay_element_0 = _sample - delay_element_1 * a1 - delay_element_2 * a2;
    output = delay_element_0 * b0 + delay_element_1 * b1 + delay_element_2 * b2;
    delay_element_2 = delay_element_1;
    delay_element_1 = delay_element_0;
    return output;
}