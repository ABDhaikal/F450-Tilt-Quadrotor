#include "PassFilter.h"
#include "math.h"


// initialize the filter with cutoff frequency and sample frequency
// to calulate alpha 
//α = (2π * f_c) / (2π * f_c + f_s)

void Lowpass::compute_alpha(float _cutoff_freq,float _sample_freq)
{
    cutoff_freq = _cutoff_freq;
    sample_freq = _sample_freq;
    if ((cutoff_freq)==0) {
        alpha = 1.0;
    }
    if ((sample_freq)==0) {
        alpha = 0.0;
    }
    alpha = (2*M_PI*cutoff_freq)/(2*M_PI*cutoff_freq+sample_freq);
}

void Lowpass::init(float _cutoff_freq,float _sample_freq)
{
    compute_alpha(_cutoff_freq,_sample_freq);
}

//y(t) = α * x(t) + (1 - α) * y(t-1)
float Lowpass::update(float _sample)
{ 
    sample = _sample;
    output = alpha*sample+(1-alpha)*last_output;
    last_output = output;
    return output;
}

// update for uncertain sample rate
float Lowpass::update(float _sample , float _dt)
{   
    sample_freq = 1/_dt;
    compute_alpha(cutoff_freq,sample_freq);
    sample = _sample;
    update(sample);
    return output;
}



void Lowpass2p::compute_alpha(float _cutoff_freq,float _sample_freq)
{
    if(_cutoff_freq <0.1f) 
    {
        cutoff_freq = 1;
    }
    else 
    {
    cutoff_freq = _cutoff_freq;
    };
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

void Lowpass2p::init(float _cutoff_freq,float _sample_freq)
{
    compute_alpha(_cutoff_freq,_sample_freq);
}

float Lowpass2p::update(float _sample)
{   sample = _sample;
    float delay_element_0 = _sample - delay_element_1 * a1 - delay_element_2 * a2;
    output = delay_element_0 * b0 + delay_element_1 * b1 + delay_element_2 * b2;
    delay_element_2 = delay_element_1;
    delay_element_1 = delay_element_0;
    return output;
}

// update for uncertain sample rate
float Lowpass2p::update(float _sample , float _dt)
{   
    sample_freq = 1/_dt;
    compute_alpha(cutoff_freq,sample_freq);
    sample = _sample;
    update(sample);
    return output;
}


void Highpass::compute_alpha(float _cutoff_freq,float _sample_freq)
{
    cutoff_freq = _cutoff_freq;
    sample_freq = _sample_freq;
    if ((cutoff_freq)==0) {
        alpha = 1.0;
    }
    if ((sample_freq)==0) {
        alpha = 0.0;
    }
    alpha = (2*M_PI*cutoff_freq)/(2*M_PI*cutoff_freq+sample_freq);
}

void Highpass::init(float _cutoff_freq,float _sample_freq)
{
    compute_alpha(_cutoff_freq,_sample_freq);
}

// y(t) = α * (y(t-1) + x(t) - x(t-1))
float Highpass::update(float _sample)
{ 
    sample = _sample;
    output = alpha*sample+(1-alpha)*(last_sample-last_output);
    output = sample-output;
    last_sample = sample;
    last_output = output;
    return output;
}

// update for uncertain sample rate
float Highpass::update(float _sample , float _dt)
{   
    sample_freq = 1/_dt;
    compute_alpha(cutoff_freq,sample_freq);
    sample = _sample;
    update(sample);
    return output;
}