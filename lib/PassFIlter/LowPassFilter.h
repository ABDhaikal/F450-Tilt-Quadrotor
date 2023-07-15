#ifndef PASS_FILTER
#define PASS_FIlTER

class Lowpass
{
    private:
    float sample;
    float cutoff_freq;
    float sample_freq;
    float a1;
    float a2;
    float b0;
    float b1;
    float b2;
    float delay_element_1;
    float delay_element_2;
    float output;
    
    public :
    void init(float _cutoff_freq,float _sample_freq);
    float update(float _sample);

}
#endif