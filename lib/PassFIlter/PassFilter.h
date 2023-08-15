#pragma once

class Lowpass
{
    private:
    float sample;
    float cutoff_freq;
    float sample_freq;
    float alpha;
    float output;
    float last_output;
    
    public :
    Lowpass(float _cutoff_freq,float _sample_freq)
    {
        init(_cutoff_freq,_sample_freq);
    }
    void init(float _cutoff_freq,float _sample_freq);
    void compute_alpha(float _cutoff_freq,float _sample_freq);
    float update(float _sample);
    float update(float _sample,float _dt);
    void reset(float _sample){last_output = _sample;};
    float getcutoff(){return cutoff_freq;};
    void setalpha(float _alpha){alpha = _alpha;};
    float getalpha(){return alpha;};
    float getoutput(){return output;};
    float getlastoutput(){return last_output;};
    float getsamplefreq(){return sample_freq;};
};

class Lowpass2p
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
    Lowpass2p(float _cutoff_freq,float _sample_freq)
    {
        init(_cutoff_freq,_sample_freq);
    }
    void compute_alpha(float _cutoff_freq,float _sample_freq);
    void init(float _cutoff_freq,float _sample_freq);
    float update(float _sample);
    float update(float _sample,float _dt);
    float getcutoff_freq(){return cutoff_freq;};

};

class Highpass
{
    private:
    float sample;
    float cutoff_freq;
    float sample_freq;
    float last_sample=0;
    double alpha;
    float output;
    float last_output=0;
    
    public :
    Highpass(float _cutoff_freq,float _sample_freq)
    {
        init(_cutoff_freq,_sample_freq);
    }
    void init(float _cutoff_freq,float _sample_freq);
    void compute_alpha(float _cutoff_freq,float _sample_freq);
    float update(float _sample);
    float update(float _sample,float _dt);
    float getalpha()
    {
        return alpha;
    }

};

