#pragma once

#define FILTER_TYPE_LOWPASS  0
#define FILTER_TYPE_NOTCH    1
#define FILTER_TYPE_PEEK     2
#define FILTER_TYPE_HIGHPASS 3

typedef struct biquad_axis_state
{
    volatile float a0, a1, a2, a3, a4;
    volatile float x1, x2, y1, y2;
} biquad_axis_state_t;


extern void  biquad_imuf_init(float filterCutFreq, biquad_axis_state_t *newState, float refreshRateSeconds, int filterType, float bandwidth);
extern float biquad_imuf_update(float sample, biquad_axis_state_t *state);
