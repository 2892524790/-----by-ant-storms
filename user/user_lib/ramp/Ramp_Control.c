#include "Ramp_Control.h"
#include "arm_math.h"

float RampCalc(RampGen_t* ramp)
{
    ramp->count++;
    if(ramp->count > ramp->XSCALE)
        ramp->count = ramp->XSCALE;
    ramp->out = (1.0f * (ramp->count) / ramp->XSCALE);
    return ramp->out;
}
void RampSetScale(struct RampGen_t* ramp, int32_t scale)
{
    ramp->XSCALE = scale;
}
void RampResetCounter(struct RampGen_t* ramp)
{
    ramp->count = 0;
}

void RampInit(RampGen_t* ramp, int32_t XSCALE)
{
    ramp->count = 0;
    ramp->XSCALE = XSCALE;
}

void RampSetCounter(struct RampGen_t* ramp, int32_t count)
{
    ramp->count = count;
}

uint8_t RampIsOverflow(struct RampGen_t* ramp)
{
    if(ramp->count >= ramp->XSCALE)
        return 1;
    else
        return 0;
}

//根据时间从-1到+1内循环输出
float RampCalcLoop(RampGenLoop_t* ramp)
{
    if(ramp->flag < 1)
    {
        ramp->count++;
    }
    else if(ramp->flag > -1)
    {
        ramp->count--;
    }

    if(ramp->count >= ramp->XSCALE)
    {
        ramp->count = ramp->XSCALE;
        ramp->flag = 1;
    }
    else if(ramp->count <= -ramp->XSCALE)
    {
        ramp->count = -ramp->XSCALE;
        ramp->flag = -1;
    }

    ramp->out = (1.0f * (ramp->count) / ramp->XSCALE);
    return ramp->out;
}
void RampSetScaleLoop(struct RampGenLoop_t* ramp, int32_t scale)
{
    ramp->XSCALE = scale;
}
void RampResetCounterLoop(struct RampGenLoop_t* ramp)
{
    ramp->count = 0;
    ramp->flag = 0;
}

void RampInitLoop(RampGenLoop_t* ramp, int32_t XSCALE)
{
    ramp->count = 0;
    ramp->flag = 0;
    ramp->XSCALE = XSCALE;
}
