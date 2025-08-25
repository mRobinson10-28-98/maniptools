#include "SimClock.hpp"

void SimClock::SetSimFreq(uint f)
{
    mFreq = f;
    mStepInc = 1.0 / f;
}
void SimClock::StepClock()
{
    mSimTime += mStepInc;
}
double SimClock::GetSimTime()
{
    return mSimTime;
}