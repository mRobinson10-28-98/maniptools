#include "SimClock.hpp"

void SimClock::SetSimFreq(double f)
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

double SimClock::GetSimFreq()
{
    return mFreq;
}
