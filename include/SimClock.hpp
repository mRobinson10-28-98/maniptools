#ifndef _SIMCLOCK_HPP_
#define _SIMCLOCK_HPP_

#include <iostream>

// Singleton class for tracking sim time between sim objects
class SimClock
{
private:
    SimClock();
    SimClock(const SimClock&) = delete;
    SimClock& operator=(const SimClock&) = delete;

public:
    static SimClock& GetInstance()
    {
        static SimClock instance;
        return instance;
    }

    // Set frequency in Hz
    void SetSimFreq(double f);
    void StepClock();

    double GetSimTime();
    double GetSimFreq();

private:
    double mSimTime {0.0};

    double mFreq {1.0};
    double mStepInc {1.0};
};

#endif //_SIMCLOCK_HPP_