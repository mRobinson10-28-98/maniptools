#ifndef _SIMCLOCK_HPP_
#define _SIMCLOCK_HPP_

#include <iostream>

class SimClock
{
private:
    SimClock() { std::cout << "SimClock singleton created.\n"; }
    SimClock(const SimClock&) = delete;
    SimClock& operator=(const SimClock&) = delete;

public:
    static SimClock& GetInstance()
    {
        static SimClock instance;
        return instance;
    }

    // Set frequency in Hz
    void SetSimFreq(uint f);
    void StepClock();
    double GetSimTime();

private:
    double mSimTime {0};

    uint mFreq;
    double mStepInc;
};

#endif //_SIMCLOCK_HPP_