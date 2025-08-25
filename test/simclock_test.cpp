#include <gtest/gtest.h>

#include "SimClock.hpp"

TEST(SimClock_test, SameInstance)
{
    SimClock& instance1 = SimClock::GetInstance();
    SimClock& instance2 = SimClock::GetInstance();

    // Check that both instances refer to the same memory address
    EXPECT_EQ(&instance1, &instance2);
}

TEST(SimClock_test, DataPersistenceTest) {
    SimClock& instance = SimClock::GetInstance();

    // Set frequency to 10 Hz so steps are in incremenets of 0.1s
    instance.SetSimFreq(10);

    // Step clock twice
    instance.StepClock();
    instance.StepClock();

    // Get the instance again and check the value persists
    SimClock& sameInstance = SimClock::GetInstance();
    EXPECT_EQ(sameInstance.GetSimTime(), instance.GetSimTime());
}
