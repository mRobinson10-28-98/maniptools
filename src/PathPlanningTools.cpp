#include "PathPlanningTools.hpp"

using namespace DualNumberTools;

DualQuaternion PathPlanningTools::ScLERP(DualQuaternion poseInitial, DualQuaternion poseFinal, double interpParam)
{
    DualQuaternion q1 = (poseInitial.conjugate() * poseFinal).pow(interpParam);
    return poseInitial * q1;
}
