#include "PathPlanningTools.hpp"

using namespace DualNumberTools;

DualQuaternion PathPlanningTools::ScLERP(DualQuaternion poseInitial, DualQuaternion poseFinal, double interpParam)
{
    return poseInitial * (poseInitial.conjugate() * poseFinal).pow(interpParam);
}
