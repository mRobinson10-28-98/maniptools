#ifndef _PATH_PLANNING_TOOLS_HPP_
#define _PATH_PLANNING_TOOLS_HPP_

#include "DualNumberTools.hpp"

using namespace DualNumberTools;

namespace PathPlanningTools
{
    DualQuaternion ScLERP(DualQuaternion poseInitial, DualQuaternion poseFinal, double interpParam);
} // end namespace PathPlanningTools

#endif //_PATH_PLANNING_TOOLS_HPP_