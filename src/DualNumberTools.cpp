#include "DualNumberTools.hpp"

using namespace DualNumberTools;

DualVector DualVectorCrossProduct(DualVector d1, DualVector d2)
{
    DualVector d_cross;
    d_cross[0] = d1[1] * d2[2] - d1[2] * d2[1];
    d_cross[1] = d1[0] * d2[2] - d1[2] * d2[0];
    d_cross[2] = d1[0] * d2[1] - d1[1] * d2[0];
    
    return d_cross;
}

DualVector DualNumberDualVectorProduct(DualNumber d1, DualVector d2)
{

}