#include "environmentoptions.h"

EnvironmentOptions::EnvironmentOptions()
{
    metrictype = CN_SP_MT_EUCL;
    allowsqueeze = false;
    allowdiagonal = true;
    cutcorners = false;
}

EnvironmentOptions::EnvironmentOptions(bool AS, bool AD, bool CC, int MT)
{
    metrictype = MT;
    allowsqueeze = AS;
    allowdiagonal = AD;
    cutcorners = CC;
}

