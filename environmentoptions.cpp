#include "environmentoptions.h"

EnvironmentOptions::EnvironmentOptions()
{
    metrictype = CN_SP_MT_EUCL;
    allowsqueeze = CN_SP_AS_FALSE;
    linecost = CN_MC_LINE;
    diagonalcost = CN_MC_DIAG;
    allowdiagonal = CN_SP_AD_TRUE;
    useresetparent = CN_SP_RP_FALSE;
}

EnvironmentOptions::EnvironmentOptions(int MT, bool AS, double LC, double DC, bool AD, bool RP)
{
    metrictype = MT;
    allowsqueeze = AS;
    linecost = LC;
    diagonalcost = DC;
    allowdiagonal = AD;
    useresetparent = RP;
}

