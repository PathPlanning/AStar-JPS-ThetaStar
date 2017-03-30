#include "astar.h"

Astar::Astar(double HW, bool BT)
{
    hweight = HW;
    breakingties = BT;
}

double Astar::computeHFromCellToCell(int i1, int j1, int i2, int j2, const EnvironmentOptions &options)
{
    switch (options.metrictype) {
        case CN_SP_MT_EUCL:
            return (sqrt((i2 - i1)*(i2 - i1)+(j2 - j1)*(j2 - j1)));
        case CN_SP_MT_DIAG:
            return (abs(abs(i2 - i1) - abs(j2 - j1)) + sqrt(2) * (std::min(abs(i2 - i1),abs(j2 - j1))));
        case CN_SP_MT_MANH:
            return (abs(i2 - i1) + abs(j2 - j1));
        case CN_SP_MT_CHEB:
            return std::max(abs(i2 - i1),abs(j2 - j1));
        default:
            return 0;
    }
}
