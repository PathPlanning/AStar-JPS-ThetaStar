#include "astar.h"

Astar::Astar(double HW, bool BT)
{
    hweight = HW;
    breakingties = BT;
}

double Astar::computeHFromCellToCell(int start_i, int start_j, int fin_i, int fin_j, const EnvironmentOptions &options)
{
    switch (options.metrictype) {
        case CN_SP_MT_MANH:
            return (abs(fin_i - start_i) + abs(fin_j - start_j));
        case CN_SP_MT_CHEB:
            return std::max(abs(fin_i - start_i),abs(fin_j - start_j));
        case CN_SP_MT_EUCL:
            return (sqrt((fin_i - start_i)*(fin_i - start_i)+(fin_j - start_j)*(fin_j - start_j)));
        case CN_SP_MT_DIAG:
            return (abs(abs(fin_i - start_i) - abs(fin_j - start_j)) + sqrt(2) * (std::min(abs(fin_i - start_i),abs(fin_j - start_j))));
        default:
            return 0;
    }
}
