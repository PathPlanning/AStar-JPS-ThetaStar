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

void Astar::addOpen(Node newNode)
{
    bool inserted = false;
    size_t idx = newNode.i;
    if (open[idx].find(newNode) != open[idx].end()) {
        if (newNode.F < open[idx].find(newNode)->F) {
            open[idx].erase(newNode);
            open[idx].insert(newNode);
            inserted = true;
        }
    } else {
        open[idx].insert(newNode);
        inserted = true;
        ++openSize;
    }

    if (inserted && newNode.F <= openMinimums[newNode.i].F) {
        if (newNode.F == openMinimums[idx].F) {
            switch (breakingties) {
                default:
                case CN_SP_BT_GMAX: {
                    if (newNode.g >= openMinimums[idx].g) {
                        openMinimums[idx] = newNode;
                    }
                    break;
                }
                case CN_SP_BT_GMIN: {
                    if (newNode.g <= openMinimums[idx].g) {
                        openMinimums[idx] = newNode;
                    }
                    break;
                }
            }
        } else {
            openMinimums[idx] = newNode;
        }
    }
}
