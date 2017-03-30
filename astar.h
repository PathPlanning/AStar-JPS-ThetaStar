#ifndef ASTAR_H
#define ASTAR_H
#include "isearch.h"

class Astar : public ISearch
{
    public:
        Astar(double HW, bool BT);

    protected:
        double computeHFromCellToCell(int i1, int j1, int i2, int j2, const EnvironmentOptions &options);
};

#endif
