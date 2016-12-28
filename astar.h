#ifndef ASTAR_H
#define ASTAR_H
#include "isearch.h"

class Astar : public ISearch
{
    public:
        Astar(double weight, bool BT);
        ~Astar();

    protected:
        double computeHFromCellToCell(int start_i, int start_j, int fin_i, int fin_j, const EnvironmentOptions &options);
        void   addOpen(Node newNode);
};

#endif
