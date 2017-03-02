#ifndef BFS_H
#define BFS_H
#include "isearch.h"

class BFS : public ISearch
{
    public:
        BFS();

        void    addOpen(Node newNode);
        double  computeHFromCellToCell(int start_i, int start_j, int fin_i, int fin_j, const EnvironmentOptions &options);
};

#endif
