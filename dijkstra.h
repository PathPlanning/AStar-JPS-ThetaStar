#ifndef DIJKSTRA_H
#define DIJKSTRA_H
#include "astar.h"

class Dijkstra : public Astar
{
    public:
        Dijkstra();

        double  computeHFromCellToCell(int start_i, int start_j, int fin_i, int fin_j, const EnvironmentOptions &options);
};
#endif
