#include "bfs.h"

BFS::BFS()
{
    breakingties = CN_SP_BT_GMIN;
}

double BFS::computeHFromCellToCell(int start_i, int start_j, int fin_i, int fin_j, const EnvironmentOptions &options)
{
    return 0;
}

void BFS::addOpen(Node newNode)
{
    std::list<Node>::iterator iter=open[newNode.i].begin();

    while(iter != open[newNode.i].end() && newNode.j != iter->j)
        ++iter;

    if(iter != open[newNode.i].end()) {
        if(iter->g > newNode.g) {
            open[newNode.i].erase(iter);
            openSize--;
        }
        else
            return;
    }
    openSize++;
    open[newNode.i].push_back(newNode);
    return;
}
