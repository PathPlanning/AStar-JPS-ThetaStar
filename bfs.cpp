#include "bfs.h"

BFS::BFS() {
    breakingties = CN_SP_BT_GMIN;
}

double BFS::computeHFromCellToCell(int start_i, int start_j, int fin_i, int fin_j, const EnvironmentOptions &options) {
    return 0;
}

void BFS::addOpen(Node newNode) {
    newNode.H = 0;
    newNode.F = openSize + close.size();

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

    if (inserted &&
        (newNode.F < openMinimums[newNode.i].F || newNode.F == openMinimums[idx].F && newNode.g < openMinimums[idx].g)) {
        openMinimums[idx] = newNode;
    }
}
