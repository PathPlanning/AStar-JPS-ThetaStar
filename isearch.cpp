#include "isearch.h"
#include <vector>
#include <math.h>
#include <limits>
#include <chrono>

ISearch::ISearch() {
    hweight = 1;
    breakingties = CN_SP_BT_GMAX;
    open = NULL;
    openSize = 0;
}

ISearch::~ISearch(void) {
    if (open) delete[]open;
}

double ISearch::MoveCost(int start_i, int start_j, int fin_i, int fin_j, const EnvironmentOptions &options) {
    if ((start_i - fin_i) != 0 && (start_j - fin_j) != 0)
        return sqrt(2);
    return 1;
}

bool ISearch::stopCriterion() {
    if (openSize == 0) {
        std::cout << "OPEN list is empty!" << std::endl;
        return true;
    }
    return false;
}

SearchResult ISearch::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options) {
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    open = new std::list<Node>[map.height];
    Node curNode;
    curNode.i = map.start_i;
    curNode.j = map.start_j;
    curNode.g = 0;
    curNode.H = computeHFromCellToCell(curNode.i, curNode.j, map.goal_i, map.goal_j, options);
    curNode.F = hweight * curNode.H;
    curNode.parent = 0;
    addOpen(curNode);
    int closeSize = 0;
    bool pathfound = false;
    while (!stopCriterion()) {
        curNode = findMin(map.height);
        close.insert({curNode.i * map.width + curNode.j, curNode});
        closeSize++;
        open[curNode.i].pop_front();
        openSize--;
        if (curNode.i == map.goal_i && curNode.j == map.goal_j) {
            pathfound = true;
            break;
        }
        std::list<Node> successors = findSuccessors(curNode, map, options);
        std::list<Node>::iterator it = successors.begin();
        auto parent = &(close.find(curNode.i * map.width + curNode.j)->second);
        while (it != successors.end()) {
            it->parent = parent;
            it->H = computeHFromCellToCell(it->i, it->j, map.goal_i, map.goal_j, options);
            *it = resetParent(*it, *it->parent, map, options);
            it->F = it->g + hweight * it->H;
            addOpen(*it);
            it++;
        }
        Logger->writeToLogOpenClose(open, close, map.height, false);
    }
    Logger->writeToLogOpenClose(open, close, map.height, true);
    sresult.pathfound = false;
    sresult.nodescreated = closeSize + openSize;
    sresult.numberofsteps = closeSize;
    if (pathfound) {
        sresult.pathfound = true;
        makePrimaryPath(curNode);
        sresult.hppath = &hppath;
        sresult.pathlength = curNode.g;
    }
    //stop the timer now because making path using back pointers is a part of the algorithm
    end = std::chrono::system_clock::now();
    sresult.time =
            static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()) / 1000000000;
    if (pathfound)
        makeSecondaryPath(map, curNode);
    return sresult;
}

Node ISearch::findMin(int size) {
    Node min;
    min.F = std::numeric_limits<double>::infinity();
    for (int i = 0; i < size; i++) {
        if (!open[i].empty())
            if (open[i].begin()->F <= min.F) {
                if (open[i].begin()->F == min.F) {
                    switch (breakingties) {
                        case CN_SP_BT_GMAX: {
                            if (open[i].begin()->g >= min.g) {
                                min = *open[i].begin();
                            }
                            break;
                        }
                        case CN_SP_BT_GMIN: {
                            if (open[i].begin()->g <= min.g) {
                                min = *open[i].begin();
                            }
                            break;
                        }
                    }
                } else
                    min = *open[i].begin();
            }
    }
    return min;
}

std::list<Node> ISearch::findSuccessors(Node curNode, const Map &map, const EnvironmentOptions &options) {
    Node newNode;
    std::list<Node> successors;
    for (int i = -1; i <= +1; i++) {
        for (int j = -1; j <= +1; j++) {
            if ((i != 0 || j != 0) && map.CellOnGrid(curNode.i + i, curNode.j + j) &&
                (map.CellIsTraversable(curNode.i + i, curNode.j + j))) {
                if (!options.allowdiagonal) {
                    if (i != 0 && j != 0)
                        continue;
                } else if (!options.cutcorners) {
                    if (i != 0 && j != 0)
                        if (map.CellIsObstacle(curNode.i, curNode.j + j) ||
                            map.CellIsObstacle(curNode.i + i, curNode.j))
                            continue;
                } else if (!options.allowsqueeze) {
                    if (i != 0 && j != 0)
                        if (map.CellIsObstacle(curNode.i, curNode.j + j) &&
                            map.CellIsObstacle(curNode.i + i, curNode.j))
                            continue;
                }
                if (close.find((curNode.i + i) * map.width + curNode.j + j) == close.end()) {
                    newNode.i = curNode.i + i;
                    newNode.j = curNode.j + j;
                    newNode.g = curNode.g + MoveCost(curNode.i, curNode.j, curNode.i + i, curNode.j + j, options);
                    successors.push_front(newNode);
                }
            }
        }
    }
    return successors;
}

void ISearch::makePrimaryPath(Node curNode) {
    Node current = curNode;
    while (current.parent) {
        lppath.List.push_front(current);
        current = *current.parent;
    }
    lppath.List.push_front(current);
    sresult.lppath = &lppath; //Here is a constant pointer
}

void ISearch::makeSecondaryPath(const Map &map, Node curNode) {
    std::list<Node>::const_iterator iter = lppath.List.begin();
    int curI, curJ, nextI, nextJ, moveI, moveJ;
    hppath.List.push_back(*iter);

    while (iter != --lppath.List.end()) {
        curI = iter->i;
        curJ = iter->j;
        ++iter;
        nextI = iter->i;
        nextJ = iter->j;
        moveI = nextI - curI;
        moveJ = nextJ - curJ;
        ++iter;
        if ((iter->i - nextI) != moveI || (iter->j - nextJ) != moveJ)
            hppath.List.push_back(*(--iter));
        else
            --iter;
    }
    sresult.hppath = &hppath;
}

void ISearch::addOpen(Node newNode)
{
    std::list<Node>::iterator iter,pos;

    bool posFound=false;

    pos = open[newNode.i].end();

    if (open[newNode.i].size() == 0)
    {
        open[newNode.i].push_back(newNode);
        openSize++;
        return;
    }

    for(iter=open[newNode.i].begin(); iter != open[newNode.i].end(); ++iter)
    {
        if ((iter->F >= newNode.F) && (!posFound))
        {
            if (iter->F == newNode.F)
            {
                switch(breakingties)
                {
                    default:
                    case CN_SP_BT_GMAX:
                    {
                        if (newNode.g >= iter->g)
                        {
                            pos=iter;
                            posFound=true;
                        }
                        break;
                    }
                    case CN_SP_BT_GMIN:
                    {
                        if (newNode.g <= iter->g)
                        {
                            pos=iter;
                            posFound=true;
                        }
                        break;
                    }
                }
            }
            else
            {
                pos=iter;
                posFound=true;
            }
        }

        if (((iter->i) == newNode.i) && (iter->j)==newNode.j)
        {
            if (newNode.F >= iter->F)
            {
                return;
            }
            else
            {
                if(pos == iter)
                {
                    iter->F = newNode.F;
                    iter->g = newNode.g;
                    iter->parent = newNode.parent;
                    return;
                }
                open[newNode.i].erase(iter);
                openSize--;
                break;
            }
        }
    }
    openSize++;
    open[newNode.i].insert(pos,newNode);
}
