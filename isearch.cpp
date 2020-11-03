#include "isearch.h"
#include <vector>
#include <math.h>
#include <limits>
#include <chrono>

ISearch::ISearch()
{
    hweight = 1;
    breakingties = CN_SP_BT_GMAX;
    openSize = 0;
    focal_weight = 1.5;
}

ISearch::~ISearch(void) {}

bool ISearch::stopCriterion()
{
    if (open.empty()) {
        std::cout << "OPEN list is empty!" << std::endl;
        return true;
    }
    return false;
}

void ISearch::printLists()
{
    /*for (int i = 0; i < open.size(); i++)
        for(auto it = open[i].begin(); it != open[i].end(); it++)
            std::cout<<it->i<<","<<it->j<<"  ";
    std::cout<<"\n\n";
    for(auto it = focal.begin(); it!=focal.end(); it++)
        std::cout<<it->i<<","<<it->j<<"  ";
    std::cout<<"\n\n";*/
}

SearchResult ISearch::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    Node curNode(map.start_i, map.start_j, 0, computeHFromCellToCell(map.start_i, map.start_j, map.goal_i, map.goal_j, options), map.start_i*map.width + map.start_j, nullptr);
    fmin = curNode.f;
    addOpen(curNode, map);
    int closeSize = 0;

    bool pathfound = false;
    while (!stopCriterion()) {
        double oldmin = fmin;
        //printLists();
        curNode = findMin();
        if(oldmin != fmin)
            updateFocal(map);
        //std::cout<<curNode.i<<" "<<curNode.j<<" "<<curNode.g<<" "<<curNode.h<<" "<<curNode.f<<" "<<map.getHValue(curNode.i, curNode.j)<<" \n";
        close.insert({curNode.i * map.width + curNode.j, curNode});
        if (curNode.i == map.goal_i && curNode.j == map.goal_j) {
            pathfound = true;
            break;
        }
        std::list<Node> successors = findSuccessors(curNode, map, options);
        std::list<Node>::iterator it = successors.begin();
        auto parent = &(close.find(curNode.i * map.width + curNode.j)->second);
        while (it != successors.end()) {
            it->parent = parent;
            it->h = computeHFromCellToCell(it->i, it->j, map.goal_i, map.goal_j, options);
            *it = resetParent(*it, *it->parent, map, options);
            it->f = it->g + hweight * it->h;
            addOpen(*it, map);
            it++;
        }
        //Logger->writeToLogOpenClose(open, close, false);
    }
    //Logger->writeToLogOpenClose(open, close, true);
    sresult.pathfound = false;
    sresult.nodescreated = close.size() + open.size();
    sresult.numberofsteps = close.size();
    if (pathfound) {
        sresult.pathfound = true;
        makePrimaryPath(curNode);
        sresult.pathlength = curNode.g;
    }
    //Stop the timer now because making path using back pointers is a part of the algorithm
    end = std::chrono::system_clock::now();
    sresult.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()) / 1000000000;
    if (pathfound)
        makeSecondaryPath();
    sresult.hppath = &hppath; //Here is a constant pointer
    sresult.lppath = &lppath;
    return sresult;
}


Node ISearch::findMin()
{
    Node min;
    FocalElem elem = *focal.get<0>().begin();
    auto min_it = open.get<1>().find(elem.id);
    min = *min_it;
    open.get<1>().erase(min_it);
    focal.get<0>().erase(focal.get<0>().begin());
    if(!open.empty())
        fmin = open.get<0>().begin()->f;
    return min;
}

std::list<Node> ISearch::findSuccessors(Node curNode, const Map &map, const EnvironmentOptions &options)
{
    Node newNode;
    std::list<Node> successors;
    for (int i = -1; i <= +1; i++)
        for (int j = -1; j <= +1; j++)
            if ((i != 0 || j != 0) && map.CellOnGrid(curNode.i + i, curNode.j + j) &&
                    (map.CellIsTraversable(curNode.i + i, curNode.j + j))) {
                if (i != 0 && j != 0) {
                    if (!options.allowdiagonal)
                        continue;
                    else if (!options.cutcorners) {
                        if (map.CellIsObstacle(curNode.i, curNode.j + j) ||
                                map.CellIsObstacle(curNode.i + i, curNode.j))
                            continue;
                    }
                    else if (!options.allowsqueeze) {
                        if (map.CellIsObstacle(curNode.i, curNode.j + j) &&
                                map.CellIsObstacle(curNode.i + i, curNode.j))
                            continue;
                    }
                }
                if (close.find((curNode.i + i) * map.width + curNode.j + j) == close.end()) {
                    newNode.i = curNode.i + i;
                    newNode.j = curNode.j + j;
                    newNode.id = newNode.i*map.width + newNode.j;
                    if(i == 0 || j == 0)
                        newNode.g = curNode.g + 1;
                    else
                        newNode.g = curNode.g + sqrt(2);
                    successors.push_front(newNode);
                }
            }
    return successors;
}

void ISearch::makePrimaryPath(Node curNode)
{
    Node current = curNode;
    while (current.parent) {
        lppath.push_front(current);
        current = *current.parent;
    }
    lppath.push_front(current);
}

void ISearch::makeSecondaryPath()
{
    std::list<Node>::const_iterator iter = lppath.begin();
    int curI, curJ, nextI, nextJ, moveI, moveJ;
    hppath.push_back(*iter);
    while (iter != --lppath.end()) {
        curI = iter->i;
        curJ = iter->j;
        ++iter;
        nextI = iter->i;
        nextJ = iter->j;
        moveI = nextI - curI;
        moveJ = nextJ - curJ;
        ++iter;
        if ((iter->i - nextI) != moveI || (iter->j - nextJ) != moveJ)
            hppath.push_back(*(--iter));
        else
            --iter;
    }
}

void ISearch::updateFocal(const Map &map)
{
    //std::cout<<"UPDATE\n";
    if(overbounded_elements.empty())
        return;
    auto it = overbounded_elements.get<0>().begin();
    int erase_num(0);
    while(it != overbounded_elements.get<0>().end())
    {
        if(it->f - 1e-6 < fmin*focal_weight)
        {
            addFocal(*it, true);
            it++;
            erase_num++;
            continue;
        }
        break;
    }
    if(it == overbounded_elements.get<0>().end())
        overbounded_elements.clear();
    else
        for(int i = 0; i < erase_num; i++)
            overbounded_elements.get<0>().erase(overbounded_elements.get<0>().begin());
    return;
}

void ISearch::addFocal(FocalElem elem, bool was_in_open)
{
    if(!was_in_open)
    {
        focal.insert(elem);
        return;
    }
    auto in_focal = focal.get<1>().find(elem.id);
    if(in_focal != focal.get<1>().end())
    {
        if(in_focal->g > elem.g)
        {
            focal.get<1>().erase(in_focal);
            focal.insert(elem);
        }
    }
    else
    {
        overbounded_elements.get<1>().erase(overbounded_elements.get<1>().find(elem.id));
        focal.insert(elem);
    }
    return;
}

void ISearch::addOverbounded(FocalElem elem, bool was_in_open)
{
    if(was_in_open)
        overbounded_elements.get<1>().erase(overbounded_elements.get<1>().find(elem.id));
    overbounded_elements.insert(elem);
    return;
}

void ISearch::addOpen(Node newNode, const Map &map)
{
    auto old = open.get<1>().find(newNode.id);
    bool was_in_open(false);
    if(old != open.get<1>().end())
    {
        if(old->g < newNode.g + 1e-6)
            return;
        else
        {
            open.get<1>().erase(old);
            was_in_open = true;
        }
    }
    open.insert(newNode);
    if(newNode.f - 1e-6 < fmin*focal_weight)
        addFocal(FocalElem(newNode, map.getHValue(newNode.i, newNode.j)), was_in_open);
    else
        addOverbounded(FocalElem(newNode, map.getHValue(newNode.i, newNode.j)), was_in_open);
    return;
}
