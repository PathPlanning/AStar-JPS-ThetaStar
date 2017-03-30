#include "jp_search.h"

JP_Search::~JP_Search()
{
}

void JP_Search::findJP(int move_i, int move_j, Node curNode, const Map &map, std::list<Node> &successors, const EnvironmentOptions &options)
{
    bool findOK = false;
    while(!findOK) {
        if(map.CellOnGrid(curNode.i + move_i, curNode.j + move_j)) {
            if(!options.cutcorners) {
                if(move_i != 0 && move_j != 0)
                    if(!map.CellIsTraversable(curNode.i, curNode.j + move_j) || !map.CellIsTraversable(curNode.i + move_i, curNode.j))
                        return;
            }
            else if(!options.allowsqueeze) {
                if(move_i != 0 && move_j != 0)
                    if(!map.CellIsTraversable(curNode.i, curNode.j + move_j) && !map.CellIsTraversable(curNode.i + move_i, curNode.j))
                        return;
            }
            if(map.CellIsTraversable(curNode.i + move_i, curNode.j + move_j)) {
                curNode.i += move_i;
                curNode.j += move_j;
                if(move_i == 0 || move_j == 0)
                    curNode.g += 1;
                else
                    curNode.g += sqrt(2);
            }
            else
                return;
        }
        else
            return;
        if(map.goal_i == curNode.i && map.goal_j == curNode.j)
            findOK = true;
        if(options.allowdiagonal) { //check whether diagonal moves is allowed
            if(options.cutcorners) {
                if(move_i == 0) { //straight move along j
                    if(map.CellOnGrid(curNode.i + 1, curNode.j + move_j))
                        if(map.CellIsTraversable(curNode.i + 1, curNode.j + move_j) && map.CellIsObstacle(curNode.i + 1, curNode.j))
                            findOK = true;
                    if(map.CellOnGrid(curNode.i - 1,curNode.j + move_j))
                        if(map.CellIsTraversable(curNode.i - 1, curNode.j + move_j) && map.CellIsObstacle(curNode.i - 1, curNode.j))
                            findOK = true;
                }
                else if(move_j == 0) { //straight move along i
                    if(map.CellOnGrid(curNode.i + move_i, curNode.j + 1))
                        if(map.CellIsTraversable(curNode.i + move_i, curNode.j + 1) && map.CellIsObstacle(curNode.i, curNode.j + 1))
                            findOK = true;
                    if(map.CellOnGrid(curNode.i + move_i, curNode.j - 1))
                        if(map.CellIsTraversable(curNode.i + move_i, curNode.j - 1) && map.CellIsObstacle(curNode.i, curNode.j - 1))
                            findOK = true;
                }
                else { //diagonal move
                    if(map.CellOnGrid(curNode.i - move_i, curNode.j + move_j))
                        if(map.CellIsObstacle(curNode.i - move_i, curNode.j) && map.CellIsTraversable(curNode.i - move_i, curNode.j + move_j))
                            findOK = true;
                    if(!findOK && map.CellOnGrid(curNode.i + move_i, curNode.j - move_j))
                        if(map.CellIsObstacle(curNode.i, curNode.j - move_j) && map.CellIsTraversable(curNode.i + move_i, curNode.j - move_j))
                            findOK = true;
                    if(!findOK)
                        if(findNeighbors(move_i, 0, curNode, map, options))
                            findOK = true;
                    if(!findOK)
                        if(findNeighbors(0, move_j, curNode, map, options))
                            findOK = true;
                }
            }
            else
            {
                if(move_i == 0) { //straight move along j
                    if(map.CellOnGrid(curNode.i + 1, curNode.j))
                        if(map.CellIsTraversable(curNode.i + 1, curNode.j) && map.CellIsObstacle(curNode.i + 1, curNode.j - move_j))//check forced neighbor
                            findOK = true;
                    if(map.CellOnGrid(curNode.i - 1,curNode.j))
                        if(map.CellIsTraversable(curNode.i - 1, curNode.j) && map.CellIsObstacle(curNode.i - 1, curNode.j - move_j))
                            findOK = true;
                }
                else if(move_j == 0) { //straight move along i
                    if(map.CellOnGrid(curNode.i, curNode.j + 1))
                        if(map.CellIsTraversable(curNode.i, curNode.j + 1) && map.CellIsObstacle(curNode.i - move_i, curNode.j + 1))
                            findOK = true;
                    if(map.CellOnGrid(curNode.i, curNode.j - 1))
                        if(map.CellIsTraversable(curNode.i, curNode.j - 1) && map.CellIsObstacle(curNode.i - move_i, curNode.j - 1))
                            findOK = true;
                }
                else { //diagonal move
                    if(findNeighbors(move_i, 0, curNode, map, options))//looking for forced neighbor along i
                        findOK = true;
                    if(!findOK)
                        if(findNeighbors(0, move_j, curNode, map, options))//looking for forced neighbor along j
                            findOK = true;
                }
            }
        }
        else { //only straight moves is allowed
            if(!findOK)
                if(findNeighbors(move_j, move_i, curNode, map, options))
                    findOK = true;
            if(!findOK)
                if(findNeighbors(-move_j, -move_i, curNode, map, options))
                    findOK = true;
        }
    }
    if(close.find(curNode.i * map.width + curNode.j) == close.end())
        successors.push_front(curNode);
    return;
}

bool JP_Search::findNeighbors(int move_i, int move_j, Node curNode, const Map &map, const EnvironmentOptions &options)
{
    while(map.CellOnGrid(curNode.i, curNode.j) && map.CellIsTraversable(curNode.i, curNode.j)) {
        if(map.goal_i == curNode.i && map.goal_j == curNode.j)//goal location is found
            return true;
        if(options.cutcorners) {
            if(move_i == 0 && map.CellOnGrid(curNode.i, curNode.j+move_j)) {
                if(map.CellOnGrid(curNode.i + 1, curNode.j))
                    if(map.CellIsTraversable(curNode.i + 1, curNode.j+move_j) && map.CellIsObstacle(curNode.i + 1, curNode.j))
                        return true;
                if(map.CellOnGrid(curNode.i - 1,curNode.j))
                    if(map.CellIsTraversable(curNode.i - 1, curNode.j+move_j) && map.CellIsObstacle(curNode.i - 1, curNode.j))
                        return true;
            }
            if(move_j == 0 && map.CellOnGrid(curNode.i + move_i, curNode.j)) {
                if(map.CellOnGrid(curNode.i, curNode.j + 1))
                    if(map.CellIsTraversable(curNode.i + move_i, curNode.j + 1) && map.CellIsObstacle(curNode.i, curNode.j + 1))
                        return true;
                if(map.CellOnGrid(curNode.i, curNode.j - 1))
                    if(map.CellIsTraversable(curNode.i + move_i, curNode.j - 1) && map.CellIsObstacle(curNode.i, curNode.j - 1))
                        return true;
            }
        }
        else {
            if(move_i == 0 && map.CellOnGrid(curNode.i, curNode.j - move_j)) {
                if(map.CellOnGrid(curNode.i + 1, curNode.j))
                    if(map.CellIsTraversable(curNode.i + 1, curNode.j) && map.CellIsObstacle(curNode.i + 1, curNode.j - move_j))
                        return true;
                if(map.CellOnGrid(curNode.i - 1,curNode.j))
                    if(map.CellIsTraversable(curNode.i - 1, curNode.j) && map.CellIsObstacle(curNode.i - 1, curNode.j - move_j))
                        return true;
            }
            if(move_j == 0 && map.CellOnGrid(curNode.i - move_i, curNode.j)) {
                if(map.CellOnGrid(curNode.i, curNode.j + 1))
                    if(map.CellIsTraversable(curNode.i, curNode.j + 1) && map.CellIsObstacle(curNode.i - move_i, curNode.j + 1))
                        return true;
                if(map.CellOnGrid(curNode.i, curNode.j - 1))
                    if(map.CellIsTraversable(curNode.i, curNode.j - 1) && map.CellIsObstacle(curNode.i - move_i, curNode.j - 1))
                        return true;
            }
        }
        curNode.i += move_i;
        curNode.j += move_j;
    }
    return false;
}



int JP_Search::findDirection(int current_i, int parent_i)
{
    if(current_i < parent_i)
        return -1;
    else if(current_i > parent_i)
        return 1;
    else
        return 0;
}

std::list<Node> JP_Search::findSuccessors(Node curNode, const Map &map, const EnvironmentOptions &options)
{
    int move_i = 0, move_j = 0;
    std::list<Node> successors;

    if(options.allowdiagonal) {
        if(curNode.i == map.start_i && curNode.j == map.start_j)//if curNode is the start location, then look for jump points in all directions
            for(int n = -1; n <= 1; n++)
                for(int m = -1; m <= 1; m++)
                    if(n != 0 || m != 0)
                        findJP(n, m, curNode, map, successors, options);
        if(curNode.i != map.start_i || curNode.j != map.start_j) {
            move_i = findDirection(curNode.i, curNode.parent->i);
            move_j = findDirection(curNode.j, curNode.parent->j);
            findJP(move_i, move_j, curNode, map, successors, options);//continue to look for jump points in the same direction

            if(move_i != 0 && move_j != 0) { //if curNoode is a diagonal jump point
                if(map.CellIsObstacle(curNode.i - move_i, curNode.j))
                    findJP(-move_i, move_j, curNode, map, successors, options);
                if(map.CellIsObstacle(curNode.i, curNode.j - move_j))
                    findJP(move_i, -move_j, curNode, map, successors, options);
                findJP(move_i, 0, curNode, map, successors, options);//look for jump point in straight direction along i
                findJP(0, move_j, curNode, map, successors, options);//the same check along j
            }

            if(options.cutcorners) { //original JPS, when cutcorners is allowed
                if(move_i == 0) {
                    if(map.CellOnGrid(curNode.i - move_j, curNode.j))
                        if(map.CellIsObstacle(curNode.i - move_j, curNode.j))
                            findJP(-move_j, move_j, curNode, map, successors, options);
                    if(map.CellOnGrid(curNode.i+move_j, curNode.j))
                        if(map.CellIsObstacle(curNode.i + move_j, curNode.j))
                            findJP(move_j, move_j, curNode, map, successors, options);
                }
                else if(move_j == 0) {
                    if(map.CellOnGrid(curNode.i, curNode.j - move_i))
                        if(map.CellIsObstacle(curNode.i, curNode.j - move_i))
                            findJP(move_i, -move_i, curNode, map, successors, options);
                    if(map.CellOnGrid(curNode.i, curNode.j + move_i))
                        if(map.CellIsObstacle(curNode.i, curNode.j + move_i))
                            findJP(move_i, move_i, curNode, map, successors, options);
                }
            }
            else { //cutcorners disallowed
                if(move_i == 0) {
                    if(map.CellOnGrid(curNode.i - move_j, curNode.j))
                        if(map.CellIsObstacle(curNode.i - move_j, curNode.j - move_j)) {
                            findJP(-move_j, move_j, curNode, map, successors, options);
                            findJP(-move_j, 0, curNode, map, successors, options);
                        }
                    if(map.CellOnGrid(curNode.i + move_j, curNode.j))
                        if(map.CellIsObstacle(curNode.i + move_j, curNode.j - move_j)) {
                            findJP(move_j, 0, curNode, map, successors, options);
                            findJP(move_j, move_j, curNode, map, successors, options);
                        }
                }
                else if(move_j == 0) {
                    if(map.CellOnGrid(curNode.i, curNode.j-move_i))
                        if(map.CellIsObstacle(curNode.i - move_i, curNode.j - move_i)) {
                            findJP(0, -move_i, curNode, map, successors, options);//additional check
                            findJP(move_i, -move_i, curNode, map, successors, options);
                        }
                    if(map.CellOnGrid(curNode.i, curNode.j + move_i))
                        if(map.CellIsObstacle(curNode.i - move_i, curNode.j + move_i)) {
                            findJP(0, move_i, curNode, map, successors, options);//additional check
                            findJP(move_i, move_i, curNode, map, successors, options);
                        }
                }
            }
        }
    }
    else { //only straight moves as allowed
        if(curNode.i == map.start_i && curNode.j == map.start_j)
            for(int n = -1; n <= 1; n++)
                for(int m = -1; m <= 1; m++)
                    if((n != 0 && m == 0) || (n == 0 && m != 0))
                        findJP(n, m, curNode, map, successors, options);
        if(curNode.i != map.start_i || curNode.j != map.start_j) {
            move_i = findDirection(curNode.i, curNode.parent->i);
            move_j = findDirection(curNode.j, curNode.parent->j);

            findJP(move_i, move_j, curNode, map, successors, options);
            findJP(move_j, move_i, curNode, map, successors, options);
            findJP(-move_j, -move_i, curNode, map, successors, options);
        }
    }
    return successors;
}

void JP_Search::makePrimaryPath(Node curNode)
{
    Node current = curNode;
    while(current.parent) {
        hppath.push_front(current);
        current = *current.parent;
    }
    hppath.push_front(current);
}

void JP_Search::makeSecondaryPath()
{
    Node pathNode = *hppath.begin();
    std::list<Node>::iterator iter = hppath.begin();
    iter++;
    Node nextNode = *iter;
    Node inpath;
    lppath.push_back(pathNode);
    while(iter != hppath.end()) {
        int steps = std::max(std::abs(pathNode.i - nextNode.i), std::abs(pathNode.j - nextNode.j));
        int step_i = (pathNode.i < nextNode.i) ? 1 : ((pathNode.i > nextNode.i) ? -1 : 0);
        int step_j = (pathNode.j < nextNode.j) ? 1 : ((pathNode.j > nextNode.j) ? -1 : 0);
        for (int k = 0; k <= steps; k++) {
                inpath.i = pathNode.i + k*step_i;
                inpath.j = pathNode.j + k*step_j;
                lppath.push_back(inpath);
            }
        pathNode = nextNode;
        iter++;
        nextNode = *iter;
    }
}
