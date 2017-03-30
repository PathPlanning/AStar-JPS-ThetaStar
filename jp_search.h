#ifndef JP_SEARCH_H
#define JP_SEARCH_H
#include "astar.h"

class JP_Search:public Astar
{
public:
    JP_Search(float hweight, bool breakingties):Astar(hweight, breakingties){}
    ~JP_Search();

private:
    bool findNeighbors(int move_i, int move_j, Node curNode, const Map &map, const EnvironmentOptions &options);//checks forced neighbors
    void findJP(int move_i, int move_j, Node curNode, const Map &map, std::list<Node> &successors, const EnvironmentOptions &options);//searches jump points
    int  findDirection(int current_i, int parent_i);//determines the direction of motion
    std::list<Node> findSuccessors(Node curNode, const Map &map, const EnvironmentOptions &options);
    void makePrimaryPath(Node curNode);
    void makeSecondaryPath();
};

#endif // JP_SEARCH_H
