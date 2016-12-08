#ifndef JP_SEARCH_H
#define JP_SEARCH_H
#include "astar.h"

class JP_Search:public Astar
{
public:
    JP_Search(float hweight, bool breakingties, int sizelimit):Astar(hweight,breakingties,sizelimit){}
    ~JP_Search();

private:
    bool findNeighbors(int move_i, int move_j, Node curNode, const Map &map);//checks forced neighbors
    void findJP(int move_i, int move_j, Node curNode, const Map &map, std::list<Node> &successors, const EnvironmentOptions &options);//searches jump points
    int findDirection(int current_i, int parent_i);//determines the direction of motion
    std::list<Node> findSuccessors(Node curNode, const Map &map, const EnvironmentOptions &options);
    Node resetParent(Node current, Node parent, const Map &map, const EnvironmentOptions &options);//additinal function that resets parent like Theta* and makes paths shorter
    void makePrimaryPath(Node curNode);
    void makeSecondaryPath(const Map &map,Node curNode);
};

#endif // JP_SEARCH_H
