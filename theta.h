#ifndef THETA_H
#define THETA_H
#include "astar.h"

class Theta: public Astar
{
    public:
        Theta(float hweight, bool breakingties, int sizelimit):Astar(hweight,breakingties, sizelimit){}
        ~Theta(void);


    private:

        bool lineOfSight(int x1, int y1, int x0, int y0, const Map &map);
        bool grid(int i, int j, const Map &map);
        Node resetParent(Node current, Node parent, const Map &map, const EnvironmentOptions &options);
        void makePrimaryPath(Node curNode);
        void makeSecondaryPath(const Map &map,Node curNode);


};


#endif // THETA_H
