#ifndef THETA_H
#define THETA_H
#include "astar.h"

class Theta: public Astar
{
    public:
        Theta(float hweight, bool breakingties, int sizelimit):Astar(hweight,breakingties, sizelimit){}
        ~Theta(void);


    protected:

        bool lineOfSight(int y1, int x1, int y0, int x0, const Map &map);
        Node resetParent(Node current, Node parent, const Map &map, const EnvironmentOptions &options);
        void makePrimaryPath(Node curNode);
        void makeSecondaryPath(const Map &map,Node curNode);


};


#endif // THETA_H
