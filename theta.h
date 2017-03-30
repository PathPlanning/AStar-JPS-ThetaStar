#ifndef THETA_H
#define THETA_H
#include "astar.h"

class Theta: public Astar
{
    public:
        Theta(double hweight, bool breakingties):Astar(hweight, breakingties){}
        ~Theta(void);

    protected:

        double distance(int i1, int j1, int i2, int j2);
        bool lineOfSight(int i1, int j1, int i2, int j2, const Map &map, bool cutcorners);
        Node resetParent(Node current, Node parent, const Map &map, const EnvironmentOptions &options);
        void makePrimaryPath(Node curNode);
        void makeSecondaryPath();

};


#endif // THETA_H
