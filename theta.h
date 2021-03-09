#ifndef THETA_H
#define THETA_H
#include "astar.h"

class Theta: public Astar
{
    public:
        Theta(double hweight, bool breakingties):Astar(hweight, breakingties){}
        ~Theta(void);
        static bool lineOfSight(int i1, int j1, int i2, int j2, const Map &map, bool cutcorners);
        static double distance(int i1, int j1, int i2, int j2);
    protected:
        
        
        Node resetParent(Node current, Node parent, const Map &map, const EnvironmentOptions &options);
        void makePrimaryPath(Node curNode);
        void makeSecondaryPath();

};


#endif // THETA_H
