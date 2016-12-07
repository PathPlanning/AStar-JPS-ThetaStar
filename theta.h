#ifndef THETA_H
#define THETA_H
#include "gl_const.h"
#include "astar.h"

class Theta: public Astar
{
    public:
        Theta(float hweight, int breakingties, int sizelimit, int i):Astar(hweight,breakingties, sizelimit, i){/*open=new NodeList[i];*/}
        ~Theta(void);


    private:

        bool lineOfSight(int x1, int y1, int x0, int y0, const Map &map);//проверяет существование линии между двумя точками
        bool grid(int i, int j, const Map &map);//проверяет принадлежность точки к карте и проходимость
        Node resetParent(Node current, Node parent, const Map &map, const EnvironmentOptions &options);
        void makePrimaryPath(Node curNode);
        void makeSecondaryPath(const Map &map,Node curNode);


};


#endif // THETA_H
