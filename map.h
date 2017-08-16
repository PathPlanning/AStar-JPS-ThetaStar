#ifndef MAP_H
#define	MAP_H
#include <iostream>
#include "gl_const.h"
#include <sstream>
#include <string>
#include <algorithm>
class Map
{
    public:
        Map();
        Map(const Map& orig);
        ~Map();

        bool getMap(const char *FileName);
        bool CellIsTraversable (int i, int j) const;
        bool CellOnGrid (int i, int j) const;
        bool CellIsObstacle(int i, int j) const;
        void enumerateObstacles();
        void destroyObstacle(int n);
        int  getValue(int i, int j) const;

        int     height, width;
        int     start_i, start_j;
        int     goal_i, goal_j;
        int     second_si, second_sj, second_gi, second_gj;
        int     numberOfObs;
        double  cellSize;
        int**   Grid;
};

#endif

