#ifndef SEARCHRESULT_H
#define SEARCHRESULT_H
#include <list>
#include "node.h"
struct SearchResult
{
        bool pathfound;
        float pathlength; //if path not found, then pathlength=0
        const std::list<Node>* lppath;
        const std::list<Node>* hppath;
        std::list<Node>        fly_path;
        unsigned int nodescreated; //|OPEN| + |CLOSE| = total number of nodes saved in memory during search process.
        unsigned int numberofsteps; //number of iterations made by algorithm to find a solution
        float secondlength;
        double time;
        SearchResult()
        {
            pathfound = false;
            pathlength = 0;
            lppath = nullptr;
            hppath = nullptr;
            nodescreated = 0;
            numberofsteps = 0;
            time = 0;
            secondlength = 0;
        }

};

#endif
