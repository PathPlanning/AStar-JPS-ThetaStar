#ifndef SEARCHRESULT_H
#define SEARCHRESULT_H

#include "list.h"

struct SearchResult
{
        bool pathfound;
        float pathlength; //if path not found, then pathlength=0
        const NodeList* lppath;
        const NodeList* hppath;
        unsigned int nodescreated; //|OPEN| + |CLOSE| = total number of nodes saved in memory during search process.
        unsigned int numberofsteps; //number of iterations made by algorithm to find a solution
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
        }

};

#endif
