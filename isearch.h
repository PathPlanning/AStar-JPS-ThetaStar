#ifndef ISEARCH_H
#define ISEARCH_H
#include "ilogger.h"
#include "searchresult.h"
#include "environmentoptions.h"
#include <vector>
#include <math.h>
#include <limits>
#include <chrono>

class ISearch
{
    public:
        ISearch();
        virtual ~ISearch(void);

        SearchResult startSearch(ILogger *Logger, const Map &Map, const EnvironmentOptions &options);
        Node findMin(int size);
        double MoveCost(int start_i, int start_j, int fin_i, int fin_j, const EnvironmentOptions &options);

    protected:
        virtual void addOpen(Node newNode) = 0;
        virtual double computeHFromCellToCell(int start_i, int start_j, int fin_i, int fin_j, const EnvironmentOptions &options) = 0;
        virtual std::list<Node> findSuccessors(Node curNode, const Map &map, const EnvironmentOptions &options);
        virtual void makePrimaryPath(Node curNode);//Makes path using back pointers
        virtual void makeSecondaryPath(const Map &map, Node curNode);//Makes another type of path(sections or points)
        virtual Node resetParent(Node current, Node parent, const Map &map, const EnvironmentOptions &options) {return current;}//Function for Theta* and modification of JPS
        virtual bool stopCriterion();

        SearchResult                    sresult;
        NodeList                        lppath, hppath;
        std::unordered_map<int,Node>    close;
        NodeList                        *open;
        int                             openSize;
        int                             sizelimit;//max size of OPEN list //TODO:remove it
        float                           hweight;//weight of h-value
        bool                            breakingties;//flag that sets the priority of nodes in addOpen function when their F-values is equal

};
#endif
