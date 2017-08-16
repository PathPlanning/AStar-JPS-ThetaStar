#ifndef ISEARCH_H
#define ISEARCH_H
#include "ilogger.h"
#include "searchresult.h"
#include "environmentoptions.h"
#include <list>
#include <unordered_map>
#include <map>

class ISearch
{
    public:
        ISearch();
        virtual ~ISearch(void);

        SearchResult startSearch(ILogger *Logger, Map &Map, const EnvironmentOptions &options, bool replan);

    protected:
        virtual void addOpen(Node newNode);
        Node findMin(int size);
        virtual double computeHFromCellToCell(int start_i, int start_j, int fin_i, int fin_j, const EnvironmentOptions &options) = 0;
        virtual std::list<Node> findSuccessors(Node curNode, const Map &map, const EnvironmentOptions &options);
        virtual void makePrimaryPath(Node curNode);//Makes path using back pointers
        virtual void makeSecondaryPath();//Makes another type of path(sections or points)
        virtual Node resetParent(Node current, Node parent, const Map &map, const EnvironmentOptions &options) {return current;}//Function for Theta*
        virtual bool stopCriterion();
        void findBlockObstacle(const Map &map, std::pair<int, int> boom, int numOfObs);
        int chooseObstacleToDestroy(const Map& map);
        void findFlyPath(const Map& map);
        SearchResult                     sresult;
        std::list<Node>                  lppath, hppath;
        std::unordered_map<int,Node>     close;
        std::vector<std::list<Node>>     open;
        int                              openSize;
        double                           hweight;//weight of h-value
        bool                             breakingties;//flag that sets the priority of nodes in addOpen function when their F-values is equal
        std::vector<int>                 numberOfBlocks;
        std::vector<std::pair<int, int>> blockedCells;
        std::map<int,std::pair<double,double> > obs;
        std::vector<std::map<int,std::pair<double,double>>> allObs;
        int                              unblocked;
        int maxDestroy;

};
#endif
