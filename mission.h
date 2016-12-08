#ifndef MISSION_H
#define	MISSION_H

#include "map.h"
#include "config.h"
#include "isearch.h"
#include "ilogger.h"
#include "searchresult.h"
#include "environmentoptions.h"
#include "jp_search.h"
#include "astar.h"
#include "bfs.h"
#include "dijkstra.h"
#include "theta.h"
#include "xmllogger.h"

class Mission
{
    public:
        Mission();
        Mission (const char* fileName);
        ~Mission();

        bool getMap();
        bool getConfig();
        bool createLog();
        void createSearch();
        void createEnvironmentOptions();
        void startSearch();
        void printSearchResultsToConsole();
        void saveSearchResultsToLog();

    private:
        const char* getAlgorithmName();

        Map                     map;
        Config                  config;
        EnvironmentOptions      options;
        ISearch*                search;
        ILogger*                logger;
        const char*             fileName;
        SearchResult            sr;
};

#endif

