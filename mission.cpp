#include "mission.h"
#include "astar.h"
#include "bfs.h"
#include "dijkstra.h"
#include "theta.h"
#include "xmllogger.h"
#include "gl_const.h"

Mission::Mission()
{
    logger = nullptr;
    search = nullptr;
    fileName = nullptr;
}

Mission::Mission(const char *FileName)
{
    fileName = FileName;
    logger = nullptr;
    search = nullptr;
}

Mission::~Mission()
{
    if (logger)
        delete logger;
    if (search)
        delete search;
}

bool Mission::getMap()
{
    return map.getMap(fileName);
}

bool Mission::getConfig()
{
    return config.getConfig(fileName);
}

bool Mission::createLog()
{
    if (logger != NULL) delete logger;
    logger = new XmlLogger(config.LogParams[CN_LP_LEVEL]);
    return logger->getLog(fileName, config.LogParams);
}

void Mission::createEnvironmentOptions()
{
    if (config.SearchParams[CN_SP_ST] == CN_SP_ST_BFS || config.SearchParams[CN_SP_ST] == CN_SP_ST_DIJK)
        options = EnvironmentOptions(config.SearchParams[CN_SP_AS], config.SearchParams[CN_SP_AD],
                                     config.SearchParams[CN_SP_CC]);
    else
        options = EnvironmentOptions(config.SearchParams[CN_SP_AS], config.SearchParams[CN_SP_AD],
                                     config.SearchParams[CN_SP_CC], config.SearchParams[CN_SP_MT]);
}

void Mission::createSearch()
{
    if (search)
        delete search;
    if (config.SearchParams[CN_SP_ST] == CN_SP_ST_BFS)
        search = new BFS();
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_DIJK)
        search = new Dijkstra();
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_ASTAR)
        search = new Astar(config.SearchParams[CN_SP_HW], config.SearchParams[CN_SP_BT]);
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_JP_SEARCH)
        search = new JP_Search(config.SearchParams[CN_SP_HW], config.SearchParams[CN_SP_BT]);
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_TH)
        search = new Theta(config.SearchParams[CN_SP_HW], config.SearchParams[CN_SP_BT]);
}

void Mission::startSearch()
{
    sr = search->startSearch(logger, map, options,true);
    std::cout<<"REPLAN!\n";
    sr_replan = search->startSearch(logger, map, options,false);
}

void Mission::printSearchResultsToConsole()
{
    std::cout << "Path ";
    if (!sr.pathfound)
        std::cout << "NOT ";
    std::cout << "found!" << std::endl;
    std::cout << "numberofsteps=" << sr.numberofsteps << std::endl;
    std::cout << "nodescreated=" << sr.nodescreated << std::endl;
    std::cout << "secondlength=" << sr.secondlength << std::endl;
    if (sr.pathfound) {
        std::cout << "pathlength=" << sr.pathlength << std::endl;
        std::cout << "pathlength_scaled=" << sr.pathlength * map.cellSize << std::endl;
    }
    std::cout << "time=" << sr.time << std::endl;
    std::cout<<"\nAfter replanning:\n";
    std::cout << "Path ";
    if (!sr_replan.pathfound)
        std::cout << "NOT ";
    std::cout << "found!" << std::endl;
    std::cout << "numberofsteps=" << sr_replan.numberofsteps << std::endl;
    std::cout << "nodescreated=" << sr_replan.nodescreated << std::endl;
    std::cout << "secondlength=" << sr_replan.secondlength << std::endl;
    if (sr_replan.pathfound) {
        std::cout << "pathlength=" << sr_replan.pathlength << std::endl;
        std::cout << "pathlength_scaled=" << sr_replan.pathlength * map.cellSize << std::endl;
    }
    std::cout << "time=" << sr_replan.time << std::endl;
}

void Mission::saveSearchResultsToLog()
{
    logger->writeToLogSummary(sr, sr_replan);
    if (sr_replan.pathfound) {
       // logger->writeToLogPath(*sr_replan.lppath);
        //logger->writeToLogHPpath(*sr_replan.hppath);
        //logger->writeToLogHPpath(sr_replan.fly_path);
        //logger->writeToLogMap(map, *sr_replan.lppath);
    } else
        logger->writeToLogNotFound();
    logger->saveLog();
}

const char *Mission::getAlgorithmName()
{
    if (config.SearchParams[CN_SP_ST] == CN_SP_ST_ASTAR)
        return CNS_SP_ST_ASTAR;
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_DIJK)
        return CNS_SP_ST_DIJK;
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_BFS)
        return CNS_SP_ST_BFS;
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_JP_SEARCH)
        return CNS_SP_ST_JP_SEARCH;
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_TH)
        return CNS_SP_ST_TH;
    else
        return "";
}
