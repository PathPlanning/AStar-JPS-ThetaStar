#ifndef ILOGGER_H
#define	ILOGGER_H
#include "map.h"
#include "list.h"
#include <unordered_map>

class ILogger
{
    public:
        virtual bool getLog(const char* FileName, const std::string* LogParams) = 0;
        virtual void saveLog() = 0;
        virtual void writeToLogMap(const Map& map, const NodeList& path) = 0;
        virtual void writeToLogOpenClose(const NodeList *open, const std::unordered_map<int,Node>& close, int size, bool last) = 0;
        virtual void writeToLogPath(const NodeList& path) = 0;
        virtual void writeToLogHPpath(const NodeList& path) = 0;
        virtual void writeToLogNotFound() = 0;
        virtual void writeToLogSummary(unsigned int numberofsteps, unsigned int nodescreated, float length, double time, double cellSize) = 0;
    public:
        std::string loglevel;
};

#endif

