#ifndef XMLLOGGER_H
#define	XMLLOGGER_H

#include "ilogger.h"

class XmlLogger : public ILogger
{

    public:
        XmlLogger(){}
        ~XmlLogger();

        bool getLog(const char* FileName, const std::string* LogParams);
        void saveLog();
        void writeToLogMap(const Map& Map, const NodeList& path);
        void writeToLogOpenClose(const NodeList *open, const std::unordered_map<int, Node> &close, int size, bool last);
        void writeToLogPath(const NodeList& path);
        void writeToLogHPpath(const NodeList& hppath);
        void writeToLogNotFound();
        void writeToLogSummary(unsigned int numberofsteps, unsigned int nodescreated, float length, double time, double cellSize);

    private:
        std::string     LogFileName;
        TiXmlDocument   *doc;
};

#endif

