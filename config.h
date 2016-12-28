#ifndef CONFIG_H
#define	CONFIG_H
#include <string>
#include <sstream>
#include "tinyxml.h"
#include "gl_const.h"
#include <iostream>
#include <algorithm>
#include <math.h>

class Config
{
    public:
        Config();
        Config(const Config& orig);
        ~Config();
        bool getConfig(const char *FileName);

    public:
        double*         SearchParams;
        std::string*    LogParams;
        unsigned int    N;

};

#endif

