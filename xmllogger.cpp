#include "xmllogger.h"

using tinyxml2::XMLElement;
using tinyxml2::XMLNode;

bool XmlLogger::getLog(const char *FileName, const std::string *LogParams)
{
    if (loglevel == CN_LP_LEVEL_NOPE_WORD) return true;

    if (doc.LoadFile(FileName) != tinyxml2::XMLError::XML_SUCCESS) {
        std::cout << "Error opening input XML file" << std::endl;
        return false;
    }

    if (LogParams[CN_LP_PATH] == "" && LogParams[CN_LP_NAME] == "") {
        std::string str;
        str.append(FileName);
        size_t found = str.find_last_of(".");
        if (found != std::string::npos)
            str.insert(found, "_log");
        else
            str.append("_log");
        LogFileName.append(str);
    } else if (LogParams[CN_LP_PATH] == "") {
        LogFileName.append(FileName);
        std::string::iterator it = LogFileName.end();
        while (*it != '\\')
            it--;
        ++it;
        LogFileName.erase(it, LogFileName.end());
        LogFileName.append(LogParams[CN_LP_NAME]);
    } else if (LogParams[CN_LP_NAME] == "") {
        LogFileName.append(LogParams[CN_LP_PATH]);
        if (*(--LogParams[CN_LP_PATH].end()) != '\\') LogFileName.append("\\");
        std::string lfn;
        lfn.append(FileName);
        size_t found = lfn.find_last_of("\\");
        std::string str = lfn.substr(found);
        found = str.find_last_of(".");
        if (found != std::string::npos)
            str.insert(found, "_log");
        else
            str.append("_log");
        LogFileName.append(str);
    } else {
        LogFileName.append(LogParams[CN_LP_PATH]);
        if (*(--LogParams[CN_LP_PATH].end()) != '\\') LogFileName.append("\\");
        LogFileName.append(LogParams[CN_LP_NAME]);
    }

    XMLElement *log, *root = doc.FirstChildElement(CNS_TAG_ROOT);

    if (!root) {
        std::cout << "No '" << CNS_TAG_ROOT << "' element found in XML file" << std::endl;
        std::cout << "Can not create log" << std::endl;
        return false;
    }

    root->InsertEndChild(doc.NewElement(CNS_TAG_LOG));

    root = (root->LastChild())->ToElement();

    if (loglevel != CN_LP_LEVEL_NOPE_WORD) {
        log = doc.NewElement(CNS_TAG_MAPFN);
        log->LinkEndChild(doc.NewText(FileName));
        root->InsertEndChild(log);

        root->InsertEndChild(doc.NewElement(CNS_TAG_SUM));

        root->InsertEndChild(doc.NewElement(CNS_TAG_PATH));

        root->InsertEndChild(doc.NewElement(CNS_TAG_LPLEVEL));

        root->InsertEndChild(doc.NewElement(CNS_TAG_HPLEVEL));
    }

    if (loglevel == CN_LP_LEVEL_FULL_WORD || loglevel == CN_LP_LEVEL_MEDIUM_WORD)
        root->InsertEndChild(doc.NewElement(CNS_TAG_LOWLEVEL));

    return true;
}

void XmlLogger::saveLog()
{
    if (loglevel == CN_LP_LEVEL_NOPE_WORD)
        return;
    doc.SaveFile(LogFileName.c_str());
}

void XmlLogger::writeToLogMap(const Map &map, const std::list<Node> &path)
{
    if (loglevel == CN_LP_LEVEL_NOPE_WORD || loglevel == CN_LP_LEVEL_TINY_WORD)
        return;

    XMLElement *mapTag = doc.FirstChildElement(CNS_TAG_ROOT);
    mapTag = mapTag->FirstChildElement(CNS_TAG_LOG)->FirstChildElement(CNS_TAG_PATH);

    int iterate = 0;
    bool inPath;
    std::string str;
    for (int i = 0; i < map.height; ++i) {
        XMLElement *element = doc.NewElement(CNS_TAG_ROW);
        element->SetAttribute(CNS_TAG_ATTR_NUM, iterate);

        for (int j = 0; j < map.width; ++j) {
            inPath = false;
            for(std::list<Node>::const_iterator it = path.begin(); it != path.end(); it++)
                if(it->i == i && it->j == j) {
                    inPath = true;
                    break;
                }
            if (!inPath)
                str += std::to_string(map.getValue(i,j));
            else
                str += CNS_OTHER_PATHSELECTION;
            str += CNS_OTHER_MATRIXSEPARATOR;
        }

        element->InsertEndChild(doc.NewText(str.c_str()));
        mapTag->InsertEndChild(element);
        str.clear();
        iterate++;
    }
}

void XmlLogger::writeToLogOpenClose(const std::vector<std::list<Node> > &open, const std::unordered_map<int, Node> &close, bool last)
{
    if (loglevel != CN_LP_LEVEL_FULL_WORD  && !(loglevel == CN_LP_LEVEL_MEDIUM_WORD && last))
        return;

    XMLElement *element  = doc.NewElement(CNS_TAG_STEP);
    XMLElement *child    = 0;
    XMLElement *lowlevel = doc.FirstChildElement(CNS_TAG_ROOT);
    lowlevel = lowlevel->FirstChildElement(CNS_TAG_LOG)->FirstChildElement(CNS_TAG_LOWLEVEL);

    int iterate = 0;
    for (child = lowlevel->FirstChildElement(); child != nullptr; child = child->NextSiblingElement())
        iterate++;

    element->SetAttribute(CNS_TAG_ATTR_NUM, iterate);
    lowlevel->InsertEndChild(element);
    lowlevel = lowlevel->LastChildElement();

    lowlevel->InsertEndChild(doc.NewElement(CNS_TAG_OPEN));
    child = lowlevel->LastChildElement();

    Node min;
    min.F = -1;
    int exc = 0;
    for (int i = 0; i < open.size(); ++i) {
        if (open[i].size() > 0) {
            if (open[i].begin()->F <= min.F || min.F == -1) {
                if (open[i].begin()->F == min.F && open[i].begin()->g > min.g) {
                    min = *open[i].begin();
                    exc = i;
                } else if (open[i].begin()->F < min.F || min.F == -1) {
                    min = *open[i].begin();
                    exc = i;
                }
            }
        }
    }
    if (min.F != -1) {
        element = doc.NewElement(CNS_TAG_POINT);
        element->SetAttribute(CNS_TAG_ATTR_X, min.j);
        element->SetAttribute(CNS_TAG_ATTR_Y, min.i);
        element->SetAttribute(CNS_TAG_ATTR_F, min.F);
        element->SetAttribute(CNS_TAG_ATTR_G, min.g);
        if (min.g > 0) {
            element->SetAttribute(CNS_TAG_ATTR_PARX, min.parent->j);
            element->SetAttribute(CNS_TAG_ATTR_PARY, min.parent->i);
        }
        child->InsertEndChild(element);
    }
    for (int i = 0; i < open.size(); ++i) {
        if (open[i].size() > 0) {
            for (auto it = open[i].begin(); it != open[i].end(); ++it) {
                if (it != open[exc].begin()) {
                    element = doc.NewElement(CNS_TAG_POINT);
                    element->SetAttribute(CNS_TAG_ATTR_X, it->j);
                    element->SetAttribute(CNS_TAG_ATTR_Y, it->i);
                    element->SetAttribute(CNS_TAG_ATTR_F, it->F);
                    element->SetAttribute(CNS_TAG_ATTR_G, it->g);
                    if (it->g > 0) {
                        element->SetAttribute(CNS_TAG_ATTR_PARX, it->parent->j);
                        element->SetAttribute(CNS_TAG_ATTR_PARY, it->parent->i);
                    }
                    child->InsertEndChild(element);
                }
                child->InsertEndChild(element);
            }
        }
    }

    lowlevel->InsertEndChild(doc.NewElement(CNS_TAG_CLOSE));
    child = lowlevel->LastChildElement();

    for (auto it = close.begin(); it != close.end(); ++it) {
        element = doc.NewElement(CNS_TAG_POINT);
        element->SetAttribute(CNS_TAG_ATTR_X, it->second.j);
        element->SetAttribute(CNS_TAG_ATTR_Y, it->second.i);
        element->SetAttribute(CNS_TAG_ATTR_F, it->second.F);
        element->SetAttribute(CNS_TAG_ATTR_G, it->second.g);
        if (it->second.g > 0) {
            element->SetAttribute(CNS_TAG_ATTR_PARX, it->second.parent->j);
            element->SetAttribute(CNS_TAG_ATTR_PARY, it->second.parent->i);
        }
        child->InsertEndChild(element);
    }
}

void XmlLogger::writeToLogPath(const std::list<Node> &path)
{
    if (loglevel == CN_LP_LEVEL_NOPE_WORD || loglevel == CN_LP_LEVEL_TINY_WORD || path.empty())
        return;
    int iterate = 0;
    XMLElement *lplevel = doc.FirstChildElement(CNS_TAG_ROOT);
    lplevel = lplevel->FirstChildElement(CNS_TAG_LOG)->FirstChildElement(CNS_TAG_LPLEVEL);

    for (std::list<Node>::const_iterator it = path.begin(); it != path.end(); it++) {
        XMLElement *element = doc.NewElement(CNS_TAG_POINT);
        element->SetAttribute(CNS_TAG_ATTR_X, it->j);
        element->SetAttribute(CNS_TAG_ATTR_Y, it->i);
        element->SetAttribute(CNS_TAG_ATTR_NUM, iterate);
        lplevel->InsertEndChild(element);
        iterate++;
    }
}

void XmlLogger::writeToLogHPpath(const std::list<Node> &hppath)
{
    if (loglevel == CN_LP_LEVEL_NOPE_WORD || loglevel == CN_LP_LEVEL_TINY_WORD || hppath.empty())
        return;
    int partnumber = 0;
    XMLElement *hplevel = doc.FirstChildElement(CNS_TAG_ROOT);
    hplevel = hplevel->FirstChildElement(CNS_TAG_LOG)->FirstChildElement(CNS_TAG_HPLEVEL);
    std::list<Node>::const_iterator iter = hppath.begin();
    std::list<Node>::const_iterator it = hppath.begin();

    while (iter != --hppath.end()) {
        XMLElement *part = doc.NewElement(CNS_TAG_SECTION);
        part->SetAttribute(CNS_TAG_ATTR_NUM, partnumber);
        part->SetAttribute(CNS_TAG_ATTR_STX, it->j);
        part->SetAttribute(CNS_TAG_ATTR_STY, it->i);
        ++iter;
        part->SetAttribute(CNS_TAG_ATTR_FINX, iter->j);
        part->SetAttribute(CNS_TAG_ATTR_FINY, iter->i);
        part->SetAttribute(CNS_TAG_ATTR_LENGTH, iter->g - it->g);
        hplevel->LinkEndChild(part);
        ++it;
        ++partnumber;
    }
}

void XmlLogger::writeToLogSummary(unsigned int numberofsteps, unsigned int nodescreated, float length, double time, double cellSize)
{
    if (loglevel == CN_LP_LEVEL_NOPE_WORD)
        return;

    XMLElement *summary = doc.FirstChildElement(CNS_TAG_ROOT);
    summary = summary->FirstChildElement(CNS_TAG_LOG)->FirstChildElement(CNS_TAG_SUM);
    XMLElement *element = summary->ToElement();
    element->SetAttribute(CNS_TAG_ATTR_NUMOFSTEPS, numberofsteps);
    element->SetAttribute(CNS_TAG_ATTR_NODESCREATED, nodescreated);
    element->SetAttribute(CNS_TAG_ATTR_LENGTH, length);
    element->SetAttribute(CNS_TAG_ATTR_LENGTH_SCALED, length*cellSize);
    element->SetAttribute(CNS_TAG_ATTR_TIME, std::to_string(time).c_str());
}

void XmlLogger::writeToLogNotFound()
{
    if (loglevel == CN_LP_LEVEL_NOPE_WORD)
        return;

    XMLElement *node = doc.FirstChildElement(CNS_TAG_ROOT)->FirstChildElement(CNS_TAG_LOG)->FirstChildElement(CNS_TAG_PATH);
    node->InsertEndChild(doc.NewText("Path NOT found!"));
}
