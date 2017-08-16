#include "isearch.h"
#include <vector>
#include <math.h>
#include <limits>
#include <chrono>

ISearch::ISearch()
{
    hweight = 1;
    breakingties = CN_SP_BT_GMAX;
    openSize = 0;
}

ISearch::~ISearch(void) {}

bool ISearch::stopCriterion()
{
    if (openSize == 0) {
        std::cout << "OPEN list is empty!" << std::endl;
        return true;
    }
    return false;
}

SearchResult ISearch::startSearch(ILogger *Logger, Map &map, const EnvironmentOptions &options, bool replan)
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    numberOfBlocks.resize(map.numberOfObs+1, 0);
    blockedCells.resize(map.numberOfObs+1);
    close.clear();
    open.resize(map.height);
    Node curNode;
    curNode.i = map.start_i;
    curNode.j = map.start_j;
    curNode.g = 0;
    curNode.H = computeHFromCellToCell(curNode.i, curNode.j, map.goal_i, map.goal_j, options);
    curNode.F = hweight * curNode.H;
    curNode.parent = nullptr;
    addOpen(curNode);
    int closeSize = 0;
    bool pathfound = false;
    while (!stopCriterion()) {
        curNode = findMin(map.width);
        open[curNode.i].pop_front();
        close.insert({curNode.i * map.width + curNode.j, curNode});
        closeSize++;
        openSize--;
        if (curNode.i == map.goal_i && curNode.j == map.goal_j) {
            pathfound = true;
            break;
        }
        std::list<Node> successors = findSuccessors(curNode, map, options);
        std::list<Node>::iterator it = successors.begin();
        auto parent = &(close.find(curNode.i * map.width + curNode.j)->second);
        while (it != successors.end()) {
            it->parent = parent;
            it->H = computeHFromCellToCell(it->i, it->j, map.goal_i, map.goal_j, options);
            *it = resetParent(*it, *it->parent, map, options);
            it->F = it->g + hweight * it->H;
            addOpen(*it);
            it++;
        }
        Logger->writeToLogOpenClose(open, close, false);
    }
    if(replan)
        Logger->writeToLogOpenClose(open, close, true);

    sresult.pathfound = false;
    sresult.nodescreated = closeSize + openSize;
    sresult.numberofsteps = closeSize;
    if (pathfound) {
        sresult.pathfound = true;
        makePrimaryPath(curNode);
        sresult.pathlength = curNode.g;
    }
    end = std::chrono::system_clock::now();
    if(replan){
        for(int k = 0; k < std::min(CN_DESTROY_NUM, map.numberOfObs); k++){
            int destroyNum = chooseObstacleToDestroy(map);
            findBlockObstacle(map, blockedCells[destroyNum], destroyNum);
            allObs.push_back(obs);
            numberOfBlocks[destroyNum] = 0;
            map.destroyObstacle(destroyNum);
        }
        sresult.secondlength = sqrt(pow(map.start_i - map.goal_i, 2) + pow(map.start_j - map.goal_j, 2));
    }
    else
        findFlyPath(map);
    sresult.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()) / 1000000000;
    if (pathfound)
        makeSecondaryPath();
    sresult.hppath = &hppath; //Here is a constant pointer
    sresult.lppath = &lppath;
    return sresult;
}

void ISearch::findFlyPath(const Map &map)
{
    double mindist = std::numeric_limits<double>::infinity();
    double dist;
    int curMin;
    Node curNode;
    std::pair<int,int> curCell;
    std::vector<std::pair<std::pair<int,int>, double>> airPath(0);
    std::list<Node> Path;
    Path.push_back(Node(map.start_i, map.start_j,0,0));
    for(int k = 0; k < allObs.size(); k++){
        mindist = std::numeric_limits<double>::infinity();
        for(auto it = allObs[k].begin(); it!=allObs[k].end(); it++)
        {
            if((it->second.first - map.start_i)*(it->second.first - map.goal_i) > 0 && (it->second.second - map.goal_j)*(it->second.second - map.start_j) > 0)
                continue;
            dist = fabs((map.start_i - map.goal_i)*it->second.second + (map.goal_j - map.start_j)*it->second.first + (map.start_j*map.goal_i - map.start_i*map.goal_j))
                    /sqrt(pow(map.start_i-map.goal_i,2)+pow(map.start_j-map.goal_j,2));
            if(dist < mindist){
                curMin = k;
                curCell = {it->second.first, it->second.second};
                mindist = dist;
            }
        }
        airPath.push_back({curCell, sqrt(pow(map.start_i - curCell.first, 2) + pow(map.start_j - curCell.second, 2))});
    }

    airPath.push_back({{map.goal_i, map.goal_j}, sqrt(pow(map.start_i - map.goal_i, 2) + pow(map.start_j - map.goal_j, 2))});
    while(!airPath.empty()){
        mindist = std::numeric_limits<double>::infinity();
        for(int k = 0; k < airPath.size(); k++)
            if(airPath[k].second < mindist){
                curMin = k;
                mindist = airPath[k].second;
            }
        curNode = Node(airPath[curMin].first.first, airPath[curMin].first.second);
        curNode.g = Path.begin()->g + sqrt(pow(Path.begin()->i - curNode.i,2)+pow(Path.begin()->j - curNode.j, 2));
        Path.push_front(curNode);
        airPath.erase(airPath.begin() + curMin);
    }
    sresult.fly_path = Path;
    sresult.secondlength = curNode.g;
}

Node ISearch::findMin(int size)
{
    Node min;
    min.F=std::numeric_limits<double>::infinity();
    for(int i=0; i<size; i++){
        if(!open[i].empty())
            if(open[i].begin()->F<=min.F){
                if (open[i].begin()->F == min.F){
                    switch(breakingties){
                        case CN_SP_BT_GMAX:{
                            if (open[i].begin()->g >= min.g)
                                min=*open[i].begin();
                            break;
                        }
                        case CN_SP_BT_GMIN:{
                            if (open[i].begin()->g <= min.g)
                                min=*open[i].begin();
                            break;
                        }
                    }
                }
                else
                    min=*open[i].begin();
            }
    }
    return min;

}

void ISearch::addOpen(Node newNode)
{
    std::list<Node>::iterator iter,pos;

    bool posFound=false;

    pos = open[newNode.i].end();

    if (open[newNode.i].size() == 0)
    {
        open[newNode.i].push_back(newNode);
        openSize++;
        return;
    }

    for(iter=open[newNode.i].begin(); iter != open[newNode.i].end(); ++iter)
    {
        if ((iter->F >= newNode.F) && (!posFound))
        {
            if (iter->F == newNode.F)
            {
                switch(breakingties)
                {
                    case CN_SP_BT_GMAX:
                    {
                        if (newNode.g >= iter->g)
                        {
                            pos=iter;
                            posFound=true;
                        }
                        break;
                    }
                    case CN_SP_BT_GMIN:
                    {
                        if (newNode.g <= iter->g)
                        {
                            pos=iter;
                            posFound=true;
                        }
                        break;
                    }
                }
            }
            else
            {
                pos=iter;
                posFound=true;
            }
        }

        if (((iter->i) == newNode.i) && (iter->j)==newNode.j)
        {
            if (newNode.F >= iter->F)
            {
                return;
            }
            else
            {
                if(pos == iter)
                {
                    iter->F = newNode.F;
                    iter->g = newNode.g;
                    iter->parent = newNode.parent;
                    return;
                }
                open[newNode.i].erase(iter);
                openSize--;
                break;
            }
        }
    }
    openSize++;
    open[newNode.i].insert(pos,newNode);
}


std::list<Node> ISearch::findSuccessors(Node curNode, const Map &map, const EnvironmentOptions &options)
{
    Node newNode;
    std::list<Node> successors;
    for (int i = -1; i <= +1; i++)
        for (int j = -1; j <= +1; j++)
            if ((i != 0 || j != 0) && map.CellOnGrid(curNode.i + i, curNode.j + j)){
                if(map.getValue(curNode.i + i, curNode.j + j) > 0){
                    numberOfBlocks[map.getValue(curNode.i + i, curNode.j + j)]++;
                    if(i == 0 || j == 0)
                        blockedCells[map.getValue(curNode.i + i, curNode.j + j)] = {curNode.i, curNode.j};
                    continue;
                }
                if (i != 0 && j != 0) {
                    if (!options.allowdiagonal)
                        continue;
                    else if (!options.cutcorners) {
                        if (map.CellIsObstacle(curNode.i, curNode.j + j) ||
                                map.CellIsObstacle(curNode.i + i, curNode.j))
                            continue;
                    }
                    else if (!options.allowsqueeze) {
                        if (map.CellIsObstacle(curNode.i, curNode.j + j) &&
                                map.CellIsObstacle(curNode.i + i, curNode.j))
                            continue;
                    }
                }
                if (close.find((curNode.i + i) * map.width + curNode.j + j) == close.end()) {
                    newNode.i = curNode.i + i;
                    newNode.j = curNode.j + j;
                    if(i == 0 || j == 0)
                        newNode.g = curNode.g + 1;
                    else
                        newNode.g = curNode.g + sqrt(2);
                    successors.push_front(newNode);
                }
            }
    return successors;
}

void ISearch::makePrimaryPath(Node curNode)
{
    lppath.clear();
    Node current = curNode;
    while (current.parent) {
        lppath.push_front(current);
        current = *current.parent;
    }
    lppath.push_front(current);
}

void ISearch::makeSecondaryPath()
{
    //hppath.clear();
    std::list<Node>::const_iterator iter = lppath.begin();
    int curI, curJ, nextI, nextJ, moveI, moveJ;
    hppath.push_back(*iter);
    while (iter != --lppath.end()) {
        curI = iter->i;
        curJ = iter->j;
        ++iter;
        nextI = iter->i;
        nextJ = iter->j;
        moveI = nextI - curI;
        moveJ = nextJ - curJ;
        ++iter;
        if ((iter->i - nextI) != moveI || (iter->j - nextJ) != moveJ)
            hppath.push_back(*(--iter));
        else
            --iter;
    }
}

void ISearch::findBlockObstacle(const Map &map, std::pair<int, int> boom, int numOfObs)
{
    Node min(boom.first,boom.second);
    obs.clear();
    int rotation;
    //std::cout<<min.i<<" "<<min.j<<"\n";
    if(map.getValue(min.i + 1, min.j) == numOfObs)
        rotation = 3;//down
    else if(map.getValue(min.i - 1, min.j) == numOfObs)
        rotation = 1;//up
    else if(map.getValue(min.i, min.j + 1) == numOfObs)
        rotation = 2;//right
    else if(map.getValue(min.i, min.j - 1) == numOfObs)
        rotation = 4;//left
    else
    {
        std::cout<<"Found cell is wrong!\n";
        return;
    }
    //std::cout<<min.i<<" "<<min.j<<" "<<rotation<<"\n";
    int previ=min.i,prevj=min.j,prevr=rotation;
    int firstrot=rotation;
    int curi=min.i;
    int curj=min.j;
    int minrot = rotation;

    //obs.insert({curi*map.width+curj,std::make_pair(computeHFromCellToCell(map.start_i,map.start_j,curi,curj,options),tang)});
    obs.insert({curi*map.width+curj,std::make_pair(min.i,min.j)});
    int addi=0, addj=0;
    int side=1;
    int sumr=0;
    int k=0;
    do
    {
        //std::cout<<curi<<" "<<curj<<" "<<rotation<<" "<<side<<"\n";
        addi=0;
        addj=0;
        if(rotation==1)
            addi=-side;
        else if(rotation==2)
            addj=side;
        else if(rotation==3)
            addi=side;
        else
            addj=-side;
        previ=curi;
        prevj=curj;
        if(map.getValue(curi+addi,curj+addj)!=numOfObs)
        {
            rotation-=side;
        }
        if(map.getValue(curi+addi,curj+addj)==numOfObs || map.getValue(curi+addi,curj+addj) == -1)
        {
            if(map.getValue(curi+addj*side,curj-addi*side)==numOfObs)
            {
                rotation+=side;
            }
            if(map.getValue(curi+addj*side,curj-addi*side)==-1)//have found end of the grid
            {
                if(side==-1)
                    break;
                curi=min.i;
                curj=min.j;
                previ=curi;
                prevj=curj;
                rotation=firstrot+2;
                if(rotation>4)
                    rotation-=4;
                prevr=rotation;
                sumr=0;
                side=-1;
                continue;
            }
        }
        if(rotation==0)
            rotation=4;
        else if(rotation==5)
            rotation=1;
        //std::cout<<map.getValue(curi+addi,curj+addj)<<" "<<map.getValue(curi,curj-1)<<" "<<curi<<" "<<curj<<"\n";

        if(rotation==1 && map.getValue(curi,curj+1)!=numOfObs && map.getValue(curi,curj+1)!=-1 && ((prevr>=rotation && side==1) || ((prevr<=rotation || (prevr==4 && rotation==1)) && side==-1)))
            curj++;
        else if(rotation==2 && map.getValue(curi+1,curj)!=numOfObs && map.getValue(curi+1,curj)!=-1 && ((prevr>=rotation && side==1) || (prevr<=rotation && side==-1)))
            curi++;
        else if(rotation==3 && map.getValue(curi,curj-1)!=numOfObs && map.getValue(curi,curj-1)!=-1 && ((prevr>=rotation && side==1) || (prevr<=rotation && side==-1)))
            curj--;
        else if(map.getValue(curi-1,curj)!=numOfObs && map.getValue(curi-1,curj)!=-1 && (((prevr>=rotation || (prevr==1 && rotation==4)) && side==1) || (prevr<=rotation && side==-1)))
            curi--;
        if(previ!=curi || prevj!=curj)
        {
            k++;
            //if(obs.find(curi*map.width+curj)==obs.end())
            //    std::cout<<curi<<" "<<curj<<"\n";
            //obs.insert({curi*map.width+curj,std::make_pair(computeHFromCellToCell(map.start_i,map.start_j,curi,curj,options),tang)});
            obs.insert({curi*map.width+curj,std::make_pair(curi,curj)});
        }
        if((rotation-prevr)*side>0 || (rotation-prevr)*side==3)
            sumr--;
        else if(rotation*side>prevr)
            sumr++;
        if(k/obs.size()>2)
            break;
        prevr=rotation;
    }
    while(curi!=min.i || curj!=min.j ||  minrot!=rotation);
}

int ISearch::chooseObstacleToDestroy(const Map &map)
{
    std::pair<int, int> maxBlocked({-1,-1});
    for(int i=0; i<map.numberOfObs+1; i++)
    {
        //if(numberOfBlocks[i]>0)
        //    std::cout<<i<<" "<<numberOfBlocks[i]<<" "<<blockedCells[i].first<<" "<<blockedCells[i].second<<"\n";
        if(maxBlocked.second < numberOfBlocks[i])
            maxBlocked = {i,numberOfBlocks[i]};
    }
    return maxBlocked.first;
}
