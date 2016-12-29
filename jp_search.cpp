#include "jp_search.h"

JP_Search::~JP_Search()
{
}

void JP_Search::findJP(int move_i, int move_j, Node curNode, const Map &map, std::list<Node> &successors, const EnvironmentOptions &options)
{
    bool findOK = false;
    while(!findOK)
    {
        if(map.CellOnGrid(curNode.i+move_i, curNode.j+move_j))
        {
            if(options.cutcorners == false)
            {
                if(move_i != 0 && move_j != 0)
                    if(!map.CellIsTraversable(curNode.i, curNode.j+move_j) || !map.CellIsTraversable(curNode.i+move_i, curNode.j))
                        return;
            }
            else if(options.allowsqueeze == false)
            {
                if(move_i != 0 && move_j != 0)
                    if(!map.CellIsTraversable(curNode.i, curNode.j+move_j) && !map.CellIsTraversable(curNode.i+move_i, curNode.j))
                        return;
            }
            if(map.CellIsTraversable(curNode.i+move_i, curNode.j+move_j))
            {
                curNode.i += move_i;
                curNode.j += move_j;
                curNode.g = curNode.g+MoveCost(curNode.i, curNode.j, curNode.i+move_i, curNode.j+move_j, options);
            }
            else
                return;
        }
        else
            return;
        if(map.goal_i == curNode.i && map.goal_j == curNode.j)
            findOK = true;
        if(options.allowdiagonal == true)//check whether diagonal moves is allowed
        {
            if(options.cutcorners == true)
            {
                if(move_i == 0 && map.CellOnGrid(curNode.i, curNode.j+move_j))
                {
                    if(map.CellOnGrid(curNode.i+1, curNode.j))
                        if(map.CellIsTraversable(curNode.i+1, curNode.j+move_j) && map.CellIsObstacle(curNode.i+1, curNode.j))
                            findOK = true;
                    if(map.CellOnGrid(curNode.i-1,curNode.j))
                        if(map.CellIsTraversable(curNode.i-1, curNode.j+move_j) && map.CellIsObstacle(curNode.i-1, curNode.j))
                            findOK = true;
                }
                if(move_j == 0 && map.CellOnGrid(curNode.i+move_i, curNode.j))
                {
                    if(map.CellOnGrid(curNode.i, curNode.j+1))
                        if(map.CellIsTraversable(curNode.i+move_i, curNode.j+1) && map.CellIsObstacle(curNode.i, curNode.j+1))
                            findOK = true;
                    if(map.CellOnGrid(curNode.i, curNode.j-1))
                        if(map.CellIsTraversable(curNode.i+move_i, curNode.j-1) && map.CellIsObstacle(curNode.i, curNode.j-1))
                            findOK = true;
                }
                if(move_i != 0 && move_j != 0)
                {
                    if(map.CellOnGrid(curNode.i-move_i, curNode.j+move_j))
                        if(map.CellIsObstacle(curNode.i-move_i, curNode.j) && map.CellIsTraversable(curNode.i-move_i, curNode.j+move_j))
                            findOK = true;
                    if(!findOK && map.CellOnGrid(curNode.i+move_i, curNode.j-move_j))
                        if(map.CellIsObstacle(curNode.i, curNode.j-move_j) && map.CellIsTraversable(curNode.i+move_i, curNode.j-move_j))
                            findOK = true;
                    if(!findOK)
                        if(findNeighbors(move_i, 0, curNode, map, options))
                            findOK = true;
                    if(!findOK)
                        if(findNeighbors(0, move_j, curNode, map, options))
                            findOK = true;
                }
            }
            else
            {
                if(move_i == 0 && map.CellOnGrid(curNode.i, curNode.j+move_j))//straight move along j
                {
                    if(map.CellOnGrid(curNode.i+1, curNode.j))
                        if(map.CellIsTraversable(curNode.i+1, curNode.j) && map.CellIsObstacle(curNode.i+1, curNode.j-move_j))//check forced neighbor
                            findOK = true;
                    if(map.CellOnGrid(curNode.i-1,curNode.j))
                        if(map.CellIsTraversable(curNode.i-1, curNode.j) && map.CellIsObstacle(curNode.i-1, curNode.j-move_j))
                            findOK = true;
                }
                if(move_j == 0 && map.CellOnGrid(curNode.i+move_i, curNode.j))//straight move along i
                {
                    if(map.CellOnGrid(curNode.i, curNode.j+1))
                        if(map.CellIsTraversable(curNode.i, curNode.j+1) && map.CellIsObstacle(curNode.i-move_i, curNode.j+1))
                            findOK = true;
                    if(map.CellOnGrid(curNode.i, curNode.j-1))
                        if(map.CellIsTraversable(curNode.i, curNode.j-1) && map.CellIsObstacle(curNode.i-move_i, curNode.j-1))
                            findOK = true;
                }
                if(move_i != 0 && move_j != 0)//diagonal move
                {
                    if(findNeighbors(move_i, 0, curNode, map, options))//looking for forced neighbor along i
                        findOK = true;
                    if(!findOK)
                        if(findNeighbors(0, move_j, curNode, map, options))//looking for forced neighbor along j
                            findOK = true;
                }
            }
        }
        else//only straight moves is allowed
        {
            if(!findOK)
                if(findNeighbors(move_j, move_i, curNode, map, options))
                    findOK = true;
            if(!findOK)
                if(findNeighbors(-move_j, -move_i, curNode, map, options))
                    findOK = true;
        }
    }
    if(findOK && close.find(curNode.i*map.width+curNode.j)==close.end())
        successors.push_front(curNode);
    return;
}

bool JP_Search::findNeighbors(int move_i, int move_j, Node curNode, const Map &map, const EnvironmentOptions &options)
{
    while(map.CellOnGrid(curNode.i, curNode.j) && map.CellIsTraversable(curNode.i, curNode.j))
    {
        if(map.goal_i == curNode.i && map.goal_j == curNode.j)//goal location is found
            return true;
        if(options.cutcorners == true)
        {
            if(move_i == 0 && map.CellOnGrid(curNode.i, curNode.j+move_j))
            {
                if(map.CellOnGrid(curNode.i+1, curNode.j))
                    if(map.CellIsTraversable(curNode.i+1, curNode.j+move_j) && map.CellIsObstacle(curNode.i+1, curNode.j))
                        return true;
                if(map.CellOnGrid(curNode.i-1,curNode.j))
                    if(map.CellIsTraversable(curNode.i-1, curNode.j+move_j) && map.CellIsObstacle(curNode.i-1, curNode.j))
                        return true;
            }
            if(move_j == 0 && map.CellOnGrid(curNode.i+move_i, curNode.j))
            {
                if(map.CellOnGrid(curNode.i, curNode.j+1))
                    if(map.CellIsTraversable(curNode.i+move_i, curNode.j+1) && map.CellIsObstacle(curNode.i, curNode.j+1))
                        return true;
                if(map.CellOnGrid(curNode.i, curNode.j-1))
                    if(map.CellIsTraversable(curNode.i+move_i, curNode.j-1) && map.CellIsObstacle(curNode.i, curNode.j-1))
                        return true;
            }
        }
        else
        {
            if(move_i == 0 && map.CellOnGrid(curNode.i, curNode.j-move_j))
            {
                if(map.CellOnGrid(curNode.i+1, curNode.j))
                    if(map.CellIsTraversable(curNode.i+1, curNode.j) && map.CellIsObstacle(curNode.i+1, curNode.j-move_j))
                        return true;
                if(map.CellOnGrid(curNode.i-1,curNode.j))
                    if(map.CellIsTraversable(curNode.i-1, curNode.j) && map.CellIsObstacle(curNode.i-1, curNode.j-move_j))
                        return true;
            }
            if(move_j == 0 && map.CellOnGrid(curNode.i-move_i, curNode.j))
            {
                if(map.CellOnGrid(curNode.i, curNode.j+1))
                    if(map.CellIsTraversable(curNode.i, curNode.j+1) && map.CellIsObstacle(curNode.i-move_i, curNode.j+1))
                        return true;
                if(map.CellOnGrid(curNode.i, curNode.j-1))
                    if(map.CellIsTraversable(curNode.i, curNode.j-1) && map.CellIsObstacle(curNode.i-move_i, curNode.j-1))
                        return true;
            }
        }
        curNode.i += move_i;
        curNode.j += move_j;
    }
    return false;
}



int JP_Search::findDirection(int current_i, int parent_i)
{
    if(current_i < parent_i)
        return -1;
    else if(current_i > parent_i)
        return 1;
    else
        return 0;
}

std::list<Node> JP_Search::findSuccessors(Node curNode, const Map &map, const EnvironmentOptions &options)
{
    int move_i = 0, move_j = 0;
    std::list<Node> successors;

    if(options.allowdiagonal == true)
    {
        if(curNode.i == map.start_i && curNode.j == map.start_j)//if curNode is the start location, then look for jump points in all directions
            for(int n=-1; n<=1; n++)
                for(int m=-1; m<=1; m++)
                    if(n != 0 || m != 0)
                        findJP(n, m, curNode, map, successors, options);
        if(curNode.i != map.start_i || curNode.j != map.start_j)
        {
            move_i = findDirection(curNode.i, curNode.parent->i);
            move_j = findDirection(curNode.j, curNode.parent->j);
            findJP(move_i, move_j, curNode, map, successors, options);//continue to look for jump points in the same direction

            if(move_i != 0 && move_j != 0)//if curNoode is a diagonal jump point
            {
                if(map.CellIsObstacle(curNode.i-move_i, curNode.j))
                    findJP(-move_i, move_j, curNode, map, successors, options);
                if(map.CellIsObstacle(curNode.i, curNode.j-move_j))
                    findJP(move_i, -move_j, curNode, map, successors, options);
                findJP(move_i, 0, curNode, map, successors, options);//look for jump point in straight direction along i
                findJP(0, move_j, curNode, map, successors, options);//the same check along j
            }

            if(options.cutcorners==true)//original JPS, when cutcorners is allowed
            {
                if(move_i == 0)
                {
                    if(map.CellOnGrid(curNode.i-move_j, curNode.j))
                        if(map.CellIsObstacle(curNode.i-move_j, curNode.j))
                            findJP(-move_j, move_j, curNode, map, successors, options);
                    if(map.CellOnGrid(curNode.i+move_j, curNode.j))
                        if(map.CellIsObstacle(curNode.i+move_j, curNode.j))
                            findJP(move_j, move_j, curNode, map, successors, options);
                }
                else if(move_j==0)
                {
                    if(map.CellOnGrid(curNode.i, curNode.j-move_i))
                        if(map.CellIsObstacle(curNode.i, curNode.j-move_i))
                            findJP(move_i, -move_i, curNode, map, successors, options);
                    if(map.CellOnGrid(curNode.i, curNode.j+move_i))
                        if(map.CellIsObstacle(curNode.i, curNode.j+move_i))
                            findJP(move_i, move_i, curNode, map, successors, options);
                }
            }
            else//cutcorners disallowed
            {
                if(move_i == 0)
                {
                    if(map.CellOnGrid(curNode.i-move_j, curNode.j))
                        if(map.CellIsObstacle(curNode.i-move_j, curNode.j-move_j))
                        {
                            findJP(-move_j, move_j, curNode, map, successors, options);
                            findJP(-move_j, 0, curNode, map, successors, options);
                        }
                    if(map.CellOnGrid(curNode.i+move_j, curNode.j))
                        if(map.CellIsObstacle(curNode.i+move_j, curNode.j-move_j))
                        {
                            findJP(move_j, 0, curNode, map, successors, options);
                            findJP(move_j, move_j, curNode, map, successors, options);
                        }
                }
                else if(move_j==0)
                {
                    if(map.CellOnGrid(curNode.i, curNode.j-move_i))
                        if(map.CellIsObstacle(curNode.i-move_i, curNode.j-move_i))
                        {
                            findJP(0, -move_i, curNode, map, successors, options);//additional check
                            findJP(move_i, -move_i, curNode, map, successors, options);
                        }
                    if(map.CellOnGrid(curNode.i, curNode.j+move_i))
                        if(map.CellIsObstacle(curNode.i-move_i, curNode.j+move_i))
                        {
                            findJP(0, move_i, curNode, map, successors, options);//additional check
                            findJP(move_i, move_i, curNode, map, successors, options);
                        }
                }
            }
        }
    }
    else //only straight moves as allowed
    {
        if(curNode.i == map.start_i && curNode.j == map.start_j)
            for(int n=-1; n<=1; n++)
                for(int m=-1; m<=1; m++)
                    if((n != 0 && m == 0) || (n == 0 && m != 0))
                        findJP(n, m, curNode, map, successors, options);
        if(curNode.i != map.start_i || curNode.j != map.start_j)
        {
            move_i=findDirection(curNode.i, curNode.parent->i);
            move_j=findDirection(curNode.j, curNode.parent->j);

            findJP(move_i, move_j, curNode, map, successors, options);
            findJP(move_j, move_i, curNode, map, successors, options);
            findJP(-move_j, -move_i, curNode, map, successors, options);
        }
    }
    return successors;
}

void JP_Search::makePrimaryPath(Node curNode)
{
    Node current=curNode;
    while(current.parent)
    {
        hppath.List.push_front(current);
        current=*current.parent;
    }
    hppath.List.push_front(current);
    sresult.hppath = &hppath;
}

void JP_Search::makeSecondaryPath(const Map &map, Node curNode)
{
    Node *forpath=&*hppath.List.begin();
    std::list<Node>::iterator iter = hppath.List.begin();
    iter++;
    Node *forpath2=&*iter;
    int x0,x1,y0,y1,s_x,s_y,dx,dy,f;
    Node inpath;

    lppath.List.push_back(*forpath);
    while(forpath2!=&*hppath.List.end())
    {
        x0=forpath->i;
        y0=forpath->j;
        x1=forpath2->i;
        y1=forpath2->j;
        dx= x1-x0;
        dy= y1-y0;
        f=0;

        if (dy<0) {dy=-dy; s_y=-1;}
        else s_y=1;

        if (dx<0) {dx=-dx; s_x=-1;}
        else s_x=1;

        if (dx>=dy)
        {
            while (x0!=x1)
            {
                f+=dy;
                if (f>=dx)
                {
                    y0+=s_y;
                    f-=dx;
                }
                x0+=s_x;
                inpath.i=x0;
                inpath.j=y0;
                lppath.List.push_back(inpath);
            }
        }
        else
            while (y0!=y1)
            {
                f+=dx;
                if (f>=dy)
                {
                    x0+=s_x;
                    f-=dy;
                }

                y0+=s_y;
                inpath.i=x0;
                inpath.j=y0;
                lppath.List.push_back(inpath);
            }
        forpath=forpath2;
        iter++;
        forpath2=&*iter;
    }
    sresult.lppath = &lppath;
}

/*Node JP_Search::resetParent(Node current, Node parent, const Map &map, const EnvironmentOptions &options )
{
    if(options.useresetparent==false)
        return current;
    if ((parent.i == map.start_i && parent.j == map.start_j) || (current.i == map.start_i && current.j == map.start_j))
        return current;
    if (lineOfSight(parent.parent->i, parent.parent->j, current.i, current.j, map))
    {
        current.g = parent.parent->g + computeHFromCellToCell(parent.parent->i, parent.parent->j, current.i, current.j, options);
        current.parent = parent.parent;
    }
    return current;
}*/
