#include "theta.h"
Theta::~Theta()
{
}

bool Theta::lineOfSight(int i1, int j1, int i2, int j2, const Map &map, bool cutcorners)
{
    int delta_i = std::abs(i1 - i2);
    int delta_j = std::abs(j1 - j2);
    int step_i = (i1 < i2 ? 1 : -1);
    int step_j = (j1 < j2 ? 1 : -1);
    int error = 0;
    int i = i1;
    int j = j1;
    if(delta_i == 0)
    {
        for(; j != j2; j += step_j)
            if(map.CellIsObstacle(i, j))
                return false;
        return true;
    }
    else if(delta_j == 0)
    {
        for(; i != i2; i += step_i)
            if(map.CellIsObstacle(i, j))
                return false;
        return true;
    }
    if(cutcorners)
    {
        if (delta_i > delta_j)
        {
            for (; i != i2; i += step_i)
            {
                if (map.CellIsObstacle(i, j))
                    return false;
                error += delta_j;
                if ((error << 1) > delta_i)
                {
                    if(((error << 1) - delta_j) < delta_i)
                    {
                        if (map.CellIsObstacle(i+step_i, j))
                            return false;
                    }
                    else if(((error << 1) - delta_j) > delta_i)
                    {
                        if (map.CellIsObstacle(i, j+step_j))
                            return false;
                    }
                    j += step_j;
                    error -= delta_i;
                }
            }
        }
        else
        {
            for (; j != j2; j += step_j)
            {
                if (map.CellIsObstacle(i, j))
                    return false;
                error += delta_i;
                if ((error << 1) > delta_j)
                {
                    if(((error << 1) - delta_i) < delta_j)
                    {
                        if (map.CellIsObstacle(i, j+step_j))
                            return false;
                    }
                    else if(((error << 1) - delta_i) > delta_j)
                    {
                        if (map.CellIsObstacle(i+step_i, j))
                            return false;
                    }
                    i += step_i;
                    error -= delta_j;
                }
            }
        }

    }
    else
    {
        int sep_value = delta_i*delta_i + delta_j*delta_j;
        if(delta_i > delta_j)
        {
            for(; i != i2; i += step_i)
            {
                if(map.CellIsObstacle(i, j))
                    return false;
                if(map.CellIsObstacle(i, j+step_j))
                    return false;
                error += delta_j;
                if(error >= delta_i)
                {
                    if(((error << 1) - delta_i - delta_j)*((error << 1) - delta_i - delta_j) < sep_value)
                        if(map.CellIsObstacle(i + step_i,j))
                            return false;
                    if((3*delta_i - ((error << 1) - delta_j))*(3*delta_i - ((error << 1) - delta_j)) < sep_value)
                        if(map.CellIsObstacle(i,j + 2*step_j))
                            return false;
                    j += step_j;
                    error -= delta_i;
                }
            }
            if(map.CellIsObstacle(i, j))
                return false;
        }
        else
        {
            for(; j != j2; j += step_j)
            {
                if(map.CellIsObstacle(i, j))
                    return false;
                if(map.CellIsObstacle(i + step_i, j))
                    return false;
                error += delta_i;
                if(error >= delta_j)
                {
                    if(((error << 1) - delta_i - delta_j)*((error << 1) - delta_i - delta_j) < (delta_i*delta_i + delta_j*delta_j))
                        if(map.CellIsObstacle(i, j + step_j))
                            return false;
                    if((3*delta_j - ((error << 1) - delta_i))*(3*delta_j - ((error << 1) - delta_i)) < (delta_i*delta_i + delta_j*delta_j))
                        if(map.CellIsObstacle(i + 2*step_i, j))
                            return false;
                    i += step_i;
                    error -= delta_j;
                }
            }
            if(map.CellIsObstacle(i, j))
                return false;
        }
    }
    return true;
}

Node Theta::resetParent(Node current, Node parent, const Map &map, const EnvironmentOptions &options )
{
    if ((parent.i == map.start_i && parent.j == map.start_j) || (current.i == map.start_i && current.j == map.start_j))
        return current;
    if (lineOfSight(parent.parent->i, parent.parent->j, current.i, current.j, map, options.cutcorners))
    {
        current.g = parent.parent->g + computeHFromCellToCell(parent.parent->i, parent.parent->j, current.i, current.j, options);
        current.parent = parent.parent;
        return current;
    }
    return current;
}

void Theta::makeSecondaryPath(const Map &map, Node curNode)
{

    Node *forpath = &*hppath.List.begin();
    std::list<Node>::iterator iter = hppath.List.begin();
    iter++;
    Node *forpath2 = &*iter;
    int x0,x1,y0,y1,s_x,s_y,dx,dy,f;
    Node inpath;

    lppath.List.push_back(*forpath);
    while(forpath2 != &*hppath.List.end())
    {
        x0 = forpath->i;
        y0 = forpath->j;
        x1 = forpath2->i;
        y1 = forpath2->j;
        dx = x1-x0;
        dy = y1-y0;
        f = 0;

        if (dy<0)
        {
            dy = -dy;
            s_y = -1;
        }
        else
            s_y = 1;

        if (dx<0)
        {
            dx=-dx;
            s_x=-1;
        }
        else
            s_x=1;

        if (dx >= dy)
            while (x0 != x1)
            {
                f += dy;
                if (f >= dx)
                {
                    y0 += s_y;
                    f -= dx;
                }
                x0 += s_x;
                inpath.i = x0;
                inpath.j = y0;
                lppath.List.push_back(inpath);
            }
        else
            while (y0!=y1)
            {
                f += dx;
                if (f >= dy)
                {
                    x0 += s_x;
                    f -= dy;
                }

                y0 += s_y;
                inpath.i = x0;
                inpath.j = y0;
                lppath.List.push_back(inpath);
            }
        forpath = forpath2;
        iter++;
        forpath2 = &*iter;
    }
    sresult.lppath = &lppath;
}

void Theta::makePrimaryPath(Node curNode)
{
    Node current = curNode;
    while(current.parent)
    {
        hppath.List.push_front(current);
        current = *current.parent;
    }
    hppath.List.push_front(current);
    sresult.hppath = &hppath;
}
