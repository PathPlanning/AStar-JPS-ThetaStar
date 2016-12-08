#include "theta.h"
Theta::~Theta()
{
}

bool Theta::grid(int i, int j, const Map &map)
{
    if (!map.CellOnGrid(i,j)|| (map.CellOnGrid(i,j)&& !map.CellIsTraversable(i,j))) return true;
    else return false;
}

bool Theta::lineOfSight(int x1, int y1, int x0, int y0, const Map &map)
{
    //x1-parent_i, y1-parent_j, x0-current_i, y0-current_j
    int dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int dy = (y1 > y0) ? (y1 - y0) : (y0 - y1);
    if(dx==0)
    {
        for(int y=y0; y<=y1; y++)
            if(!map.CellIsTraversable(x0,y))
                return false;
        return true;
    }
    else if(dy==0)
    {
        for(int x=x0; x<=x1; x++)
            if(!map.CellIsTraversable(x,y0))
                return false;
        return true;
    }
    else if (dy < dx)
    {
        if (x1 < x0)
        {
            x1 += x0; x0 = x1 - x0; x1 -= x0;
            y1 += y0; y0 = y1 - y0; y1 -= y0;
        }
        float grad = (float)dy / dx;
        if(y1<y0)
            grad=-grad;
        float intery = y0 + grad;
        if(!map.CellIsTraversable(x0,y0))
            return false;
        for (int x = x0+1; x < x1; x++)
        {
            if(!map.CellIsTraversable(x,(int)intery))
                return false;
            if(!map.CellIsTraversable(x,(int)intery+1))
                return false;
            intery += grad;
        }
        if(!map.CellIsTraversable(x1,y1))
            return false;
    }
    else
    {
        if (y1 < y0)
        {

            x1 += x0; x0 = x1 - x0; x1 -= x0;
            y1 += y0; y0 = y1 - y0; y1 -= y0;
        }
        float grad = (float)dx / dy;
        if(x1<x0)
            grad=-grad;
        float interx = x0 + grad;
        if(!map.CellIsTraversable(x0,y0))
            return false;
        for (int y = y0+1; y < y1; y++)
        {
            if(!map.CellIsTraversable((int)interx,y))
                return false;
            if(!map.CellIsTraversable((int)interx+1,y))
                return false;
            interx += grad;
        }
        if(!map.CellIsTraversable(x1,y1))
            return false;
    }
    return true;
}


Node Theta::resetParent(Node current, Node parent, const Map &map, const EnvironmentOptions &options )
{
    if ((parent.i==map.start_i && parent.j==map.start_j) || (current.i==map.start_i && current.j==map.start_j))
        return current;
    if (lineOfSight(parent.parent->i, parent.parent->j,current.i, current.j, map))
    {
        if ((parent.parent->g + computeHFromCellToCell(parent.parent->i,parent.parent->j, current.i, current.j, options)-current.g )<0.0001)
        {
            current.g=parent.parent->g +computeHFromCellToCell(parent.parent->i,parent.parent->j, current.i, current.j, options);
            current.parent=parent.parent;
            return current;
        }
    }
    return current;
}

void Theta::makeSecondaryPath(const Map &map, Node curNode)
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

void Theta::makePrimaryPath(Node curNode)
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

