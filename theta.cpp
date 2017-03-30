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
    if(delta_i == 0) {
        for(; j != j2; j += step_j)
            if(map.CellIsObstacle(i, j))
                return false;
        return true;
    }
    else if(delta_j == 0) {
        for(; i != i2; i += step_i)
            if(map.CellIsObstacle(i, j))
                return false;
        return true;
    }
    if(cutcorners) {
        if (delta_i > delta_j) {
            for (; i != i2; i += step_i) {
                if (map.CellIsObstacle(i, j))
                    return false;
                error += delta_j;
                if ((error << 1) > delta_i) {
                    if(((error << 1) - delta_j) < delta_i && map.CellIsObstacle(i+step_i, j))
                            return false;
                    else if(((error << 1) - delta_j) > delta_i && map.CellIsObstacle(i, j+step_j))
                            return false;
                    j += step_j;
                    error -= delta_i;
                }
            }
        }
        else {
            for (; j != j2; j += step_j) {
                if (map.CellIsObstacle(i, j))
                    return false;
                error += delta_i;
                if ((error << 1) > delta_j) {
                    if(((error << 1) - delta_i) < delta_j && map.CellIsObstacle(i, j+step_j))
                            return false;
                    else if(((error << 1) - delta_i) > delta_j && map.CellIsObstacle(i+step_i, j))
                            return false;
                    i += step_i;
                    error -= delta_j;
                }
            }
        }

    }
    else {
        int sep_value = delta_i*delta_i + delta_j*delta_j;
        if(delta_i > delta_j) {
            for(; i != i2; i += step_i) {
                if(map.CellIsObstacle(i, j))
                    return false;
                if(map.CellIsObstacle(i, j + step_j))
                    return false;
                error += delta_j;
                if(error >= delta_i) {
                    if(((error << 1) - delta_i - delta_j)*((error << 1) - delta_i - delta_j) < sep_value)
                        if(map.CellIsObstacle(i + step_i,j))
                            return false;
                    if((3*delta_i - ((error << 1) - delta_j))*(3*delta_i - ((error << 1) - delta_j)) < sep_value)
                        if(map.CellIsObstacle(i, j + 2*step_j))
                            return false;
                    j += step_j;
                    error -= delta_i;
                }
            }
            if(map.CellIsObstacle(i, j))
                return false;
        }
        else {
            for(; j != j2; j += step_j) {
                if(map.CellIsObstacle(i, j))
                    return false;
                if(map.CellIsObstacle(i + step_i, j))
                    return false;
                error += delta_i;
                if(error >= delta_j) {
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
    if (parent.parent == nullptr)
        return current;
    if(current == *parent.parent)
        return current;
    if (lineOfSight(parent.parent->i, parent.parent->j, current.i, current.j, map, options.cutcorners)) {
        current.g = parent.parent->g + distance(parent.parent->i, parent.parent->j, current.i, current.j);
        current.parent = parent.parent;
        return current;
    }
    return current;
}

double Theta::distance(int i1, int j1, int i2, int j2)
{
    return sqrt(pow(i1 - i2, 2) + pow(j1 - j2, 2));
}

void Theta::makeSecondaryPath()
{
    Node *forpath = &*hppath.begin();
    std::list<Node>::iterator iter = hppath.begin();
    iter++;
    Node *forpath2 = &*iter;
    Node inpath;
    lppath.push_back(*forpath);
    while(forpath2 != &*hppath.end()) {
        int i1 = forpath->i;
        int j1 = forpath->j;
        int i2 = forpath2->i;
        int j2 = forpath2->j;
        int delta_i = std::abs(i1 - i2);
        int delta_j = std::abs(j1 - j2);
        int step_i = (i1 < i2 ? 1 : -1);
        int step_j = (j1 < j2 ? 1 : -1);
        int error = 0;
        if (delta_i > delta_j) {
            int j = j1;
            for (int i = i1; i != i2; i += step_i) {
                inpath.i = i;
                inpath.j = j;
                lppath.push_back(inpath);
                error += delta_j;
                if ((error << 1) > delta_i) {
                    j += step_j;
                    error -= delta_i;
                }
            }
        }
        else {
            int i = i1;
            for (int j = j1; j != j2; j += step_j) {
                inpath.i = i;
                inpath.j = j;
                lppath.push_back(inpath);
                error += delta_i;
                if ((error << 1) > delta_j) {
                    i += step_i;
                    error -= delta_j;
                }
            }
        }
        forpath = forpath2;
        iter++;
        forpath2 = &*iter;
    }
}

void Theta::makePrimaryPath(Node curNode)
{
    Node current = curNode;
    while(current.parent) {
        hppath.push_front(current);
        current = *current.parent;
    }
    hppath.push_front(current);
}
