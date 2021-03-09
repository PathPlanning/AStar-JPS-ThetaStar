#include "path_smoothing.h"


void smooth_search_result(SearchResult& sr, const Map &map, bool cutcorners)
{
    if(!sr.pathfound)
    {
        return;
    }
    
    std::list<Node> new_lppath;
    std::list<Node> new_hppath;
    sr.pathlength = 0.0;
    new_hppath.push_back(sr.hppath->front());
    
    for(auto node = std::next(sr.hppath->begin()); node != std::prev(sr.hppath->end()); node++)
    {
        if(!Theta::lineOfSight(new_hppath.back().i, new_hppath.back().j, node->i, node->j, map, cutcorners))
        {
            sr.pathlength += static_cast<float>(Theta::distance(new_hppath.back().i, new_hppath.back().j, std::prev(node)->i, std::prev(node)->j));
            new_hppath.push_back(*std::prev(node));
        }
    }
    
    sr.pathlength += static_cast<float>(Theta::distance(new_hppath.back().i, new_hppath.back().j, sr.hppath->back().i, sr.hppath->back().j));
    new_hppath.push_back(sr.hppath->back());
    *sr.hppath = new_hppath;
    
    
    Node *forpath = &*new_hppath.begin();
    std::list<Node>::iterator iter = new_hppath.begin();
    iter++;
    Node *forpath2 = &*iter;
    Node inpath;
    new_lppath.push_back(*forpath);
    while(forpath2 != &*new_hppath.end()) {
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
                new_lppath.push_back(inpath);
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
                new_lppath.push_back(inpath);
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

    *sr.lppath = new_lppath;
}