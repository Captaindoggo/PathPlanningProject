#include "theta.h"
Theta::~Theta(){}

Node Theta::updateVertex(Node s, Node ps, const Map &map, const EnvironmentOptions &options)
{
    if (ps.parent == nullptr)
        return s;
    if (lineOfSight(ps.parent->i, ps.parent->j, s.i, s.j, map)) {
        s.g = ps.parent->g + dist(ps.parent->i, ps.parent->j, s.i, s.j);
        s.parent = ps.parent;
        return s;
    }
    return s;
}

double Theta::dist(int i1, int j1, int i2, int j2)
{
    return sqrt((i1 - i2) * (i1 - i2) + (j1 - j2) * (j1 - j2));
}

bool Theta::lineOfSight(int i1, int j1, int i2, int j2, const Map &map) {
    int di = std::abs(i1 - i2);
    int dj = std::abs(j1 - j2);
    int f = 0;
    int si, sj;
    if (i2 - i1 < 0) {
        si = -1;
    } else {
        si = 1;
    }
    if (j2 - j1 < 0) {
        sj = -1;
    } else {
        sj = 1;
    }
    if (di >= dj) {
        while (i1 != i2) {
            f += dj;
            if (f >= di) {
                if (map.CellIsObstacle(i1 + (si - 1) / 2, j1 + (sj - 1) / 2))
                    return false;
                j1 += sj;
                f -= di;
            }
            if (f != 0 && map.CellIsObstacle(i1 + (si - 1) / 2, j1 + (sj - 1) / 2))
                return false;
            if (sj == 0 && map.CellIsObstacle(i1 + (si - 1) / 2, j1) && map.CellIsObstacle(i1 + (si - 1) / 2, j1 - 1))
                    return false;
            i1 += si;
        }
    } else {
        while (j1 != j2) {
            f += di;
            if (f >= dj) {
                if (map.CellIsObstacle(i1 + (si - 1) / 2, j1 + (sj - 1) / 2))
                    return false;
                i1 += si;
                f -= dj;
            }
            if (f != 0 && map.CellIsObstacle(i1 + (si - 1) / 2, j1 + (sj - 1) / 2))
                return false;
            if (si == 0 && map.CellIsObstacle(i1, j1 + (sj - 1) / 2) && map.CellIsObstacle(i1 - 1, j1 + (sj - 1) / 2))
                return false;
            j1 += sj;
        }
    }
    return true;
}


void Theta::makePrimaryPath(Node curNode)
{
    Node current = curNode;

    while (current.parent != nullptr) {
        lppath.push_front(current);
        Node a = *(current.parent);
        current = a;
    }
    lppath.push_front(current);
}

void Theta::makeSecondaryPath()
{
    std::list<Node>::const_iterator iter = lppath.begin();
    int x, y, nx, ny;
    int dirx, diry;
    hppath.push_back(*iter);
    while (iter != --lppath.end()) {
        x = iter->i;
        y = iter->j;
        ++iter;
        nx = iter->i;
        ny = iter->j;
        dirx = nx - x;
        diry = ny - y;
        ++iter;
        if ((iter->i - nx) != dirx || (iter->j - ny) != diry) {
            hppath.push_back(*(--iter));
        } else {
            --iter;
        }

    }
}
