#include "isearch.h"
#include "iostream"


ISearch::ISearch()
{
    opennodes = 0;
    hweight = 1;
    breakingties = CN_SP_BT_GMAX;
}

ISearch::~ISearch(void) {}




void ISearch::addOpen(Node newnd, const Map &map)
{
    open.insert({newnd.i * map.getMapWidth() + newnd.j, newnd});
}




SearchResult ISearch::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{

    std::chrono::time_point<std::chrono::system_clock> begin, end;
    begin = std::chrono::system_clock::now();
    bool found = false;
    int steps = 0;
    std::pair<int, int> goal;
    goal = map.get_goal();
    Node current;
    current.i = map.getstart_i();
    current.j = map.getstart_j();
    current.parent = nullptr;
    current.g = 0;
    current.H = computeHFromCellToCell(current.i, current.j, goal.first, goal.second, options);
    current.F = current.H * hweight;
    open.insert({current.i * map.getMapWidth() + current.j, current});
    while (open.size() != 0) {
        ++steps;
        current = (*open.begin()).second;
        double f = (*open.begin()).second.F;
        for (auto i : open) {
            if (i.second.F < f) {
                current = i.second;
                f = current.F;
            }
        }
        close.insert({current.i * map.getMapWidth() + current.j, current});
        open.erase(current.i * map.getMapWidth() + current.j);

        if((current.i == goal.first) && (current.j == goal.second)) {

            found = true;
            break;
        }


        std::list<Node> sc = findSuccessors(current, map, options);
        Node * par = &(close.find(current.i * map.getMapWidth() + current.j)->second);
        for (auto x : sc) {
            if (open.find(x.i * map.getMapWidth() + x.j) == open.end()) {
                x.parent = par;
                x = updateVertex(x, *par, map, options);
                x.H = computeHFromCellToCell(x.i, x.j, goal.first, goal.second, options);
                x.F = x.g + x.H;

                open.insert({x.i * map.getMapWidth() + x.j, x});
            } else {
                if(x.g < open[x.i * map.getMapWidth() + x.j].g) {
                    open[x.i * map.getMapWidth() + x.j].g = x.g;
                    open[x.i * map.getMapWidth() + x.j].parent = par;
                    open[x.i * map.getMapWidth() + x.j] = updateVertex(open[x.i * map.getMapWidth() + x.j], *par, map, options);
                    open[x.i * map.getMapWidth() + x.j].F = x.g + open[x.i * map.getMapWidth() + x.j].H;
                }
            }
        }
    }

    if (found) {

        sresult.pathfound = true;
        makePrimaryPath(current);
        sresult.pathlength = current.g;

    }
    end = std::chrono::system_clock::now();
    sresult.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count()) / 1000000000;
    if (found) {
        makeSecondaryPath();
    }
    sresult.nodescreated = open.size() + close.size();
    sresult.numberofsteps = steps;
    sresult.hppath = &hppath;
    sresult.lppath = &lppath;
    return sresult;
}

std::list<Node> ISearch::findSuccessors(Node curNode, const Map &map, const EnvironmentOptions &options)
{
    Node c;
    std::vector<int> range = {-1, 0, 1};
    std::list<Node> successors;
    for (int i : range) {
        for (int j : range) {
            if (!(i == 0 && j == 0) && map.CellOnGrid(curNode.i + i, curNode.j + j) && map.CellIsTraversable(curNode.i + i, curNode.j + j)) {
                if (close.find((curNode.i + i) * map.getMapWidth() + curNode.j + j) == close.end()) {
                    if (i * i + j * j == 1) {
                        c.i = curNode.i + i;
                        c.j = curNode.j + j;
                        c.g = curNode.g + hweight;
                        successors.push_front(c);
                    } else if ((i * i + j * j == 2) && options.allowdiagonal) {
                        if ((map.CellIsTraversable(curNode.i + i, curNode.j) && map.CellIsTraversable(curNode.i, curNode.j + j)) || options.allowsqueeze || (options.cutcorners && (map.CellIsTraversable(curNode.i + i, curNode.j) || map.CellIsTraversable(curNode.i, curNode.j + j)))) { //проверка что нет препятствий/можно просачиваться/можно срезать углы и только одна из клеток препятствие
                            c.i = curNode.i + i;
                            c.j = curNode.j + j;
                            c.g = curNode.g + sqrt(2 * hweight);
                            successors.push_front(c);
                        }
                    }
                }
            }

        }
    }
    return successors;
}



void ISearch::makePrimaryPath(const Node &curNode)
{
    Node current = curNode;

    while (current.parent != nullptr) {
        lppath.push_front(current);
        Node a = *(current.parent);
        current = a;
    }
    lppath.push_front(current);
}

void ISearch::makeSecondaryPath()
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
