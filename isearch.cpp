#include "isearch.h"


ISearch::ISearch()
{
    opennodes = 0;
    hweight = 1;
    breakingties = CN_SP_BT_GMAX;
}

ISearch::~ISearch(void) {}




void ISearch::addOpen(Node newnd)
{
    ++opennodes;
    if (open.size() == 0) {
        open.push_back(newnd);
    } else if (open[0].F > newnd.F) {
        open.push_front(newnd);
    } else if (open[open.size() - 1].F < newnd.F) {
        open.push_back(newnd);
    }
    std::deque<Node>::iterator it = open.begin();
    for (size_t i = 0; i < open.size(); ++i) {
        if (open[i].F < newnd.F && open[i + 1].F > newnd.F) {
            it += i + 1;
            open.insert(it, open.begin(), open.end());
        }
    }
}

SearchResult ISearch::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
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
    addOpen(current);
    while (open.size() != 0) {
        ++steps;
        current = open[0];
        open.pop_front();
        close.insert({current.i * map.getMapWidth() + current.j, current});
        if (current.i == goal.first && current.j == goal.second) {
            found = true;
            break;
        }
        current = open[0];
        open.pop_front();
        close.insert({current.i * map.getMapWidth() + current.j, current});
        for (auto i : findSuccessors(current, map, options)) {
            addOpen(i);
        }
    }
    if (found) {
        sresult.pathfound = true;
        makePrimaryPath(current);
    }
    sresult.nodescreated = open.size() + close.size();
    sresult.numberofsteps = steps;
    //sresult.time = ;
    //sresult.hppath = &hppath; //Here is a constant pointer
    sresult.lppath = &lppath;
    return sresult;
}

std::list<Node> ISearch::findSuccessors(Node curNode, const Map &map, const EnvironmentOptions &options)
{
    std::vector<int> range = {-1, 0, 1};
    std::list<Node> successors;
    for (auto i : range) {
        for (auto j : range) {
            if (curNode.i + i >= 0 && curNode.i + i <= map.getMapWidth()) {
                if (curNode.j + j >= 0 && curNode.j + j <= map.getMapHeight()) {
                    if(!(map.CellIsObstacle(i, j))) {
                        if (close.find((curNode.i + i) * map.getMapWidth() + curNode.j + j) == close.end()) {
                            if (options.allowdiagonal) {
                                if (i * i + j * j != 0) {
                                    Node c;
                                    if (i * i + j * j == 2) {
                                        if (options.allowsqueeze || !(map.CellIsObstacle(i, j - j) && map.CellIsTraversable(i - i, j)) || (options.allowsqueeze && (map.CellIsObstacle(i, j - j) xor map.CellIsTraversable(i - i, j)))) {
                                            c.i = curNode.i + i;
                                            c.j = curNode.j + j;
                                            c.parent = &curNode;
                                            std::pair<int, int> goal;
                                            goal = map.get_goal();
                                            c.g = curNode.g + sqrt(2 * hweight);
                                            c.H = computeHFromCellToCell(c.i, c.j, goal.first, goal.second, options);
                                            c.F = c.g + c.H;
                                            successors.push_back(c);
                                        }
                                    } else {
                                        Node c;
                                        c.i = curNode.i + i;
                                        c.j = curNode.j + j;
                                        c.parent = &curNode;
                                        std::pair<int, int> goal;
                                        goal = map.get_goal();
                                        c.g = curNode.g + hweight;
                                        c.H = computeHFromCellToCell(c.i, c.j, goal.first, goal.second, options);
                                        c.F = c.g + c.H;
                                        successors.push_back(c);
                                    }
                                }
                            } else {
                                Node c;
                                c.i = curNode.i + i;
                                c.j = curNode.j + j;
                                c.parent = &curNode;
                                std::pair<int, int> goal;
                                goal = map.get_goal();
                                c.g = curNode.g + hweight;
                                c.H = computeHFromCellToCell(c.i, c.j, goal.first, goal.second, options);
                                c.F = c.g + c.H;
                                successors.push_back(c);
                            }
                        }
                    }
                }
            }
        }
    }
    return successors;
}

void ISearch::makePrimaryPath(Node curNode)
{
    Node current = curNode;
        while (current.parent != nullptr) {
            lppath.push_front(current);
            current = *current.parent;
        }
        lppath.push_front(current);
}

/*void ISearch::makeSecondaryPath()
{
    //need to implement
}
*/
