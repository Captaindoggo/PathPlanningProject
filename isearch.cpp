#include "isearch.h"
#include "iostream"


ISearch::ISearch()
{
    opennodes = 0;
    hweight = 1;
    breakingties = CN_SP_BT_GMAX;
}

ISearch::~ISearch(void) {}




void ISearch::addOpen(const Node &newnd)
{
    if (open.size() == 0) {
        //std::cout << "first" << '\n';
        open.push_back(newnd);
        return;
        //std::cout << open.size() << '\n';
    } else if (open[0].F > newnd.F) {
        //std::cout << " before first" << '\n';
        open.push_front(newnd);
        return;
    } else if (open[open.size() - 1].F < newnd.F) {
        //std::cout << "last" << '\n';
        open.push_back(newnd);
        return;
    }
    std::deque<Node>::iterator it = open.begin();
    for (size_t i = 0; i < open.size(); ++i) {
        if (open[i].F < newnd.F && open[i + 1].F > newnd.F) {
            //std::cout << "in place "<< i << '\n';
            it += i + 1;
            open.insert(it, newnd);
            return;
        }
    }
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
    std::cout << current.i << ' ' << current.j <<'\n';
    std::cout << goal.first << ' ' << goal.second <<'\n';
    addOpen(current);
    while (open.size() != 0) {
        ++steps;
        current = open[0];
        open.pop_front();
        close.insert({current.i * map.getMapWidth() + current.j, current});
        //std::cout << "current:" << current.i - goal.first << ' ' << current.j - goal.second << '\n';

        if(current.i == goal.first && current.j == goal.second) {
            found = true;
            break;
        }


        for (auto i : findSuccessors(current, map, options)) {
            //std::cout << open.size() << " before" << '\n';
            addOpen(i);
            //std::cout << open.size() << " after" << '\n';
        }
    }
    end = std::chrono::system_clock::now();
    sresult.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count()) / 1000000000;
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
    for (int i : range) {
        for (int j : range) {
            if (curNode.i + i >= 0 && curNode.i + i <= map.getMapWidth() - 1 && curNode.j + j >= 0 && curNode.j + j <= map.getMapHeight() - 1) {
                if (!(map.CellIsObstacle(curNode.i + i, curNode.j + j))) {
                    if (close.find((curNode.i + i) * map.getMapWidth() + curNode.j + j) == close.end()) {
                        bool in_open = false;
                        size_t index;
                        for (size_t k = 0; k != open.size(); ++k) {
                            if(open[k].i == curNode.i + i && open[k].j == curNode.j + j) {
                                in_open = true;
                                index = k;
                            }
                        }

                        if(in_open) {
                            double cur_g = map.getMapWidth();
                            if(i * i + j * j == 1) {
                                cur_g = curNode.g + 1;
                            } else if(i * i + j * j == 2) {
                                cur_g = curNode.g + sqrt(2 * hweight);
                            }
                            if (cur_g < open[index].g) {
                                Node c;
                                c.i = curNode.i + i;
                                c.j = curNode.j + j;
                                c.parent = &curNode;
                                std::pair<int, int> goal;
                                goal = map.get_goal();
                                c.g = cur_g;
                                c.H = computeHFromCellToCell(c.i, c.j, goal.first, goal.second, options);
                                c.F = c.g + c.H;
                                open[index] = c;
                            }
                        } else {
                            if (options.allowdiagonal) {
                                if (i * i + j * j == 1) {
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
                                } else if (i * i + j * j == 2) {
                                    if (options.allowsqueeze || !(map.CellIsObstacle(curNode.i, curNode.j + j) && map.CellIsObstacle(curNode.i + i, curNode.j)) || (options.cutcorners && (map.CellIsObstacle(curNode.i, curNode.j + j) xor map.CellIsObstacle(curNode.i + i, curNode.j)))) {
                                        Node c;
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
                                }

                            } else {
                                if (i * i + j * j == 1) {
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
