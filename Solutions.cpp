#include "Maze.hpp"
#include <vector>
#include <stack>
#include <queue>
#include <algorithm>
using namespace std;

namespace solutions
{

int getNumberOfWalls(MazeNode *a_node)
{
    int wall_counter = 0;
    for (directions::nesw dir = directions::NORTH; dir < directions::EAST; dir = directions::nesw(dir + 1))
    {
        if (a_node->getDirectionNode(dir) == nullptr || a_node->getDirectionNode(dir)->isWall() || a_node->getDirectionNode(dir)->isVisited())
        {
            wall_counter++;
        }
    }
    return wall_counter;
}

bool canTravel(MazeNode *a_node)
{
    if (a_node->isVisited() || a_node->isWall())
    {
        return false;
    }
    return true;
}

bool dfs(stack<MazeNode> &path, Maze &maze, MazeNode *node, MazeNode *end) {
    //mark visited
    node->setVisited();
    bool isPath = false;
    
    // return true if its the end of maze
    if (node == end) {
        //cout << node->getStrPos() << " ";
        path.push(*node); 
        return true;
    }
    else {
        // check if there is a path from the nesw nodes
        MazeNode **dir = node->getDirectionNodeArray();
        for (int i = 0; i < 4; i++) {
            MazeNode* next = dir[i];
            // call dfs if node exists and can travel
            if (next != nullptr && canTravel(next)){
                isPath = dfs(path, maze, next, end);
                if (isPath) {
                    //cout << node->getStrPos()<< " ";
                    path.push(*node);
                    break;
                }
            }
        }
    }
    // returns false if there is no path from node
    return isPath;
}

std::vector<MazeNode> solveDFS(Maze &a_maze)
{
    // get start and end
    size_t row = a_maze.getLength(), col = a_maze.getWidth();
    MazeNode *start = a_maze.getFirstNode(), *end = a_maze.getLastNode();
    // stack to store nodes in path
    stack<MazeNode> path;
    // call helper function
    dfs(path, a_maze, start, end);
    vector<MazeNode> solution;
    // store nodes in vector and return it
    while(!path.empty()) {
        solution.push_back(path.top());
        path.pop();
    }
    return solution;
}

// returns path if at the end of maze, else returns empty vector
vector<MazeNode> addToPath(vector<vector<MazeNode>> &paths, MazeNode *node, MazeNode *prev, MazeNode *end) {
    int size = paths.size();
    
    // loop through all paths and add node to correct
    for (int j = 0; j < size; j++){
        vector<MazeNode> path = paths[j];
        MazeNode lastNode = path[path.size()-1];
        MazeNode second = path[path.size()-2];
        // correct path if last node in path is prev node
        if (lastNode.getStrPos() == prev->getStrPos()) {
            paths[j].push_back(*node);
            paths.push_back(path);
            //if (node->getStrPos() == end->getStrPos())
                return paths[j];
        }

        // check second to last in path of for nodes with 2 paths
        else if (second.getStrPos() == prev->getStrPos()) {
            path.pop_back();
            path.push_back(*node);
            paths.push_back(path);
            return path;
        }

    }
    return {};
}

std::vector<MazeNode> solveBFS(Maze &a_maze) 
{
    queue<MazeNode *> nodes, prev; // nodes to visit
    vector<vector<MazeNode>> paths; // all paths
    vector<MazeNode> solution;
    MazeNode *start = a_maze.getFirstNode(), *end = a_maze.getLastNode();
    nodes.push(start); // add start to queue
    prev.push(end);    
    paths.push_back({*start});
    start->setVisited();

    while (!nodes.empty()) {
        // visit and pop off first starting node
        MazeNode *cur = nodes.front();
        nodes.pop();

        
        cout << cur->getStrPos() << " ";
        
        // add cur node to paths 
        solution = addToPath(paths, cur, prev.front(), end);

        if (cur == end) {   cout << " bfs" << endl;
            if (!solution.empty())
                return solution;
        }

        // get neighbors
        MazeNode **dir = cur->getDirectionNodeArray();
        for(int i = 0; i < 4; i++) {
            MazeNode *neighbor = dir[i];

            // visit neighbors and add to queue
            if (neighbor != nullptr && canTravel(neighbor)) {
                neighbor->setVisited();
                nodes.push(neighbor);
                prev.push(cur);
            }
        }
        //update prev queue
        prev.pop();
    }
    
    return {};
}

// returns number of neighbors that is not a wall
int numOfNeighbors(MazeNode *node) {
    int count = 0; 
    MazeNode ** dir = node->getDirectionNodeArray();
    for (int i = 0; i < 4; i++) {
        if (dir[i] != nullptr && canTravel(dir[i])) {
            count++;
        }
    }
    return count;
}

// returns the pointer to neighbor
MazeNode *getNeighbor(MazeNode *node) {
    MazeNode ** dir = node->getDirectionNodeArray();
    for (int i = 0; i < 4; i++) {
        if (dir[i] != nullptr && canTravel(dir[i])) {
            return dir[i];
        }
    }
    return nullptr;
}

std::vector<MazeNode> solveDEF(Maze &a_maze)
{
    MazeNode *start = a_maze.getFirstNode(), *end = a_maze.getLastNode();
    vector<MazeNode *> deadEnds;
    pair<int, int> nodePos;
    MazeNode *node = start;
    a_maze.setCurrentNode(start);

    // find and save all deand ends
    for (int i = 0; i < a_maze.getNodes().size(); i++) {
        nodePos = a_maze.getNodes()[i].getPos();
        a_maze.setCurrentNode(a_maze.contains(nodePos));
        node = a_maze.getCurrentNode();
        if (numOfNeighbors(node) == 1 && node != start && node != end && canTravel(node)) {
            deadEnds.push_back(node);
        } 
    }

    // fill dead ends
    for (int i = 0; i < deadEnds.size(); i++) {
        node = deadEnds[i];
        // stop if node has more than 1 neighbor that is not a wall
        while (numOfNeighbors(node) == 1) {
            node->setWall();
            node = getNeighbor(node);
        }
    }  

    // go through maze and add all nodes that are not walls to solution 
    // starting with the first node
    node = start;
    vector<MazeNode> solution;
    while (node != end) {
        node->setVisited();
        solution.push_back(*node);
        node = getNeighbor(node);
    }
    solution.push_back(*end);
    
    return solution;
}   

// compare function for min-heap
class compare {
public:
    int operator()(const pair<pair<MazeNode *, MazeNode *>, int> &a, const pair<pair<MazeNode *, MazeNode *>, int> &b) {
        return a.second > b.second;
    }
};

// return distance from node to end
int distance(pair<int, int> node, pair<int, int> end) {
    return abs(end.second-node.second) + abs(end.first-node.first);
}

// A* algorithm
std::vector<MazeNode> solveCustom(Maze &a_maze)
{
    MazeNode *start = a_maze.getFirstNode(), *end = a_maze.getLastNode();
    // node closest to end is visited first
    // priority_queue<<curNode, prev>, distance> 
    priority_queue<pair<pair<MazeNode *,MazeNode*>, int>, vector<pair<pair<MazeNode *, MazeNode*>, int>>, compare> nodes; // nodes to visit
    vector<vector<MazeNode>> paths; // all paths
    vector<MazeNode> solution;
    
    start->setVisited();
    nodes.push({{start, end}, 0}); // add start to queue
    paths.push_back({*start});
    

    while (!nodes.empty()) {
        // visit and pop off first starting node
        pair<pair<MazeNode *,MazeNode*>, int> node = nodes.top();
        nodes.pop();

        MazeNode *cur = node.first.first, *prev = node.first.second;
        int dist = node.second;
        // add cur node to paths 
        
        solution = addToPath(paths, cur, prev, end);
        if (cur == end) { 
            return solution;
        }
    
        // get neighbors
        MazeNode **dir = cur->getDirectionNodeArray();
        for(int i = 0; i < 4; i++) {
            MazeNode *neighbor = dir[i];
            // visit neighbors and add to queue
            if (neighbor != nullptr && canTravel(neighbor)) {
                neighbor->setVisited();
                prev = cur;
                dist = solution.size() + distance(neighbor->getPos(), end->getPos());
                nodes.push({{neighbor,prev}, dist});
            }
        }
    }
    return {};
} 
} // namespace solutions
