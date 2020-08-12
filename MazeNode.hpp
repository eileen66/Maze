/**********************************************************************************************************************
Title :       MazeNode.hpp
Auhor :       Nigel Ferrer
Description : header/interface file of a quadruply-linked node class - intended 
              to be used by Maze class 
**********************************************************************************************************************/

#include <utility>
#include <iostream>

#ifndef MAZE_NODE_
#define MAZE_NODE_

/* creates a context-specific definition of North, East, South, and West directions */
namespace directions
{
enum nesw
{
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
};
}

class MazeNode
{
private:
    MazeNode *nesw_nodes_[4] = {nullptr, nullptr, nullptr, nullptr}; /* links that correspond to the directions in 
                                                                        which an iterator can travel from the node 
                                                                        - intended storage is in the order:
                                                                         N, E, S, W 
                                                                     */
    bool visited_ = false;
    bool wall_;
    std::pair<int, int> grid_pos_; /* ordered pair [ (x,y) coordinate ] position of node in the grid that 
                                      represents the maze; the positions are ordered in the same fashion
                                      as is a numpy array. Examples: (0,0) is the top-left corner, (0,1) 
                                      is 1 position to the right of the top-left corner. (1,0) is
                                      1 position down from the top-left corner
                                    */

public:
    /* Default Constructor */
    MazeNode();

    /* Parameterized Constructor */
    MazeNode(std::pair<int, int> grid_pos, MazeNode *dir[4] = nullptr, const bool &isWall = false);

    /* Getter Functions */
    MazeNode **getDirectionNodeArray();                      // returns a pointer to the calling-node's direction node array
    MazeNode *getDirectionNode(const directions::nesw &dir); // returns a pointer to the direction node
    bool isVisited() const;                                  // returns visited_
    bool isWall() const;                                     // returns wall_
    std::pair<int, int> getPos() const;                      // returns the calling node's position as an integer pair
    std::string getStrPos() const;                           // returns the calling node's position as a string

    /* Setter Functions */
    void setDirectionNode(MazeNode *node, const directions::nesw &dir); /* sets the calling node's direction node to
                                                                           specified node in the specified direction
                                                                        */
    void setVisited();                                                  // sets visited_ to true
    void setWall();                                                     // sets wall_ to true
    void resetVisited();                                                // sets visited_ to false

    /* Insertion Operator Overload - gives a context-specific meaning 
       to the insertion operator in reference to MazeNode objects
    */
    friend std::ostream &operator<<(std::ostream &os, MazeNode &node);
};

#endif // end MazeNode