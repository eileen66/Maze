/****************************************************************************************************************************
Title :       MazeNode.hpp
Auhor :       Nigel Ferrer
Description : header/interface file of a linked-based maze
Dependencies: MazeNode.hpp, MazeNode.cpp 
****************************************************************************************************************************/

#include "MazeNode.hpp"
#include <vector>

#ifndef MAZE_
#define MAZE_

class Maze
{
private:
    std::vector<MazeNode> nodes_; // vector that stores all MazeNodes of a respective Maze object
    size_t size_;                 // stores the total number of nodes in the Maze
    size_t length_;
    size_t width_;
    MazeNode *start_node_;   // pointer to the node that has been distinguished as the Maze's start point
    MazeNode *end_node_;     // pointer to the node that has been distinguished as the Maze's end point
    MazeNode *current_node_; // pointer to the node that is currently being processed - meant to act as an internal iterator

public:
    /* Default Constructor */
    Maze();

    /* Parameterized Constructor - reads a CSV file that contains node information */
    Maze(std::string in_file);

    /* Getter Functions */
    std::vector<MazeNode> getNodes() const; // returns nodes_ vector of nodes in Maze object
    size_t getSize() const;                 // returns size_
    size_t getLength() const;               // returns length_
    size_t getWidth() const;                // returns width_
    MazeNode *getFirstNode() const;         // returns start_node_
    MazeNode *getLastNode() const;          // returns end_node_
    MazeNode *getCurrentNode() const;       // returns current_node_

    /* Sets Current Node */
    void setCurrentNode(MazeNode *a_node);

    /* returns a pointer to a node if it is at the position of parameter pos
       otherwise, it returns nullptr
    */
    MazeNode *contains(const std::pair<int, int> &pos);

    /* Inserts Node */
    void insertNode(const MazeNode &a_node);

    /* Assignment Operator Overload */
    void operator=(Maze &maze);

    /* Insertion Operator Overload */
    friend std::ostream &operator<<(std::ostream &os, Maze &maze);
};
#endif // end Maze
