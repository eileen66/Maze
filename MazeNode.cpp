/**********************************************************************************************************************
Title :       MazeNode.cpp
Auhor :       Nigel Ferrer
Description : implemenatation file of a quadruply-linked node class - intended 
              to be used by Maze class 
Dependencies: MazeNode.hpp
**********************************************************************************************************************/

#include "MazeNode.hpp"
#include <sstream>

/* Default Constructor */
MazeNode::MazeNode() : wall_{false}, grid_pos_{std::make_pair(0, 0)}
{
    for (size_t i = 0; i < 4; i++)
    {
        nesw_nodes_[i] = nullptr;
    }
}

/* Parameterized Constructor */
MazeNode::MazeNode(std::pair<int, int> grid_pos, MazeNode *nesw_nodes[4], const bool &isWall) : wall_{isWall}, grid_pos_{grid_pos}
{
    for (size_t i = 0; i < 4; i++)
    {
        nesw_nodes_[i] = nesw_nodes[i];
    }
}

/************************************* start: Getter Functions ***************************************/

MazeNode **MazeNode::getDirectionNodeArray()
{
    MazeNode **directions_ptr = nesw_nodes_;
    return directions_ptr;
}

MazeNode *MazeNode::getDirectionNode(const directions::nesw &direction)
{
    return nesw_nodes_[direction];
}

bool MazeNode::isVisited() const
{
    return visited_;
}

bool MazeNode::isWall() const
{
    return wall_;
}

std::pair<int, int> MazeNode::getPos() const
{
    return grid_pos_;
}

std::string MazeNode::getStrPos() const
{
    /* in order to obtain an acceptable
       output, it is necessary to extract
       each element of the pair and convert
       that element to a string - used
       stringstreams here to accomplish that
     */

    std::string left_pos_string, right_pos_string;
    std::stringstream left_pos;
    left_pos << std::get<0>(grid_pos_);
    left_pos >> left_pos_string;

    std::stringstream right_pos;

    right_pos << std::get<1>(grid_pos_);

    right_pos >> right_pos_string;

    return "(" + left_pos_string + "," + right_pos_string + ")"; // returns in the format: (x, y)
}
/************************************** end: Getter Functions ****************************************/

/************************************* start: Setter Functions ***************************************/
void MazeNode::setDirectionNode(MazeNode *node, const directions::nesw &direction)
{
    nesw_nodes_[direction] = node;
}

void MazeNode::setVisited()
{
    visited_ = true;
}

void MazeNode::setWall()
{
    wall_ = true;
}

void MazeNode::resetVisited()
{
    visited_ = false;
}
/************************************** end: Setter Functions ****************************************/

/* Insertion Operator Overload 

    note that this function is not a member of the MazeNode class
    in order for an operator overload to be a member function, it must have a
    first parameter of its respective class type - here the first parameter
    is an output stream object; thus, in this case and in general, a friend function 
    is the only way of implementing an overloaded operator<<
*/
std::ostream &operator<<(std::ostream &os, MazeNode &node)
{
    std::string dir[4] = {"North", "East", "South", "West"};
    os << "Node: " << node.getStrPos();
    for (size_t i = 0; i < 4; i++)
    {
        MazeNode *dir_node = (node.getDirectionNodeArray())[i];
        os << " | " << dir[i] << ": ";
        if (dir_node != nullptr)
        {
            os << dir_node->getStrPos();
        }
        else
        {
            os << "NONE";
        }
    }
    os << " | Wall: " << node.isWall();
    return os;
}
