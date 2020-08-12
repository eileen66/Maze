/****************************************************************************************************************************
Title :       MazeNode.hpp
Auhor :       Nigel Ferrer
Description : implementation file of a linked-based maze
Dependencies: Maze.cpp, MazeNode.hpp, MazeNode.cpp 
****************************************************************************************************************************/

#include "Maze.hpp"
#include <sstream>
#include <fstream>
#include <queue>

/************************************ start: namespace helpers ***************************************/
namespace helpers
{
/* converts a string position to a pair
   and returns that pair
   @pre: the parameter str is a string position of format (x, y)
*/
std::pair<int, int> getPositionFromString(const std::string &str)
{
    if (str[0] != '(')
    {
        return std::make_pair(-1, -1); // flag value for invalid string
    }
    size_t pos = str.find("/");
    std::string first_half, second_half;
    first_half = str.substr(1, pos - 1);
    second_half = str.substr(pos + 1);
    std::stringstream remove_paren(second_half);
    getline(remove_paren, second_half, ')');
    return std::make_pair(std::stoi(first_half), std::stoi(second_half));
}

/* converts a string position to a pair
   and creates a node at that position 
   @pre: the paramater str is a string position of format (x, y)
*/
MazeNode getNodeFromString(const std::string &str)
{
    std::pair<int, int> pos = getPositionFromString(str);
    MazeNode *default_dir[4] = {nullptr, nullptr, nullptr, nullptr};
    MazeNode start_node(pos, default_dir);
    return start_node;
}
} // namespace helpers
/************************************* end: namespace helpers ****************************************/

/* Default Constructor */
Maze::Maze() : size_{0}, length_{0}, width_{0}, start_node_{nullptr}, end_node_{nullptr}, current_node_{nullptr}
{
    nodes_.clear();
}

/* Parameterized Constructor */
Maze::Maze(std::string in_file)
{
    std::fstream fin;                // instantiates a filestream variable
    fin.open(in_file, std::ios::in); // assigns to it the content of the maze file
    if (fin.fail())
    {
        std::cerr << "File cannot be opened. Maze has not been constructed." << std::endl;
        return;
    }

    std::string temp, line;
    getline(fin, line);
    getline(fin, line); // gets maze qualifier information from csv file

    std::stringstream ss_qualifier(line);
    std::string qualifier_string[5];
    int i = 0;

    while (getline(ss_qualifier, temp, ','))
    {
        qualifier_string[i] = temp; // inserts qualifier information into temporary array
        i++;
    }

    // update simple private members
    length_ = std::stoi(qualifier_string[3]);
    width_ = std::stoi(qualifier_string[4]);

    // iterate through csv file and store all node positions and their related info
    getline(fin, line);
    std::vector<std::vector<std::string>> node_holder;
    while (getline(fin, line))
    {
        std::stringstream node_info(line);
        std::vector<std::string> node_pieces;
        while (getline(node_info, temp, ','))
        {
            node_pieces.push_back(temp);
        }
        node_holder.push_back(node_pieces);
    }

    // inserts each node into the maze
    for (size_t i = 0; i < node_holder.size(); i++)
    {
        if (contains(helpers::getPositionFromString(node_holder[i][0])) == nullptr)
        {
            MazeNode *default_dirs[4] = {nullptr, nullptr, nullptr, nullptr};
            MazeNode not_updated(helpers::getPositionFromString(node_holder[i][0]), default_dirs);
            insertNode(not_updated);
        }
    }

    // update each node's neighbor array with its respective content
    for (size_t i = 0; i < node_holder.size(); i++)
    {
        MazeNode *the_node = contains(helpers::getPositionFromString(node_holder[i][0]));

        MazeNode *connected_node_N = contains(helpers::getPositionFromString(node_holder[i][1]));
        the_node->setDirectionNode(connected_node_N, directions::NORTH);

        MazeNode *connected_node_E = contains(helpers::getPositionFromString(node_holder[i][2]));
        the_node->setDirectionNode(connected_node_E, directions::EAST);

        MazeNode *connected_node_S = contains(helpers::getPositionFromString(node_holder[i][3]));
        the_node->setDirectionNode(connected_node_S, directions::SOUTH);

        MazeNode *connected_node_W = contains(helpers::getPositionFromString(node_holder[i][4]));
        the_node->setDirectionNode(connected_node_W, directions::WEST);

        // update each node's wall status
        if (node_holder[i][5][0] == '1' || node_holder[i][5] == "1")
        {
            the_node->setWall();
        }
    }

    // set size
    size_ = std::stoi(qualifier_string[0]);

    // set start_node_ to the proper node
    start_node_ = contains(helpers::getPositionFromString(qualifier_string[1].substr(0, (qualifier_string[1].size()))));

    // set current_node_ to the proper node - before any solution algorithm is applied this should be the start_node_
    current_node_ = start_node_;

    // set end_node_ to the proper node
    end_node_ = contains(helpers::getPositionFromString(qualifier_string[2].substr(0, (qualifier_string[2].size()))));

} // end Parameterized Constructor

/************************************* start: Getter Functions ***************************************/
std::vector<MazeNode> Maze::getNodes() const
{
    return nodes_;
}
size_t Maze::getSize() const
{
    return size_;
}
size_t Maze::getLength() const
{
    return length_;
}

size_t Maze::getWidth() const
{
    return width_;
}

MazeNode *Maze::getFirstNode() const
{
    return start_node_;
}

MazeNode *Maze::getCurrentNode() const
{
    return current_node_;
}

MazeNode *Maze::getLastNode() const
{
    return end_node_;
}

/************************************** end: Getter Functions ****************************************/

// used to internally iterate over nodes in the maze
void Maze::setCurrentNode(MazeNode *a_node)
{
    current_node_ = a_node;
}

MazeNode *Maze::contains(const std::pair<int, int> &pos)
{
    if (std::get<0>(pos) == -1)
    {
        return nullptr;
    }

    for (auto &x : nodes_)
    {
        if (pos == x.getPos())
        {
            MazeNode *reference = &x;
            return reference; /*  ensures a pointer to the actual node in the nodes_
                                  vector is returned as opposed to a pointer that 
                                  points to a copy of that node 
                              */
        }
    }
    return nullptr;
}

void Maze::insertNode(const MazeNode &a_node)
{
    nodes_.push_back(a_node);
    size_++;
}

std::ostream &operator<<(std::ostream &os, Maze &maze)
{
    os << "Dimensions: " << maze.getLength() << " x " << maze.getWidth() << "\n"
       << "Size: " << maze.getSize() << "\n"
       << "Start Node Position: " << maze.getFirstNode()->getStrPos() << "\n"
       << "End Node Position: " << maze.getLastNode()->getStrPos() << "\n"
       << "Nodes: \n";
    std::vector<MazeNode> nodes = maze.getNodes();
    for (size_t i = 0; i < nodes.size(); i++)
    {
        os << "\t" << nodes[i];
        if (i != nodes.size() - 1)
        {
            os << std::endl;
        }
    }
    return os;
}
