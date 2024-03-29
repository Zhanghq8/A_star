#ifndef _A_STAR_H_
#define _A_STAR_H_

#include <vector>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <set>

namespace AStar
{   
    struct Vec2i
    {
        int x, y;
    };

    struct Node
    {
        Vec2i coordinates;
        Node* parent;
        float Cost_G, Cost_H, Cost_F;
        Node(Vec2i coordinates, Node *parent = NULL);
    };

    class A_Star
    {
    private:
        // A_star();
        std::vector<Vec2i> path;
        std::vector<std::vector<int> > map; 
        bool connected_flag;
        bool distance_flag;
        std::vector<std::vector<int> > x_connected;
    public:
        void set_conflag(bool connected_flag_ = false); // 0 for 8-connected, 1 for 4-connected
        void set_disflag(bool distance_flag_ = true); // 0 for manhattan, 1 for eucilidean
        void set_x_connected();
        bool isValid(Vec2i coordinates_); // check if the coordinate is valid
        bool isGoal(Vec2i source_, Vec2i goal_); // check if the coordinate is at goal pos
        double HScore_manhattan(Vec2i source_, Vec2i goal_); // calculate the mahattan distance from current to goal 
        double HScore_euclidean(Vec2i source_, Vec2i goal_); // calculate the euclidean distance from current to goal
        void Generate_map(std::string path_); // generate map from a txt file;
        void findPath(Vec2i source_, Vec2i goal_);
        Node* findNode(std::set<Node*>& nodes_, Vec2i coordinates_);
        void releaseNodes(std::set<Node*>& nodes_);
    };
}

#endif