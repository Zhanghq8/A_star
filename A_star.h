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
        public:
            // A_star();
            std::vector<Vec2i> path;
            std::vector<std::vector<int> > map; 
            int connected_flag;
            std::vector<std::vector<int> > x_connected;
            void set_conflag(int connected_flag_ = 1); // 1 for 8-connected, 0 for 4-connected
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