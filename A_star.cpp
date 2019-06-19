#include "A_star.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <iterator>

// AStar::A_Star::A_star()
// {
// 	// Generate_map("easy.txt");
// }

AStar::Node::Node(Vec2i coordinates_, Node *parent_)
{
	parent = parent_;
	coordinates = coordinates_;
	Cost_G = 0;
	Cost_H = 0;
	Cost_F = 0;
}

void AStar::A_Star::set_conflag(bool connected_flag_)
{
	connected_flag = connected_flag_;
}

void AStar::A_Star::set_disflag(bool distance_flag_)
{
	distance_flag = distance_flag_;
}

void AStar::A_Star::set_x_connected()
{
	if (connected_flag == true) 
	{ 
		// 8-connected vector      		
        x_connected = { { 0, 1 }, { 1, 0 }, 
						{ 0, -1 }, { -1, 0 },
						{ -1, -1 }, { 1, 1 }, 
						{ -1, 1 }, { 1, -1 } };;           
	}
	else
		// 4-connected
	{
		x_connected = { { 0, 1 }, { 1, 0 }, 
						{ 0, -1 }, { -1, 0 } };       		
	}
}

bool AStar::A_Star::isValid(Vec2i coordinates_)
{
	if (coordinates_.x-1 < 0 || coordinates_.x-1 >= map[0].size() ||
		coordinates_.y-1 < 0 || coordinates_.y-1 >= map.size() || 
		map[coordinates_.x-1][coordinates_.y-1] == 1) 
	{
		return false;
	}
	return true;
}

bool AStar::A_Star::isGoal(Vec2i source_, Vec2i goal_)
{
	if (source_.x == goal_.x && source_.y == goal_.y) 
	{
		return true;
	}
	return false;
}

double AStar::A_Star::HScore_manhattan(Vec2i source_, Vec2i goal_)
{
	double Hcost = abs(source_.x - goal_.x) + abs(source_.y - goal_.y);
	return Hcost;
}

double AStar::A_Star::HScore_euclidean(Vec2i source_, Vec2i goal_)
{
	double Hcost = sqrt(pow(source_.x - goal_.x, 2) + pow(source_.y - goal_.y, 2));
	return Hcost; 
}

void AStar::A_Star::Generate_map(std::string path_)
{	
	std::ifstream file(path_.c_str());
	if (file)
	{
		std::string line;
		while (std::getline(file, line))
		{
			std::vector<int> row;
			char element;
			std::istringstream ss(line);
			while ( ss >> element )
			{
				row.push_back(element - '0');
			}
			map.push_back(row);
		}		
	}
	file.close();
}

void AStar::A_Star::findPath(Vec2i source_, Vec2i goal_)
{	
	Node* current = NULL;
	std::set<Node*> PathSet, VisitedSet;
	if (isValid(source_) == false || isValid(goal_) == false) 
	{	
		std::cout << "The start pos or goal pos is not valid." << std::endl;
		return;
	}
	PathSet.insert(new Node(source_));
	while (!PathSet.empty())
	{
		current = *PathSet.begin();
		// find the node with minimum Cost_F value
		for (auto node:PathSet)
		{
			if (node->Cost_F <= current->Cost_F) 
			{
				current = node;
			}
		}
		// check if current node is already a goal node
		if (isGoal(current->coordinates, goal_))
		{
			break;
		}

		// add to VisitedSet
		VisitedSet.insert(current);

		// delete current node from PathSet
		PathSet.erase(std::find(PathSet.begin(), PathSet.end(), current));

		// go through all the neighbor
		for (int i=0; i<x_connected.size(); i++) 
		{	
			Vec2i newCoordinates = {current->coordinates.x + x_connected[i][0], current->coordinates.y + x_connected[i][1]};
			// check if newCoordinates is valid or vesited
			if (isValid(newCoordinates) == false || findNode(VisitedSet, newCoordinates) != NULL) 
			{
				continue;
			}
			double newGcost;
			if (i >= 4)
			{
				newGcost = current->Cost_G + sqrt(2);
			}
			else 
			{
				newGcost = current->Cost_G + 1;
			}
			Node *successor = findNode(PathSet, newCoordinates);
			if (successor == NULL) {
				successor = new Node(newCoordinates, current);
				successor->Cost_G = newGcost;
				if (distance_flag == true)
				{
					successor->Cost_H = HScore_manhattan(successor->coordinates, goal_);
				}
				else 
				{
					successor->Cost_H = HScore_euclidean(successor->coordinates, goal_);
				}
				successor->Cost_F = successor->Cost_G + successor->Cost_H;
				PathSet.insert(successor);
			}
			else if (newGcost < successor->Cost_G) {
				successor->parent = current;
				successor->Cost_G = newGcost;
				successor->Cost_F = successor->Cost_G + successor->Cost_H;
			}
		}
	}
	while (current != NULL) 
	{
		path.push_back(current->coordinates);
		current = current->parent;
	}
	reverse(path.begin(), path.end());
	if (!path.empty()) 
	{
		std::cout << "The valid path: ";
		for (auto ele:path)
		{
			std::cout << "[" << ele.x << "," << ele.y << "] "; 
		}
		std::cout << "\n";
	}
	else
	{
		std::cout << "No valid path." << std::endl;
	}
	releaseNodes(PathSet);
	releaseNodes(VisitedSet);
}

AStar::Node* AStar::A_Star::findNode(std::set<Node*>& nodes_, Vec2i coordinates_)
{
	for (auto node:nodes_) {
		if (node->coordinates.x == coordinates_.x && node->coordinates.y == coordinates_.y) {
			return node;
		}
	}
	return NULL;
}

void AStar::A_Star::releaseNodes(std::set<Node*>& nodes_)
{
	for (auto it = nodes_.begin(); it != nodes_.end();) {
		delete *it;
		it = nodes_.erase(it);
	}
}



int main()
{
	AStar::A_Star temp;
	temp.Generate_map("easy.txt");
	// std::cout << temp.map[4][0] << temp.map[4][1] << temp.map[4][2] << temp.map[4][3] << temp.map[4][4] << std::endl;
	AStar::Vec2i start, goal;
	start.x = 3;
	start.y = 2;
	goal.x = 3;
	goal.y = 9;
	temp.set_conflag(0);
	temp.set_x_connected();
	temp.findPath(start, goal);
}