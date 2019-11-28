#pragma once

#include <iostream>
#include <string>
#include <fstream>
#include <vector>

#define WIDTH 8
#define HEIGHT 8

struct NodeFeatures {
	int cost;
	int x, y;
	char type;
	short west, north, east, south;
	bool explored, frontiered;
};
enum SearchAlgorithm { BFS, DFS, IDS, ASTAR, GBFS };
class Node {
public:
	int cost;
	int x, y;
	Node* parent;
	char type;
	short west, north, east, south;
	bool explored;
	bool frontiered;
	Node(struct NodeFeatures* nodefeats)
		: x(nodefeats->x),
		y(nodefeats->y),
		west(nodefeats->west),
		north(nodefeats->north),
		east(nodefeats->east),
		south(nodefeats->south),
		cost(nodefeats->cost),
		type(nodefeats->type),
		explored(nodefeats->explored),
		frontiered(frontiered)
	{}
	~Node() {}
};
struct NodeFeatures* ParseNodeFeatures(std::string word, int x, int y);
std::vector<Node*> ActionSpace(Node* current_node, std::vector<std::vector<Node*>> StateMatrix);
std::ostream& operator << (std::ostream& out, Node* c);
void ReturnPath(Node* end_node);