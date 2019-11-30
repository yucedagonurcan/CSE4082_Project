#pragma once

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <algorithm>

#define WIDTH 8
#define HEIGHT 8


struct NodeFeatures {
	int cost;
	int x, y;
	char type;
	short west, north, east, south;
	bool explored, frontiered;
	int depth;
};
enum SearchAlgorithm { BFS, DFS, IDS, UCS, ASTAR, GBFS, DLS};
class Node {
public:
	int cost;
	int x, y;
	Node* parent;
	char type;
	short west, north, east, south;
	bool explored;
	bool frontiered;
	int depth;
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
		frontiered(frontiered),
		depth(depth)
	{}
	~Node() {}
};
std::vector<Node*> GoalNodes;
int CurrentPathCost(Node* current_node);
bool CompareTwoNodesCosts(Node* n1, Node* n2);
bool CompareTwoNodesHeuristics(Node* n1, Node* n2);
bool CompareTwoNodesTotalCosts(Node* n1, Node* n2);
struct NodeFeatures* ParseNodeFeatures(std::string word, int x, int y);
Node* ActionSpace(Node* current_node, std::vector<Node*> &frontier,
	std::vector<std::vector<Node*>> StateMatrix,
	SearchAlgorithm search_algorithm, bool visualize_frontier, int depth_limit=NULL);
std::ostream& operator << (std::ostream& out, Node* c);
void ReturnPath(Node* end_node);
void VisualizeFrontier(std::vector<Node*> frontier, int depth);
Node* ExecuteBFS(std::vector<std::vector<Node*>> StateMatrix,std::vector<Node*> frontier,std::vector<Node*> explored, bool visualize_frontier);
Node* ExecuteUCS(std::vector<std::vector<Node*>> StateMatrix,std::vector<Node*> frontier,std::vector<Node*> explored, bool visualize_frontier);
Node* ExecuteDFS(std::vector<std::vector<Node*>> StateMatrix,std::vector<Node*> frontier,std::vector<Node*> explored, bool visualize_frontier);
Node* ExecuteIDS(std::vector<std::vector<Node*>> StateMatrix,std::vector<Node*> frontier,std::vector<Node*> explored, bool visualize_frontier, int start_row, int start_column);
Node* ExecuteDLS(std::vector<std::vector<Node*>> StateMatrix,std::vector<Node*> frontier,std::vector<Node*> explored, bool visualize_frontier, int depth);
Node* ExecuteGBFS(std::vector<std::vector<Node*>> StateMatrix,std::vector<Node*> frontier,std::vector<Node*> explored, bool visualize_frontier);
Node* ExecuteASTAR(std::vector<std::vector<Node*>> StateMatrix,std::vector<Node*> frontier,std::vector<Node*> explored, bool visualize_frontier);