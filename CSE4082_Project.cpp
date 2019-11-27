#include <iostream>
#include <string>
#include <iomanip>
#include <fstream>
#include <sstream>
#include<tuple> // for tuple 
#include <vector>


#define WIDTH 8
#define HEIGHT 8
struct NodeFeatures {
	int cost;
	int x, y;
	char type;
	short west, north, east, south;
	bool visited;
};
enum SearchAlgorithm { BFS, DFS, IDS, ASTAR, GBFS };

class Node {
public:
	int cost;
	int x, y;
	Node* parent;
	char type;
	short west, north, east, south;
	bool visited;
	Node(struct NodeFeatures *nodefeats)
		: x(nodefeats->x), y(nodefeats->y), west(nodefeats->west), north(nodefeats->north), east(nodefeats->east), south(nodefeats->south), cost(nodefeats->cost), type(nodefeats->type), visited(nodefeats->visited)
	{}
	~Node() {}
};
struct NodeFeatures* ParseNodeFeatures(std::string word, int x, int y){
	int cost=NULL;
	char type=NULL;

	if (word[0] == '1' || word[0] == '7') {
		cost = word[0] - '0';
		type = word[0] == '1' ? 'N' : 'T';
	}
	else {
		cost = 1;
		type = word[0];
	}
	bool visited = false;
	short west = (short)(word[1] == '.');
	short north = (short)(word[2] == '.');
	short east = (short)(word[3] == '.');
	short south = (short)(word[4] == '.');

	struct NodeFeatures* node_feats = new NodeFeatures{ cost, x, y, type, west, north, east, south, visited };
	return node_feats;
}
std::ostream& operator << (std::ostream& out, Node *c)
{
	out << "|cost: " << c->cost;
	out << " |x, y: " << c->x << " " << c->y;
	out << " |type: " << c->type;
	out << " |west: " << c->west;
	out << " |north: " << c->north;
	out << " |east: " << c->east;
	out << " |south: " << c->south;
	out << " |visited: " << c->visited;

	out << std::endl;
	return out;
}
int main(int argc, char* argv[])
{
	char search_algorithm = 'B';
    std::ifstream inFile;
    
    inFile.open("maze.txt");
    if (!inFile) {
        std::cout << "Unable to open Maze File";
        exit(1);
    }
	
	if (!inFile.is_open()) return 0;

	int idx = 0;
	std::string word;
	std::vector< std::vector< Node* > > StateMatrix;

	StateMatrix.resize(WIDTH, std::vector<Node*>(HEIGHT));

	int start_row =NULL, start_column = NULL;

	while (inFile >> word)
	{
		int row = idx / HEIGHT;
		int column = idx % WIDTH;

		auto new_node_features = ParseNodeFeatures(word, row, column);
		Node* new_node = new Node(new_node_features);
		StateMatrix[row][column] = new_node;
		idx++;
		if (new_node_features->type == 'S') {
			start_column = column;
			start_row = row;
		}
	}
	SearchAlgorithm s = BFS;
	std::vector<Node*> frontier;
	frontier.emplace_back((StateMatrix[start_row][start_column]));
	switch (s) {
	case BFS:
		std::cout << "BFS is selected." << std::endl;

		break;
	case DFS:
		break;

	default:
		;
	}
	return 0;
}
