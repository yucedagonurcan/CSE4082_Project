#include "CSE4082_Project.h"

struct NodeFeatures* ParseNodeFeatures(std::string word, int x, int y){
	int cost=NULL;
	char type=NULL;

	if (word[0] == '1' || word[0] == '7') {
		cost = word[0] - '0';
		type = word[0] == '1' ? 'N' : 'T';
	}
	else if (word[0] == 'S') {
		cost = 0;
		type = word[0];
	}
	else {
		cost = 1;
		type = word[0];
	}
	bool explored = false;
	bool frontiered = false;
	short west = (short)(word[1] == '.');
	short north = (short)(word[2] == '.');
	short east = (short)(word[3] == '.');
	short south = (short)(word[4] == '.');

	struct NodeFeatures* node_feats = new NodeFeatures{ cost, x, y, type, west, north, east, south, explored, frontiered };
	return node_feats;
}
std::vector<Node*> ActionSpace(Node* current_node,
								std::vector<std::vector<Node*>> StateMatrix,
								SearchAlgorithm search_algorithm) 
{


	std::vector<Node*> action_vector;
	int x = current_node->x;
	int y = current_node->y;

	switch (search_algorithm)
	{
	case BFS:
	{

		if (current_node->east) {
			action_vector.emplace_back(StateMatrix[x][y + 1]);
		}
		if (current_node->south) {

			action_vector.emplace_back(StateMatrix[x + 1][y]);
		}
		if (current_node->west) {

			action_vector.emplace_back(StateMatrix[x][y - 1]);
		}
		if (current_node->north) {

			action_vector.emplace_back(StateMatrix[x - 1][y]);
		}
	}
		break;
	case DFS:
		break;
	case IDS:
		break;
	case UCS:
	{

		if (current_node->east) {
			action_vector.emplace_back(StateMatrix[x][y + 1]);
		}
		if (current_node->south) {

			action_vector.emplace_back(StateMatrix[x + 1][y]);
		}
		if (current_node->west) {

			action_vector.emplace_back(StateMatrix[x][y - 1]);
		}
		if (current_node->north) {

			action_vector.emplace_back(StateMatrix[x - 1][y]);
		}
		
	}
		break;
	case ASTAR:
		break;
	case GBFS:
		break;
	default:
		break;
	}

	

	return action_vector;
}
bool CompareTwoNodesCosts(Node* n1, Node* n2) {
	return CurrentPathCost(n1) < CurrentPathCost(n2);
}

std::ostream& operator << (std::ostream& out, Node *c)
{
	out << "|cost: " << c->cost;
	out << " |x, y: " << "(" << c->x + 1 << ", " << c->y + 1 << ")";
	out << " |type: " << c->type;
	out << " |explored: " << c->explored;
	out << " |frontiered: " << c->frontiered;
	return out;
}
void ReturnPath(Node* end_node) {
	int total_cost = 0;
	Node* current_printed_node;
	current_printed_node = end_node;
	std::cout << "Path Found" << std::endl;
	while (current_printed_node != NULL) {

		if(current_printed_node->type != 'S')
		total_cost += current_printed_node->cost;
		std::cout << current_printed_node << std::endl;
		current_printed_node = current_printed_node->parent;
	}
	std::cout << "Total Cost: " << total_cost << std::endl;
}
void VisualizeFrontier(std::vector<Node*> frontier, int depth) {

	std::cout << "DEPTH: " << depth << "\n";
	for (auto cur_frontier : frontier) {

		std::cout << "(" << cur_frontier->x +1 << ", " << cur_frontier->y+1 << ")\n";
	}
	std::cout << "-----------\n";
}
Node* ExecuteBFS(std::vector<std::vector<Node*>> StateMatrix,
	std::vector<Node*> frontier,
	std::vector<Node*> explored,
	bool visualize_frontier)
{
	//SearchAlgorithmResult result = new SearchAlgorithmResult(){ false, NULL };
	int i = 0;
	std::cout << "BFS is selected." << std::endl;
	while (true)
	{
		if (frontier.empty()) {
			std::cout << "Failure... Path didn't found.\n";
			return NULL;
		}
		Node* current_node;
		current_node = frontier[0];
		current_node->frontiered = false;
		frontier.erase(frontier.begin());

		explored.emplace_back(current_node);
		current_node->explored = true;
		auto action_vector = ActionSpace(current_node, StateMatrix, BFS);
		for (Node* cur_child : action_vector)
		{
			if (!cur_child->explored && !cur_child->frontiered) {
				cur_child->parent = current_node;
				if (cur_child->type == 'G') {
					cur_child->explored = true;
					return cur_child;
				}
				cur_child->frontiered = true;
				frontier.emplace_back(cur_child);
			}
		}
		if (visualize_frontier)
			VisualizeFrontier(frontier, i);
		i++;
	}
}
int CurrentPathCost(Node* current_node) {

	int total_cost = 0;
	while (current_node)
	{
		total_cost += current_node->cost;
		current_node = current_node->parent;
	}
	return total_cost;
}

Node* ExecuteDFS(std::vector<std::vector<Node*>> StateMatrix,
	std::vector<Node*> frontier,
	std::vector<Node*> explored,
	bool visualize_frontier)
{
	//SearchAlgorithmResult result = new SearchAlgorithmResult(){ false, NULL };
	int i = 0;
	std::cout << "DFS is selected." << std::endl;
	while (true)
	{
		if (frontier.empty()) {
			std::cout << "Failure... Path didn't found.\n";
			return NULL;
		}
		Node* current_node;
		current_node = frontier[0];
		current_node->frontiered = false;
		frontier.erase(frontier.begin());

		explored.emplace_back(current_node);
		current_node->explored = true;
		auto action_vector = ActionSpace(current_node, StateMatrix, DFS);
		for (Node* cur_child : action_vector)
		{
			if (!cur_child->explored && !cur_child->frontiered) {
				cur_child->parent = current_node;
				if (cur_child->type == 'G') {
					cur_child->explored = true;
					return cur_child;
				}
				cur_child->frontiered = true;
				frontier.emplace_back(cur_child);
			}
		}
		if (visualize_frontier)
			VisualizeFrontier(frontier, i);
		i++;
	}
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
	SearchAlgorithm s = UCS;
	

	if (StateMatrix[start_row][start_column]->type == 'G') {
		ReturnPath(StateMatrix[start_row][start_column]);
		return 1;
	}
	std::vector<Node*> frontier;
	std::vector<Node*> explored;

	StateMatrix[start_row][start_column]->frontiered = true;
	frontier.emplace_back((StateMatrix[start_row][start_column]));
	
	switch (s) {
	case BFS:
	{
		Node* result = ExecuteBFS(StateMatrix, frontier, explored, false);
		if (result) {
			ReturnPath(result);
		}
	}
		break;
	case UCS:
	{
		Node* result = ExecuteUCS(StateMatrix, frontier, explored, true);
		if (result) {
			ReturnPath(result);
		}
	}
		break;
	case DFS:
	{
		Node* result = ExecuteDFS(StateMatrix, frontier, explored, true);
		if (result) {
			ReturnPath(result);
		}
	}
	break;

	default:
		;
	}
	return 0;
}

Node* ExecuteUCS(std::vector<std::vector<Node*>> StateMatrix,
	std::vector<Node*> frontier,
	std::vector<Node*> explored,
	bool visualize_frontier)
{
	//SearchAlgorithmResult result = new SearchAlgorithmResult(){ false, NULL };
	int i = 0;
	std::cout << "UCS is selected." << std::endl;
	while (true)
	{
		if (frontier.empty()) {
			std::cout << "Failure... Path didn't found.\n";
			return NULL;
		}
		Node* current_node;
		current_node = frontier[0];
		current_node->frontiered = false;
		frontier.erase(frontier.begin());

		explored.emplace_back(current_node);
		current_node->explored = true;
		auto action_vector = ActionSpace(current_node, StateMatrix, UCS);
		for (Node* cur_child : action_vector)
		{
			if (!cur_child->explored && !cur_child->frontiered) {
				cur_child->parent = current_node;
				if (cur_child->type == 'G') {
					cur_child->explored = true;
					return cur_child;
				}
				cur_child->frontiered = true;
				frontier.emplace_back(cur_child);
			}
		}
		std::sort(frontier.begin(), frontier.end(), CompareTwoNodesCosts);
		if (visualize_frontier)
			VisualizeFrontier(frontier, i);
		i++;
	}
}