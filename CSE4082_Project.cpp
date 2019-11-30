#include "CSE4082_Project.h"

// Calculating the Manhattan Distance Heuristic.
int CalculateHeuristic(Node* current_node) {

	if (current_node->type == 'G')
		return 0;

	int min_manhattan_distance = 9999999;

	for (std::size_t i = 0; i < GoalNodes.size(); i++)
	{
		int cur_distance = abs(GoalNodes[i]->x - current_node->x) + abs(GoalNodes[i]->y - current_node->y);
		if (cur_distance < min_manhattan_distance) {
			min_manhattan_distance = cur_distance;
		}
	}
	return min_manhattan_distance;
}

// Parses each word from the input file to form a NodeFeatures struct.
struct NodeFeatures* ParseNodeFeatures(std::string word, int x, int y) {
	int cost = NULL;
	char type = NULL;

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
	int depth = 0;
	struct NodeFeatures* node_feats = new NodeFeatures{ cost, x, y, type, west, north, east, south, explored, frontiered, depth };
	return node_feats;
}

// Searches for actions that can be taken by a cell.
// Populates the frontier vector as well as finding the goal state if it exists in one of the actions(children).
Node* ActionSpace(Node* current_node, std::vector<Node*> &frontier, std::vector<std::vector<Node*>> StateMatrix, SearchAlgorithm search_algorithm, bool visualize_vector, int depth_limit)
{
	std::vector<Node*> action_vector;
	int x = current_node->x;
	int y = current_node->y;
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
	switch (search_algorithm)
	{
	case BFS:
	{
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
		if (visualize_vector)
			VisualizeVector(frontier);
	}
	break;
	case DFS:
	{

		std::reverse(action_vector.begin(), action_vector.end());
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

		if (visualize_vector)
			VisualizeVector(frontier);
	}
	break;
	case IDS:
	{

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
		if (visualize_vector)
			VisualizeVector(frontier);
	}
	break;
	case ASTAR:
	{
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
		std::sort(frontier.begin(), frontier.end(), CompareTwoNodesTotalCosts);
		if (visualize_vector)
			VisualizeVector(frontier);
	}
	break;
	case GBFS:
	{
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
		std::sort(frontier.begin(), frontier.end(), CompareTwoNodesHeuristics);
		if (visualize_vector)
			VisualizeVector(frontier);
	}
	break;
	case DLS:
	{
		std::reverse(action_vector.begin(), action_vector.end());
		for (Node* cur_child : action_vector)
		{
			if (!cur_child->explored && !cur_child->frontiered) {
				cur_child->parent = current_node;
				cur_child->frontiered = true;
				frontier.emplace_back(cur_child);
			}
		}
		if (visualize_vector)
			VisualizeVector(frontier);
	}
	break;
	case UCS:
	{
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
		if (visualize_vector)
			VisualizeVector(frontier);
	}
	break;
	default:
		break;
	}
	return NULL;
}

// Compares two nodes with their current path costs.
bool CompareTwoNodesCosts(Node* n1, Node* n2) {
	return CurrentPathCost(n1) < CurrentPathCost(n2);
}

// Compares two nodes with their heuristic path costs.
bool CompareTwoNodesHeuristics(Node* n1, Node* n2) {
	return CalculateHeuristic(n1) < CalculateHeuristic(n2);
}

// Compares two nodes with their (heuristic + current) path costs.
bool CompareTwoNodesTotalCosts(Node* n1, Node* n2) {
	return (CurrentPathCost(n1) + CalculateHeuristic(n1)) < (CurrentPathCost(n2) + CalculateHeuristic(n2));
}

// Left shift overloading for displaying a node with standard library.
std::ostream& operator << (std::ostream& out, Node* c)
{
	out << "| cost: " << c->cost;
	out << " | x, y: " << "(" << c->x + 1 << ", " << c->y + 1 << ")";
	out << " | type: " << c->type;
	out << " | explored: " << c->explored;
	out << " | frontiered: " << c->frontiered;
	return out;
}

// Returns the path that is starting from leaf/child node to its greates parent and displays the information of each travelled node.
// Also displays the total cost of the path.
void ReturnPath(Node* end_node) {
	int total_cost = 0;
	Node* current_printed_node;
	current_printed_node = end_node;
	std::cout << "Path Found" << std::endl;
	while (current_printed_node != NULL) {

		if (current_printed_node->type != 'S')
			total_cost += current_printed_node->cost;
		std::cout << current_printed_node << std::endl;
		current_printed_node = current_printed_node->parent;
	}
	std::cout << "Total Cost: " << total_cost << std::endl;
}

// Visualizes/Displays the vector of Node* kind.
void VisualizeVector(std::vector<Node*>& frontier) {

	std::cout << "----------\n";
	for (auto cur_frontier : frontier) {

		std::cout << "(" << cur_frontier->x + 1 << ", " << cur_frontier->y + 1 << ")\n";
	}
	std::cout << "-----------\n";
}

// Execution of DFS
Node* ExecuteDFS(std::vector<std::vector<Node*>> StateMatrix, std::vector<Node*> frontier, std::vector<Node*> explored, bool visualize_vector)
{
	int i = 0;
	std::cout << "DFS is selected." << std::endl;
	while (true)
	{
		// If frontier is empty, there is something wrong.
		if (frontier.empty()) {
			std::cout << "Failure... Path didn't found.\n";
			return NULL;
		}
		// Get the current node from the last element of frontier.
		Node* current_node;
		current_node = frontier[frontier.size() - 1];
		current_node->frontiered = false;

		// Erase the last element, pop it.
		frontier.erase(frontier.end() - 1);

		// Add the current node to the explored set.
		explored.emplace_back(current_node);
		current_node->explored = true;

		// Send the current node to the ActionSpace, collect if it finds the goal node in this depth.
		Node* result_child = ActionSpace(current_node, frontier, StateMatrix, DFS, visualize_vector);
		if (result_child) {
			std::cout << "\nExplored Set:\n";
			VisualizeVector(explored);
			return result_child;
		}
	}
	
}
// Execution of BFS
Node* ExecuteBFS(std::vector<std::vector<Node*>> StateMatrix, std::vector<Node*> frontier, std::vector<Node*> explored, bool visualize_vector)
{
	int i = 0;
	std::cout << "BFS is selected." << std::endl;
	while (true)
	{
		// If frontier is empty, there is something wrong.
		if (frontier.empty()) {
			std::cout << "Failure... Path didn't found.\n";
			return NULL;
		}
		// Get the current node from the first element of frontier.
		Node* current_node;
		current_node = frontier[0];
		current_node->frontiered = false;
		// Erase the first element, pop it.
		frontier.erase(frontier.begin());

		// Add the current node to the explored set.
		explored.emplace_back(current_node);
		current_node->explored = true;

		// Send the current node to the ActionSpace, collect if it finds the goal node in this depth.
		Node* result_child = ActionSpace(current_node, frontier, StateMatrix, BFS, visualize_vector);
		if (result_child) {
			std::cout << "\nExplored Set:\n";
			VisualizeVector(explored);
			return result_child;
		}
	}	
}

// Calculates the current path cost starting from the node to its greatest parent.
int CurrentPathCost(Node* current_node) {

	int total_cost = 0;
	while (current_node)
	{
		total_cost += current_node->cost;
		current_node = current_node->parent;
	}
	return total_cost;
}
// Execution of UCS
Node* ExecuteUCS(std::vector<std::vector<Node*>> StateMatrix, std::vector<Node*> frontier, std::vector<Node*> explored, bool visualize_vector)
{

	std::cout << "UCS is selected." << std::endl;
	while (true)
	{
		// If frontier is empty, there is something wrong.
		if (frontier.empty()) {
			std::cout << "Failure... Path didn't found.\n";
			return NULL;
		}

		// Get the current node from the first element of frontier.
		Node* current_node;
		current_node = frontier[0];
		current_node->frontiered = false;
		// Erase the first element, pop it.
		frontier.erase(frontier.begin());

		// Add the current node to the explored set.
		explored.emplace_back(current_node);
		current_node->explored = true;

		// Send the current node to the ActionSpace, collect if it finds the goal node in this depth.
		Node* result_child = ActionSpace(current_node, frontier, StateMatrix, UCS, visualize_vector);
		if (result_child) {
			std::cout << "\nExplored Set:\n";
			VisualizeVector(explored);
			return result_child;
		}
	}
}
// Execution of GBFS
Node* ExecuteGBFS(std::vector<std::vector<Node*>> StateMatrix, std::vector<Node*> frontier, std::vector<Node*> explored, bool visualize_vector) {

	std::cout << "GBFS is selected." << std::endl;
	while (true)
	{
		// If frontier is empty, there is something wrong.
		if (frontier.empty()) {
			std::cout << "Failure... Path didn't found.\n";
			return NULL;
		}
		// Get the current node from the first element of frontier.
		Node* current_node;
		current_node = frontier[0];
		current_node->frontiered = false;
		
		// Erase the first element, pop it.
		frontier.erase(frontier.begin());

		// Add the current node to the explored set.
		explored.emplace_back(current_node);
		current_node->explored = true;

		// Send the current node to the ActionSpace, collect if it finds the goal node in this depth.
		Node* result_child = ActionSpace(current_node, frontier, StateMatrix, GBFS, visualize_vector);
		std::cout << "\nExplored Set:\n";
		VisualizeVector(explored);
		if (result_child) {
			return result_child;
		}
	}
}
// Execution of ASTAR
Node* ExecuteASTAR(std::vector<std::vector<Node*>> StateMatrix, std::vector<Node*> frontier, std::vector<Node*> explored, bool visualize_vector) {

	std::cout << "ASTAR is selected." << std::endl;
	while (true)
	{
		// If frontier is empty, there is something wrong.
		if (frontier.empty()) {
			std::cout << "Failure... Path didn't found.\n";
			return NULL;
		}
		// Get the current node from the first element of frontier.
		Node* current_node;
		current_node = frontier[0];
		current_node->frontiered = false;

		// Erase the first element, pop it.
		frontier.erase(frontier.begin());

		// Add the current node to the explored set.
		explored.emplace_back(current_node);
		current_node->explored = true;

		// Send the current node to the ActionSpace, collect if it finds the goal node in this depth.
		Node* result_child = ActionSpace(current_node, frontier, StateMatrix, ASTAR, visualize_vector);
		std::cout << "\nExplored Set:\n";
		VisualizeVector(explored);
		if (result_child) {
			return result_child;
		}
	}
}
// Execution of IDS
Node* ExecuteIDS(std::vector<std::vector<Node*>> StateMatrix, std::vector<Node*> frontier, std::vector<Node*> explored, bool visualize_vector)
{
	//Initialize the depth as 0.
	int depth = 0;
	std::cout << "IDS is selected." << std::endl;
	while (true)
	{
		std::cout << "Trying for limit: " << depth << std::endl;
		//Set result as the output of recursive function.
		Node* result = ExecuteDLS(StateMatrix, frontier, explored, visualize_vector, depth);
		depth++;
		if (result) {
			std::cout << "Explored set:" << std::endl;
			//Prints the final true ordered expanded queue.
			VisualizeVector(GlobalExplored);
			return result;
		}
		//In the recursive part, it never sets the explored bool of nodes to false so we need to set it all to false here for further expansions.
		for (int i = 0; i < StateMatrix.size(); i++)
		{
			for (int j = 0; j < StateMatrix[i].size(); j++)
			{
				StateMatrix[i][j]->explored = false;
			}
		}
		//If there is a limit increase again, it clears the current explored set.
		GlobalExplored.clear();
	}
}
// Recursion state on IDS, uses DLS.
Node* RecursiveDLS(std::vector<std::vector<Node*>> StateMatrix, std::vector<Node*> frontier, std::vector<Node*> explored, bool visualize_vector, int limit)
{
	Node* result;
	if (frontier.empty()) return NULL;
	Node* current_node;
	current_node = frontier[frontier.size() - 1];
	current_node->frontiered = false;
	frontier.erase(frontier.end() - 1);
	std::cout << "We are at node: " << current_node->x + 1 << ", " << current_node->y + 1 << std::endl;
	explored.emplace_back(current_node);
	current_node->explored = true;
	//When the node is extracted from the frontier list we add it to global expanded list.
	GlobalExplored.emplace_back(current_node);
	if (current_node->type == 'G') return current_node;
	else if (limit == 0) {
		//If the function is on the leaf node for the given depthh limit, it returns null.
		std::cout << "Limit reached." << "and the node was: " << current_node->x + 1 << ", " << current_node->y + 1 << std::endl;
		return NULL;
	}
	else {
		//Gets the child nodes of the current node.
		Node* result_child = ActionSpace(current_node, frontier, StateMatrix, DLS, visualize_vector);
		for (Node* cur_child : frontier)
		{
			//for each node in the action space list, it executes the recursive function again.
			std::vector<Node*> frontierRec;
			std::vector<Node*> exploredRec;
			frontierRec.emplace_back(frontier[frontier.size() - 1]);
			//Need to erase the last node from the list, so it can check the node before it at the next iteration.
			frontier.erase(frontier.end() - 1);
			result = RecursiveDLS(StateMatrix, frontierRec, exploredRec, visualize_vector, limit - 1);
			//If a goal node is reached, returns the node.
			if (result) return result;
		}
		//If the currenct frontier list of the node is empty, it means that we need to go up in the tree.
		if (frontier.size() == 0) std::cout << "Returning back to upper depth." << std::endl;
		return NULL;
	}

}
// Execution of DLS for IDS.
Node* ExecuteDLS(std::vector<std::vector<Node*>> StateMatrix, std::vector<Node*> frontier, std::vector<Node*> explored, bool visualize_vector, int limit)
{
	return  RecursiveDLS(StateMatrix, frontier, explored, visualize_vector, limit);
}
int main(int argc, char* argv[])
{
	// Create an input file stream.
	std::ifstream inFile;

	// Open the input file.
	inFile.open("maze.txt");
	if (!inFile) {
		std::cout << "Unable to open Maze File";
		exit(1);
	}

	if (!inFile.is_open()) return 0;

	int selection;
	std::vector< std::vector< Node* > > StateMatrix;
	while (true)
	{
		std::cout << "\n >>>>>>>>>>>";
		std::cout << "\n Menu";
		std::cout << "\n ========";
		std::cout << "\n BFS - 1";
		std::cout << "\n UCS - 2";
		std::cout << "\n DFS - 3";
		std::cout << "\n IDS - 4";
		std::cout << "\n GBFS - 5";
		std::cout << "\n ASTAR - 6";
		std::cout << "\n Exit - 0";
		std::cout << "\n Enter selection: ";

		std::cin >> selection;
		inFile.clear();
		inFile.seekg(0, std::ios::beg);

		int idx = 0;
		std::string word;

		StateMatrix.clear();
		StateMatrix.resize(WIDTH, std::vector<Node*>(HEIGHT));
		int start_row = NULL, start_column = NULL;

		// Read every word in input file.
		while (inFile >> word)
		{
			// Assign current row and column.
			int row = idx / HEIGHT;
			int column = idx % WIDTH;

			//Parse node features and create a new node.
			auto new_node_features = ParseNodeFeatures(word, row, column);
			Node* new_node = new Node(new_node_features);

			// Populate StateMatrix.
			StateMatrix[row][column] = new_node;
			idx++;

			// Get start column and row and populate GoalNodes.
			if (new_node_features->type == 'S') {
				start_column = column;
				start_row = row;
			}
			else if (new_node_features->type == 'G') {
				GoalNodes.emplace_back(new_node);
			}
		}

		if (StateMatrix[start_row][start_column]->type == 'G') {
			ReturnPath(StateMatrix[start_row][start_column]);
			return 1;
		}
		std::vector<Node*> frontier;
		std::vector<Node*> explored;

		StateMatrix[start_row][start_column]->frontiered = true;
		frontier.emplace_back((StateMatrix[start_row][start_column]));

		switch (selection) {
		case 1:
		{
			Node* result = ExecuteBFS(StateMatrix, frontier, explored, true);
			if (result) {
				ReturnPath(result);
			}
		}
		break;
		case 2:
		{
			Node* result = ExecuteUCS(StateMatrix, frontier, explored, true);
			if (result) {
				ReturnPath(result);
			}
		}
		break;
		case 3:
		{
			Node* result = ExecuteDFS(StateMatrix, frontier, explored, true);
			if (result) {
				ReturnPath(result);
			}
		}
		break;
		case 4:
		{
			Node* result = ExecuteIDS(StateMatrix, frontier, explored, true);
			if (result) {
				ReturnPath(result);
			}
		}
		break;
		case 5:
		{
			Node* result = ExecuteGBFS(StateMatrix, frontier, explored, true);
			if (result) {
				ReturnPath(result);
			}
		}
		break;
		case 6:
		{
			Node* result = ExecuteASTAR(StateMatrix, frontier, explored, true);
			if (result) {
				ReturnPath(result);
			}
		}
		break;
		case 0:
			return 0;
			break;
		default:
		{
			std::cout << "\n !!! Invalid selection \n";
		}
		}
		GlobalExplored.clear();
	}
	return 0;
}
