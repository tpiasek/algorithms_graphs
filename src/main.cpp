#include <iostream>
#include <conio.h>
#include <fstream>
#include <sstream>

#include "utilities.hpp"

#include "graphs/adjacency_list_graph.hpp"
#include "graphs/adjacency_matrix_graph.hpp"
#include "menu.hpp"

using namespace std;


int main(int argc, char* argv[])
{
    //menu();
    // std::cout<< "Tu wykonujemy testy efektywności algorytmów"<<std::endl;
    std::string graph_type = "V100D0.5";
    int noTests = 6000;
    std::string algorithm = "BellmanFord";

    ifstream file("C:\\Users\\tompi\\source\\repos\\PiAAGrafy\\sp_data\\graph\\graph" + graph_type + ".txt");

    if (!file.is_open()) {
		std::cerr << "Error: file not opened" << std::endl;
        cout << "Error: file not opened" << endl;
		return 1;
	}

    string path = "C:\\Users\\tompi\\source\\repos\\PiAAGrafy\\sp_data\\sp_result\\";
    string file_name = "sp" + graph_type + ".txt";
    vector<SP_Node*> results = readSPResults(path + file_name);


    std::unique_ptr<Graph> graph = AdjacencyListGraph::createGraph(file);
    file.clear();
    file.seekg(0, ios::beg);
    std::unique_ptr<Graph> graphM = AdjacencyMatrixGraph::createGraph(file);


    std::pair<int, long long int> testResult = test(graph, results, algorithm, noTests);
    cout << "AdjacencyListGraph " << algorithm << " test: " << testResult.first << " out of " << noTests << endl;
    cout << "AdjacencyListGraph " << algorithm << " time: " << testResult.second << " microseconds" << endl;


    return 0;
}
