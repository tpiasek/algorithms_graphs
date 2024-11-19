#ifndef UTILITIES_
#define UTILITIES_


#include <iostream>
#include <conio.h>
#include <fstream>
#include <sstream>

// Pomiar czasu
#include <ctime>
#include <chrono>

#include "graphs/adjacency_list_graph.hpp"
#include "graphs/adjacency_matrix_graph.hpp"

using namespace std;


vector<SP_Node*> readSPResults(string path) {
    ifstream file(path);

    if (!file.is_open()) {
		std::cerr << "Error: file not opened" << std::endl;
		return vector<SP_Node*>();
	}

	vector<SP_Node*> results;
	string line;
    while (getline(file, line)) {
		istringstream iss(line);
		int v_end, path_length;
		vector<Vertex*> path;
		iss >> v_end >> path_length;
		int v_id;
        while (iss >> v_id) {
			path.push_back(new Vertex(v_id));
		}
		results.push_back({ new SP_Node(new Vertex(v_end), path_length, path) });
	}

	return results;
}

bool checkSPResults(vector<SP_Node*> results, vector<SP_Node*> compare) {
	if (results.size() != compare.size()) {
		cerr << "Error: different number of results [checkSPResults]!" << endl;
		return false;
	}

	bool the_same = true;

	for (int i = 0; i < results.size(); i++) {
		the_same = false;

		for (int j = 0; j < compare.size(); j++) {
			the_same = false;
			if (results[i]->v_end->id == compare[j]->v_end->id) {
				the_same = true;
				if (results[i]->dist == compare[j]->dist && results[i]->path.size() == compare[j]->path.size()) {
					for (int k = 0; k < results[i]->path.size(); k++) {
						if (results[i]->path[k]->id != compare[j]->path[k]->id) {
							the_same = false;
							break;
						}
					}
					if (!the_same) {
						cerr << "Error: paths are different! [v_end: " << results[i]->v_end->id << "]" << endl;
						cerr << "Path 1: ";
						for (Vertex* v : results[i]->path) {
							cerr << v->id << " ";
						}
						cerr << "\nPath 2: ";
						for (Vertex* v : compare[j]->path) {
							cerr << v->id << " ";
						}
						return false;
					}
					break;
				}
				else {
					cerr << "Error: distance or path sizes are different! [v_end: " << results[i]->v_end->id << "]" << endl;
					cerr << "Distance 1: " << results[i]->dist << "\nDistance 2: " << compare[j]->dist << endl;
					cerr << "Path 1 size: " << results[i]->path.size() << "\nPath 2 size: " << compare[j]->path.size() << endl;
					return false;
				}
				break;
			}
		}
		if (!the_same) {
			cerr << "Error: vertex with the same ID not found! [v_end: " << results[i]->v_end->id << "]" << endl;
			return false;
		}
	}

	return true;
}

std::pair<long long int, std::vector<SP_Node*>> measureTime(std::unique_ptr<Graph>& graph, int idStart, std::string algorithm) {
	std::chrono::steady_clock::time_point start, end;
	std::vector<SP_Node*> paths;
	start = std::chrono::high_resolution_clock::now();
	if (algorithm == "Dijkstra") {
		paths = graph->spDijkstra(idStart);
	}
	else if (algorithm == "BellmanFord") {
		paths = graph->spBellmanFord(idStart);
	}
	else {
		cerr << "Error: unknown algorithm!" << endl;
		return std::pair(-1, std::vector<SP_Node*>());
	}
	end = std::chrono::high_resolution_clock::now();
	auto time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

	/*if (checkDijkstra(paths, results)) cout << algorithm << " results are the same!" << endl;
	else cout << algorithm << " results are different!" << endl;*/

	return std::pair(time, paths);
}

std::pair<int, long long int> test(std::unique_ptr<Graph>& graph, vector<SP_Node*> compare, std::string algorithm, int noTests = 10) {
	int idStart = graph->getSPVertexId();
	if (idStart == -1) {
		cerr << "Error: no starting vertex!" << endl;
		return std::pair(-1, -1);
	}
	std::chrono::steady_clock::time_point start, end;
	std::vector<SP_Node*> paths;
	long long int time = 0;
	int fails = 0;

	for (int i = 0; i < noTests; i++) {
		system("cls");
		std::cout << "Test " << i + 1 << "/" << noTests << std::endl;
		std::cout << "Time left: " << (noTests - i) * time / 1000000 / 60 << ":" << (noTests - i) * time / 1000000 % 60 << "min\n\n";

		start = std::chrono::high_resolution_clock::now();
		if (algorithm == "Dijkstra") {
			paths = graph->spDijkstra(idStart);
		}
		else if (algorithm == "BellmanFord") {
			paths = graph->spBellmanFord(idStart);
		}
		else {
			cerr << "Error: unknown algorithm!" << endl;
			return std::pair(-1, -1);
		}
		end = std::chrono::high_resolution_clock::now();

		if (paths.empty() || !checkSPResults(paths, compare)) {
			fails++;
		}

		if (i > 0) {
			time *= i;
			time += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
			time /= i + 1;
		}
		else {
			time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
			std::cout << "Initial duration: " << time << std::endl;
		}
	}

	std::cout << noTests - fails << "/" << noTests << " tests completed successfully" << std::endl;

	return std::pair(noTests - fails, time);
}


#endif // !UTILITIES_