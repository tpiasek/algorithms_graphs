#include <iostream>
#include <conio.h>
#include <fstream>
#include <sstream>
#include "utilities.hpp"

#include "graphs/adjacency_list_graph.hpp"
#include "graphs/adjacency_matrix_graph.hpp"

using namespace std;


int menu()
{
    while (1) {
        system("cls");
        cout << "Enter file name: ";
        string file_name;
        cin >> file_name;
        ifstream file("C:\\Users\\tompi\\source\\repos\\PiAAGrafy\\sp_data\\graph\\" + file_name + ".txt");

        if (!file.is_open()) {
			std::cerr << "Error: file not opened" << std::endl;
			cout << "Error: file not opened. Try again." << endl;
            break;
		}

        std::unique_ptr<Graph> graph;

        while (1) {
            file.clear();
            file.seekg(0, ios::beg);

            cout << "Choose implementation: " << endl;
            cout << "1. Adjacency List" << endl;
            cout << "2. Adjacency Matrix" << endl;
            cout << "3. Exit" << endl;

            char choice;
            choice = _getch();

            switch (choice) {
            case '1': {
				graph = AdjacencyListGraph::createGraph(file);
				break;
			}
            case '2': {
				graph = AdjacencyMatrixGraph::createGraph(file);
				break;
			}
            case '3': {
				return 0;
			}
            default: {
				cout << "Invalid choice. Try again." << endl;
				break;
			}
			}

            cout << "Graph created!" << endl << endl;

            cout << "Choose algorithm: " << endl;
            cout << "1. Dijkstra" << endl;
            cout << "2. Bellman-Ford" << endl;
            cout << "3. Exit" << endl;

            choice = _getch();

            string algorithm;
            vector<SP_Node*> paths;

            switch (choice) {
            case '1': 
                paths = graph->spDijkstra(graph->getSPVertexId());
                break;
            case '2': 
                paths = graph->spBellmanFord(graph->getSPVertexId());
                break;
            case '3':
                return 0;
            }

            system("cls");
            cout << graph->no_vertices << " " << graph->no_edges << endl;
            for (SP_Node* node : paths) {
				cout << node->v_end->id << " " << node->dist << " ";
                for (Vertex* v : node->path) {
					cout << v->id << " ";
				}
                cout << endl;
			}
            if(graph->getSPVertexId() == -1)
				cout << "Graph has no starting vertex for shortest path algorithm." << endl;
			else
				cout << graph->getSPVertexId() << endl;

            _getch();
            break;
        }
    }
    
    return 0;
}
