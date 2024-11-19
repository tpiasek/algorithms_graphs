#include "graphs/adjacency_matrix_graph.hpp"


AdjacencyMatrixGraph::AdjacencyMatrixGraph(int no_vertices, int no_edges) : Graph(no_vertices, no_edges) {}

AdjacencyMatrixGraph::AdjacencyMatrixGraph(int no_vertices, int no_edges, std::vector<Vertex*> vertices, std::vector<std::vector<int>> adjacencyMatrix) : Graph(no_vertices, no_edges), vertices(vertices), adjacencyMatrix(adjacencyMatrix) {}

AdjacencyMatrixGraph::AdjacencyMatrixGraph(int no_vertices, int no_edges, std::vector<Vertex*> vertices, std::vector<std::vector<int>> adjacencyMatrix, Vertex* v_sp) : Graph(no_vertices, no_edges, v_sp), vertices(vertices), adjacencyMatrix(adjacencyMatrix) {}


std::unique_ptr<Graph> AdjacencyMatrixGraph::createGraph(std::istream& is) {
    int no_vertices = 0, no_edges = 0;
    is >> no_vertices >> no_edges;

    std::string line;
    int v1_val = 0, v2_val = 0, w = 0, v1_pos = 0, v2_pos = 0, v_sp_val = DEFAULT_WEIGHT;
    std::vector<Vertex*> vertices;
    std::vector<std::vector<int>> adjacencyMatrix(no_vertices, std::vector(no_vertices, DEFAULT_WEIGHT));
    bool found_v1 = false, found_v2 = false;
    Vertex* v1;
    Vertex* v2;
    Vertex* v_sp = nullptr;

    for (int i = 0; i < no_edges; i++) {
		is >> v1_val >> v2_val >> w;

        // Check if vertices with given id already exist (quicker than 2x findVertex)
        for (int i = 0; i < vertices.size(); i++) {
			Vertex* v = vertices[i];
            if (v->id == v1_val) {
				v1 = v;
				found_v1 = true;
				v1_pos = i;
			}
            if (v->id == v2_val) {
				v2 = v;
				found_v2 = true;
				v2_pos = i;
			}
		}

		// If not, create new vertices
        if (!found_v1) {
			v1 = new Vertex(v1_val);
			vertices.push_back(v1);
			v1_pos = vertices.size() - 1;
		}
		found_v1 = false;

        if (!found_v2) {
			v2 = new Vertex(v2_val);
			vertices.push_back(v2);
			v2_pos = vertices.size() - 1;
		}
		found_v2 = false;

		// Add edge to adjacency matrix
		adjacencyMatrix[v1_pos][v2_pos] = w;
    }

	// Check if starting vertex for SP is provided, if so, save its ID
	if (is >> v_sp_val) {
        v_sp = new Vertex(v_sp_val);
    }

    std::cout << "Graph created with " << no_vertices << " vertices and " << no_edges << " edges.\n";

    return std::make_unique<AdjacencyMatrixGraph>(no_vertices, no_edges, vertices, adjacencyMatrix, v_sp);
}

int AdjacencyMatrixGraph::addVertex(int id) {
	// Check if vertex with given id already exists
	int v_pos = findVertexPos(id);
	if (v_pos != -1) {
		std::cerr << "Vertex with ID " << id << " already exists [addVertex].\n";
		return -1;
	}

	// Create new vertex
	Vertex* v = new Vertex(id);
	vertices.push_back(v);

	// Add new row and column to adjacency matrix
	for (int i = 0; i < adjacencyMatrix.size(); i++) {
		adjacencyMatrix[i].push_back(DEFAULT_WEIGHT);
	}
	adjacencyMatrix.push_back(std::vector<int>(adjacencyMatrix.size() + 1, DEFAULT_WEIGHT));

	no_vertices++;

	return 0;
}

void AdjacencyMatrixGraph::updateVertex(int id, int v) {
	int v_pos = findVertexPos(id);

	if (v_pos == -1) {
		std::cerr << "Vertex with ID " << id << " does not exist [updateVertex].\n";
		return;
	}
	if (findVertexPos(v) != -1) {
		std::cerr << "Vertex with id " << v << " already exists! [updateVertex]" << std::endl;
		return;
	}

	vertices[v_pos]->id = v;
}

void AdjacencyMatrixGraph::removeVertex(int id) {
	int v_pos = findVertexPos(id);
	if (v_pos == -1) {
		std::cerr << "Vertex with ID " << id << " does not exist [removeVertex].\n";
		return;
	}

	// Remove vertex from vertices list
	vertices.erase(vertices.begin() + v_pos);

	// Remove row and column from adjacency matrix
	adjacencyMatrix.erase(adjacencyMatrix.begin() + v_pos);
	for (int i = 0; i < adjacencyMatrix.size(); i++) {
		adjacencyMatrix[i].erase(adjacencyMatrix[i].begin() + v_pos);
	}

	no_vertices--;
}

void AdjacencyMatrixGraph::addEdge(int idStart, int idEnd, int w) {
	int v1_pos = findVertexPos(idStart);
	int v2_pos = findVertexPos(idEnd);

	if (v1_pos == -1 || v2_pos == -1) {
		std::cerr << "Vertices with IDs " << idStart << " and " << idEnd << " do not exist [updateEdge].\n";
		return;
	}

	if (adjacencyMatrix[v1_pos][v2_pos] != DEFAULT_WEIGHT) {
		std::cerr << "Edge between vertices " << idStart << " and " << idEnd << " already exists [updateEdge].\n";
		return;
	}

	adjacencyMatrix[v1_pos][v2_pos] = w;
	no_edges++;
}

void AdjacencyMatrixGraph::updateEdge(int idStart, int idEnd, int w) {
	int v1_pos = findVertexPos(idStart);
	int v2_pos = findVertexPos(idEnd);

	if (v1_pos == -1 || v2_pos == -1) {
		std::cerr << "Vertices with IDs " << idStart << " and " << idEnd << " do not exist [updateEdge].\n";
		return;
	}

	if (adjacencyMatrix[v1_pos][v2_pos] == DEFAULT_WEIGHT) {
		std::cerr << "Edge between vertices " << idStart << " and " << idEnd << " does not exist. Creating edge with weight: " << w << " [updateEdge].\n";
	}

	adjacencyMatrix[v1_pos][v2_pos] = w;
}

void AdjacencyMatrixGraph::removeEdge(int idStart, int idEnd) {
	int v1_pos = findVertexPos(idStart);
	int v2_pos = findVertexPos(idEnd);

	if (v1_pos == -1 || v2_pos == -1) {
		std::cerr << "Vertices with IDs " << idStart << " and " << idEnd << " do not exist [removeEdge].\n";
		return;
	}

	if (adjacencyMatrix[v1_pos][v2_pos] == DEFAULT_WEIGHT) {
		std::cerr << "Edge between vertices " << idStart << " and " << idEnd << " does not exist [removeEdge].\n";
		return;
	}

	adjacencyMatrix[v1_pos][v2_pos] = DEFAULT_WEIGHT;
	no_edges--;
}

std::vector<Vertex*> AdjacencyMatrixGraph::getNeighbours(int id) {
	int v_pos = findVertexPos(id);
	std::vector<Vertex*> neighbours;

	if (v_pos == -1) {
		std::cerr << "Vertex with ID " << id << " does not exist [getNeighbours].\n";
		return neighbours;
	}

	for (int i = 0; i < adjacencyMatrix[v_pos].size(); i++) {
		if (adjacencyMatrix[v_pos][i] != DEFAULT_WEIGHT) {
			neighbours.push_back(vertices[i]);
		}
	}

	return neighbours;
}

bool AdjacencyMatrixGraph::checkNeighbour(int id, int n) {
	int v_pos = findVertexPos(id);
	if (v_pos == -1) {
		std::cerr << "Vertex with ID " << id << " does not exist [checkNeighbour].\n";
		return false;
	}

	for (int i = 0; i < adjacencyMatrix[v_pos].size(); i++) {
		if (vertices[i]->id == n && adjacencyMatrix[v_pos][i] != DEFAULT_WEIGHT) {
			return true;
		}
	}

	return false;
}

int AdjacencyMatrixGraph::findVertexPos(int v_id) {
	for (int i = 0; i < vertices.size(); i++) {
		if (vertices[i]->id == v_id) {
			return i;
		}
	}
	return -1;
}

Edge* AdjacencyMatrixGraph::findEdge(int idStart, int idEnd) {
	int v1_pos = findVertexPos(idStart);
	int v2_pos = findVertexPos(idEnd);

	if (v1_pos == -1 || v2_pos == -1) {
		std::cerr << "Vertices with IDs " << idStart << " and " << idEnd << " do not exist [findEdge].\n";
		return nullptr;
	}

	if (adjacencyMatrix[v1_pos][v2_pos] == DEFAULT_WEIGHT) {
		std::cerr << "Edge between vertices " << idStart << " and " << idEnd << " does not exist [findEdge].\n";
		return nullptr;
	}

	return new Edge(vertices[v1_pos], vertices[v2_pos], adjacencyMatrix[v1_pos][v2_pos]);
}

void AdjacencyMatrixGraph::print() {
	if (no_vertices == 0) {
		std::cerr << "Graph is empty [print]!\n";
		return;
	}

	std::cout << no_vertices << " " << no_edges << std::endl;

	for (int i = 0; i < adjacencyMatrix.size(); i++) {
		for (int j = 0; j < adjacencyMatrix[i].size(); j++) {
			if (adjacencyMatrix[i][j] != DEFAULT_WEIGHT) {
				std::cout << vertices[i]->id << " " << vertices[j]->id << " " << adjacencyMatrix[i][j] << std::endl;
			}
		}
	}

	if (sp_vertex != nullptr) {
		std::cout << sp_vertex->id << std::endl;
	}
}

void AdjacencyMatrixGraph::printMatrix() {
	std::cout << "Vertices: ";
	for (auto v : vertices) {
		std::cout << v->id << " ";
	}
	std::cout << "\n";

	std::cout << "Adjacency matrix:\n";
	for (int i = 0; i < adjacencyMatrix.size(); i++) {
		for (int j = 0; j < adjacencyMatrix[i].size(); j++) {
			std::cout << adjacencyMatrix[i][j] << " ";
		}
		std::cout << "\n";
	}
}

std::vector<SP_Node*> AdjacencyMatrixGraph::spDijkstra(int idStart) {
	std::vector<std::vector<Vertex*>> paths(no_vertices);

	// Convert given vertex ID to its position in the vertices list
	idStart = findVertexPos(idStart);
	
	// Check if starting vertex exists
	if (idStart == -1) {
		std::cerr << "Vertex with ID " << idStart << " does not exist [spDijkstra].\n";
		return std::vector<SP_Node*>();
	}

	// Initialize paths with starting vertex
	paths[idStart].push_back(vertices[idStart]);

    std::vector<int> dist(vertices.size(), DEFAULT_WEIGHT);
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pq;

	// Initialize starting vertex and priority queue
    dist[idStart] = 0;
    pq.push({0, idStart});

	// Dijkstra's algorithm itself
    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        for (int v = 0; v < vertices.size(); ++v) {
            if (adjacencyMatrix[u][v] != DEFAULT_WEIGHT) {
                int weight = adjacencyMatrix[u][v];
                if (dist[v] > dist[u] + weight) {
                    dist[v] = dist[u] + weight;
                    pq.push({dist[v], v});

					paths[v] = paths[u];
					paths[v].push_back(vertices[v]);
                }
            }
        }
    }

	// Convert results to SP_Node objects
    std::vector<SP_Node*> result;
    for (int i = 0; i < vertices.size(); ++i) {
        SP_Node* node = new SP_Node(vertices[i], dist[i], paths[i]);
        result.push_back(node);
    }

    return result;
}

std::vector<SP_Node*> AdjacencyMatrixGraph::spBellmanFord(int idStart) {
	std::vector<std::vector<Vertex*>> paths(no_vertices);

	// Convert given vertex ID to its position in the vertices list
	idStart = findVertexPos(idStart);
	
	// Check if starting vertex exists
	if (idStart == -1) {
		std::cerr << "Vertex with ID " << idStart << " does not exist [spBellmanFord].\n";
		return std::vector<SP_Node*>();
	}

	// Initialize paths with starting vertex
	paths[idStart].push_back(vertices[idStart]);

	std::vector<int> dist(vertices.size(), DEFAULT_WEIGHT);
	dist[idStart] = 0;

	// Bellman-Ford algorithm itself
	for (int i = 0; i < vertices.size() - 1; i++) {
		for (int u = 0; u < vertices.size(); u++) {
			for (int v = 0; v < vertices.size(); v++) {
				if (adjacencyMatrix[u][v] != DEFAULT_WEIGHT) {
					int weight = adjacencyMatrix[u][v];
					// Edge relaxation
					if (dist[v] > dist[u] + weight && dist[u] != DEFAULT_WEIGHT) {
						dist[v] = dist[u] + weight;

						paths[v] = paths[u];
						paths[v].push_back(vertices[v]);
					}
				}
			}
		}
	}

	// Check for negative cycles
	for (int u = 0; u < vertices.size(); u++) {
		for (int v = 0; v < vertices.size(); v++) {
			if (adjacencyMatrix[u][v] != DEFAULT_WEIGHT) {
				int weight = adjacencyMatrix[u][v];
				if (dist[v] > dist[u] + weight) {
					std::cerr << "Graph contains negative cycle [spBellmanFord].\n";
					return std::vector<SP_Node*>();
				}
			}
		}
	}

	// Convert results to SP_Node objects
	std::vector<SP_Node*> result;
	for (int i = 0; i < vertices.size(); ++i) {
		SP_Node* node = new SP_Node(vertices[i], dist[i], paths[i]);
		result.push_back(node);
	}

	return result;
}