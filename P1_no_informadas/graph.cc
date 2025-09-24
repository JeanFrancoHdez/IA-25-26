#include "graph.h"
#include <iostream>
#include <sstream>
#include <iomanip>

Graph::Graph() : num_vertices_(0), num_edges_(0) {
}

Graph::Graph(const std::string& filename) : num_vertices_(0), num_edges_(0) {
  LoadFromFile(filename);
}

Graph::~Graph() {
}

// Carga el grafo desde archivo
bool Graph::LoadFromFile(const std::string& filename) {
  std::ifstream file(filename);
  
  if (!file.is_open()) {
    std::cerr << "Error: No se pudo abrir el archivo " << filename << std::endl;
    return false;
  }

  file >> num_vertices_;
  
  if (num_vertices_ <= 0) {
    std::cerr << "Error: Número de vértices inválido: " << num_vertices_ << std::endl;
    return false;
  }

  matrix_.resize(num_vertices_, std::vector<double>(num_vertices_, -1.0));
  
  // Establecer diagonal principal a 0 (distancia de un vértice a sí mismo)
  for (int i = 0; i < num_vertices_; ++i) {
    matrix_[i][i] = 0.0;
  }

  // Leer las distancias entre pares de vértices
  for (int i = 0; i < num_vertices_; ++i) {
    for (int j = i + 1; j < num_vertices_; ++j) {
      double distance;
      if (!(file >> distance)) {
        std::cerr << "Error: No se pudo leer la distancia entre vértices " 
                  << (i + 1) << " y " << (j + 1) << std::endl;
        return false;
      }
      
      // Como el grafo es no dirigido, establecer ambas direcciones
      matrix_[i][j] = distance;
      matrix_[j][i] = distance;
    }
  }

  file.close();
  
  CalculateEdges();
  
  return true;
}

void Graph::CalculateEdges() {
  num_edges_ = 0;
  
  for (int i = 0; i < num_vertices_; ++i) {
    for (int j = i + 1; j < num_vertices_; ++j) {
      if (matrix_[i][j] >= 0) {
        num_edges_++;
      }
    }
  }
}

// Getters
int Graph::GetNumVertices() const {
  return num_vertices_;
}

int Graph::GetNumEdges() const {
  return num_edges_;
}

double Graph::GetEdgeCost(int from, int to) const {
  if (!IsValidVertex(from) || !IsValidVertex(to)) {
    return -1.0;
  }
  
  return matrix_[from - 1][to - 1];
}

bool Graph::HasEdge(int from, int to) const {
  if (!IsValidVertex(from) || !IsValidVertex(to)) {
    return false;
  }
  
  return matrix_[from - 1][to - 1] >= 0;
}

std::vector<int> Graph::GetNeighbors(int vertex) const {
  std::vector<int> neighbors;
  
  if (!IsValidVertex(vertex)) {
    return neighbors;
  }
  
  int vertex_index = vertex - 1;
  
  for (int i = 0; i < num_vertices_; ++i) {
    if (i != vertex_index && matrix_[vertex_index][i] >= 0) {
      neighbors.push_back(i + 1);
    }
  }
  
  return neighbors;
}

bool Graph::IsValidVertex(int vertex) const {
  return vertex >= 1 && vertex <= num_vertices_;
}

std::string Graph::ToString() const {
  std::ostringstream oss;
  oss << "Grafo con " << num_vertices_ << " vértices y " << num_edges_ << " aristas\n";
  oss << "Matriz de adyacencia:\n";
  
  for (int i = 0; i < num_vertices_; ++i) {
    for (int j = 0; j < num_vertices_; ++j) {
      oss << std::fixed << std::setprecision(3) << std::setw(8) << matrix_[i][j] << " ";
    }
    oss << "\n";
  }
  
  return oss.str();
}

void Graph::PrintMatrix() const {
  std::cout << ToString();
}