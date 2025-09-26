#include "search_algorithm.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>

SearchAlgorithm::SearchAlgorithm(const Graph* graph) : graph_(graph), current_iteration_(0) {
}

SearchAlgorithm::~SearchAlgorithm() {
}

void SearchAlgorithm::MarkNodeGenerated(int vertex) {
  generated_nodes_.push_back(vertex);
  std::sort(generated_nodes_.begin(), generated_nodes_.end()); // Mantener ordenado
}

std::vector<int> SearchAlgorithm::ReconstructPath(std::shared_ptr<Node> goal_node) const {
  std::vector<int> path;
  std::shared_ptr<Node> current = goal_node;
  
  // Construir el camino hacia atrás
  while (current != nullptr) {
    path.insert(path.begin(), current->GetVertex());
    current = current->GetParent();
  }
  
  return path;
}

double SearchAlgorithm::CalculatePathCost(const std::vector<int>& path) const {
  if (path.size() < 2) {
    return 0.0;
  }
  
  double total_cost = 0.0;
  for (size_t i = 0; i < path.size() - 1; ++i) {
    double edge_cost = graph_->GetEdgeCost(path[i], path[i + 1]);
    total_cost += edge_cost;
  }
  
  return total_cost;
}

// Agrega información de la iteración actual
void SearchAlgorithm::AddIteration() {
  current_iteration_++;
  IterationInfo info(current_iteration_);
  
  info.generated_nodes = generated_nodes_;
  info.inspected_nodes = inspected_nodes_;
  iterations_.push_back(info);
}

void SearchAlgorithm::MarkNodeInspected(int vertex) {
  if (std::find(inspected_nodes_.begin(), inspected_nodes_.end(), vertex) == inspected_nodes_.end()) {
    inspected_nodes_.push_back(vertex);
    std::sort(inspected_nodes_.begin(), inspected_nodes_.end());
  }
}

// Reinicia las estructuras para nueva búsqueda
void SearchAlgorithm::Reset() {
  generated_nodes_.clear();
  inspected_nodes_.clear();
  iterations_.clear();
  current_iteration_ = 0;
}

std::string SearchAlgorithm::GenerateDetailedReport(const SearchResult& result, int start, int goal) const {
  std::ostringstream oss;
  
  // Información del grafo
  oss << "--------------------------------------\n";
  oss << "Número de nodos del grafo: " << graph_->GetNumVertices() << "\n";
  oss << "Número de aristas del grafo: " << graph_->GetNumEdges() << "\n";
  oss << "Vértice origen: " << start << "\n";
  oss << "Vértice destino: " << goal << "\n";
  oss << "--------------------------------------\n";
  
  // Información de cada iteración
  for (const auto& iteration : result.iterations) {
    oss << "Iteración " << iteration.iteration_number << "\n";
    
    oss << "Nodos generados: ";
    if (iteration.generated_nodes.empty()) {
      oss << "-";
    } else {
      for (size_t i = 0; i < iteration.generated_nodes.size(); ++i) {
        if (i > 0) oss << ", ";
        oss << iteration.generated_nodes[i];
      }
    }
    oss << "\n";
    
    oss << "Nodos inspeccionados: ";
    if (iteration.inspected_nodes.empty()) {
      oss << "-";
    } else {
      for (size_t i = 0; i < iteration.inspected_nodes.size(); ++i) {
        if (i > 0) oss << ", ";
        oss << iteration.inspected_nodes[i];
      }
    }
    oss << "\n";
    oss << "--------------------------------------\n";
  }
  
  // Resultado final
  if (result.path_found) {
    oss << "Camino: ";
    for (size_t i = 0; i < result.path.size(); ++i) {
      if (i > 0) oss << " - ";
      oss << result.path[i];
    }
    oss << "\n";
    oss << "--------------------------------------\n";
    oss << "Costo: " << std::fixed << std::setprecision(2) << result.total_cost << "\n";
  } else {
    oss << "No se encontró camino entre " << start << " y " << goal << "\n";
  }
  oss << "--------------------------------------\n";
  
  return oss.str();
}

bool SearchAlgorithm::SaveResultToFile(const SearchResult& result, int start, int goal, const std::string& filename) const {
  std::ofstream file(filename);
  
  if (!file.is_open()) {
    std::cerr << "Error: No se pudo crear el archivo " << filename << std::endl;
    return false;
  }
  
  file << GenerateDetailedReport(result, start, goal);
  file.close();
  
  return true;
}