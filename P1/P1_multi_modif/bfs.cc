#include "bfs.h"
#include <random>
#include <algorithm>

BFS::BFS(const Graph* graph) : SearchAlgorithm(graph) {}

std::string BFS::GetAlgorithmName() const {
  return "BFS";
}

// Función auxiliar para hacer BFS desde cualquier nodo
SearchResult BFS::BFS_From_Node(int start, int goal) {
  // Reiniciar estructuras
  Reset();
  while (!frontier_.empty()) {
    frontier_.pop();
  }
  explored_.clear();
  
  SearchResult result;
  
  if (!graph_->IsValidVertex(start) || !graph_->IsValidVertex(goal)) {
    return result;
  }
  
  std::shared_ptr<Node> start_node = std::make_shared<Node>(start);
  frontier_.push(start_node);
  MarkNodeGenerated(start);
  
  AddIteration();
  
  while (!frontier_.empty()) {
    std::shared_ptr<Node> current_node = frontier_.front();
    frontier_.pop();
    
    int current_vertex = current_node->GetVertex();
    
    if (explored_.find(current_vertex) != explored_.end()) {
      continue;
    }
    
    explored_.insert(current_vertex);
    MarkNodeInspected(current_vertex);
    
    if (current_vertex == goal) {
      result.path_found = true;
      result.path = ReconstructPath(current_node);
      result.total_cost = CalculatePathCost(result.path);
      result.iterations = iterations_;
      return result;
    }
    
    std::vector<int> neighbors = graph_->GetNeighbors(current_vertex);
    for (int neighbor : neighbors) {
      // Solo generar y añadir a la frontera si no ha sido explorado
      if (explored_.find(neighbor) == explored_.end()) {
        MarkNodeGenerated(neighbor);
        
        double edge_cost = graph_->GetEdgeCost(current_vertex, neighbor);
        double new_path_cost = current_node->GetPathCost() + edge_cost;
        
        std::shared_ptr<Node> neighbor_node = std::make_shared<Node>(neighbor, current_node, new_path_cost);
        frontier_.push(neighbor_node);
      }
    }
    
    AddIteration();
  }
  
  // No se encontró camino
  result.iterations = iterations_;
  return result;
}

SearchResult BFS::Search(int start, int goal) {
  if (!graph_->IsValidVertex(start) || !graph_->IsValidVertex(goal)) {
    SearchResult result;
    return result;
  }
  
  // Primero intentar BFS directo desde el nodo inicial
  SearchResult result = BFS_From_Node(start, goal);
  if (result.path_found) {
    return result;
  }
  
  // Si no se encuentra, hacer multiarranque
  std::random_device rd;
  std::mt19937 gen(rd());
  
  // Obtener los hijos del nodo inicial
  std::vector<int> children = graph_->GetNeighbors(start);
  if (children.empty()) {
    return result; // No hay hijos, devolver resultado vacío
  }
  
  // Máximo 10 intentos
  for (int attempt = 0; attempt < 10; ++attempt) {
    // Seleccionar un hijo al azar
    std::uniform_int_distribution<> dis(0, children.size() - 1);
    int random_child = children[dis(gen)];
    
    // Hacer BFS desde ese hijo
    SearchResult attempt_result = BFS_From_Node(random_child, goal);
    if (attempt_result.path_found) {
      // Añadir el nodo inicial al principio del camino
      attempt_result.path.insert(attempt_result.path.begin(), start);
      attempt_result.total_cost += graph_->GetEdgeCost(start, random_child);
      return attempt_result;
    }
  }
  
  // No se encontró camino en ningún intento
  return result;
}