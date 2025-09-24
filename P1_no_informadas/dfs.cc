#include "dfs.h"

DFS::DFS(const Graph* graph) : SearchAlgorithm(graph) {}

std::string DFS::GetAlgorithmName() const {
  return "DFS";
}

SearchResult DFS::Search(int start, int goal) {
  // Limpiar estructuras
  while (!frontier_.empty()) {
    frontier_.pop();
  }
  explored_.clear();
  
  SearchResult result;
  
  if (!graph_->IsValidVertex(start) || !graph_->IsValidVertex(goal)) {
    return result;
  }
  
  // Crear nodo inicial y añadirlo a la pila
  std::shared_ptr<Node> start_node = std::make_shared<Node>(start);
  frontier_.push(start_node);
  
  while (!frontier_.empty()) {
    // Tomar el nodo del tope de la pila
    std::shared_ptr<Node> current_node = frontier_.top();
    frontier_.pop();
    
    int current_vertex = current_node->GetVertex();
    
    if (explored_.find(current_vertex) != explored_.end()) {
      continue;
    }
    
    explored_.insert(current_vertex);
    
    if (current_vertex == goal) {
      result.path_found = true;
      result.path = ReconstructPath(current_node);
      result.total_cost = CalculatePathCost(result.path);
      return result;
    }
    
    // Expandir vecinos (en orden reverso para DFS)
    std::vector<int> neighbors = graph_->GetNeighbors(current_vertex);
    for (auto it = neighbors.rbegin(); it != neighbors.rend(); ++it) {
      int neighbor = *it;
      // Solo añadir vecinos no explorados
      if (explored_.find(neighbor) == explored_.end()) {
        double edge_cost = graph_->GetEdgeCost(current_vertex, neighbor);
        double new_path_cost = current_node->GetPathCost() + edge_cost;
        
        std::shared_ptr<Node> neighbor_node = std::make_shared<Node>(neighbor, current_node, new_path_cost);
        frontier_.push(neighbor_node);
      }
    }
  }
  
  // No se encontró camino
  return result;
}