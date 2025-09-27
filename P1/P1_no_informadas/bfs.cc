#include "bfs.h"

BFS::BFS(const Graph* graph) : SearchAlgorithm(graph) {}

std::string BFS::GetAlgorithmName() const {
  return "BFS";
}

SearchResult BFS::Search(int start, int goal) {
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