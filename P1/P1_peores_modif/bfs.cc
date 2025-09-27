#include "bfs.h"

BFS::BFS(const Graph* graph) : SearchAlgorithm(graph) {}

std::string BFS::GetAlgorithmName() const {
  return "BFS";
}

SearchResult BFS::Search(int start, int goal) {
  // Reiniciar estructuras
  Reset();
  frontier_.clear();
  explored_.clear();
  
  SearchResult result;
  
  if (!graph_->IsValidVertex(start) || !graph_->IsValidVertex(goal)) {
    return result;
  }
  
  std::shared_ptr<Node> start_node = std::make_shared<Node>(start);
  frontier_.push_back(start_node);
  MarkNodeGenerated(start);
  
  AddIteration();
  
  while (!frontier_.empty()) {
    std::shared_ptr<Node> current_node = SelectWorstNode();
    
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
        frontier_.push_back(neighbor_node);
      }
    }
    
    AddIteration();
  }
  
  // No se encontró camino
  result.iterations = iterations_;
  return result;
}

std::shared_ptr<Node> BFS::SelectWorstNode() {
  // Si solo hay un nodo, devolverlo (primera iteración)
  if (frontier_.size() == 1) {
    std::shared_ptr<Node> node = frontier_[0];
    frontier_.erase(frontier_.begin());
    return node;
  }
  
  // Encontrar los dos nodos con mayor costo
  int worst_idx = 0;
  int second_worst_idx = 1;
  
  // Asegurar que worst tenga mayor costo que second_worst
  if (frontier_[1]->GetPathCost() > frontier_[0]->GetPathCost()) {
    worst_idx = 1;
    second_worst_idx = 0;
  }
  
  // Buscar los dos peores en toda la frontera
  for (size_t i = 2; i < frontier_.size(); ++i) {
    double current_cost = frontier_[i]->GetPathCost();
    
    if (current_cost > frontier_[worst_idx]->GetPathCost()) {
      // Nuevo peor nodo encontrado
      second_worst_idx = worst_idx;
      worst_idx = i;
    } else if (current_cost > frontier_[second_worst_idx]->GetPathCost()) {
      // Nuevo segundo peor nodo encontrado
      second_worst_idx = i;
    }
  }
  
  // Seleccionar aleatoriamente entre los dos peores
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, 1);
  
  int selected_idx = (dis(gen) == 0) ? worst_idx : second_worst_idx;
  
  // Remover y devolver el nodo seleccionado
  std::shared_ptr<Node> selected_node = frontier_[selected_idx];
  frontier_.erase(frontier_.begin() + selected_idx);
  
  return selected_node;
}