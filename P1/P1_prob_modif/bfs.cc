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
    std::shared_ptr<Node> current_node = SelectNodeByProbability();
    
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

std::shared_ptr<Node> BFS::SelectNodeByProbability() {
  // Si solo hay un nodo, devolverlo directamente
  if (frontier_.size() == 1) {
    std::shared_ptr<Node> node = frontier_[0];
    frontier_.erase(frontier_.begin());
    return node;
  }
  
  // Calcular t(n) = 1/v(n) para cada nodo y sumatoria total
  std::vector<double> probabilities;
  double total_sum = 0.0;
  
  for (const auto& node : frontier_) {
    double cost = node->GetPathCost();
    // Evitar división por cero: si cost = 0, usar un valor pequeño
    if (cost <= 0.0) {
      cost = 0.001;  // Valor pequeño para evitar división por cero
    }
    double t_n = 1.0 / cost;  // t(n) = 1/v(n)
    probabilities.push_back(t_n);
    total_sum += t_n;
  }
  
  // Convertir a probabilidades normalizadas: Pr(n) = t(n) / Σ t(n)
  for (double& prob : probabilities) {
    prob = prob / total_sum;
  }
  
  // Generar número aleatorio entre 0 y 1
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.0, 1.0);
  double random_value = dis(gen);
  
  // Selección por ruleta: encontrar el índice según la distribución acumulativa
  double cumulative_prob = 0.0;
  size_t selected_idx = 0;
  
  for (size_t i = 0; i < probabilities.size(); ++i) {
    cumulative_prob += probabilities[i];
    if (random_value <= cumulative_prob) {
      selected_idx = i;
      break;
    }
  }
  
  // Remover y devolver el nodo seleccionado
  std::shared_ptr<Node> selected_node = frontier_[selected_idx];
  frontier_.erase(frontier_.begin() + selected_idx);
  
  return selected_node;
}