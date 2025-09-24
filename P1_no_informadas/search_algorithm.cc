#include "search_algorithm.h"

SearchAlgorithm::SearchAlgorithm(const Graph* graph) : graph_(graph) {
}

SearchAlgorithm::~SearchAlgorithm() {
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