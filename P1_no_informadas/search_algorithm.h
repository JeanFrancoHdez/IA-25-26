#ifndef SEARCH_ALGORITHM_H
#define SEARCH_ALGORITHM_H

#include "graph.h"
#include "node.h"
#include <vector>
#include <string>
#include <memory>

struct SearchResult {
  bool path_found;
  std::vector<int> path;
  double total_cost;
  
  SearchResult() : path_found(false), total_cost(0.0) {}
};

class SearchAlgorithm {
  public:
    SearchAlgorithm(const Graph* graph);
    virtual ~SearchAlgorithm();
    virtual SearchResult Search(int start, int goal) = 0;
    virtual std::string GetAlgorithmName() const = 0;
  
  protected:
    std::vector<int> ReconstructPath(std::shared_ptr<Node> goal_node) const;
    double CalculatePathCost(const std::vector<int>& path) const;
    
    const Graph* graph_;
};

#endif