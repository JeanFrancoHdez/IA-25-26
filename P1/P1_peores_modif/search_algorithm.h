#ifndef SEARCH_ALGORITHM_H
#define SEARCH_ALGORITHM_H

#include "graph.h"
#include "node.h"
#include <vector>
#include <string>
#include <memory>

// Estructura para almacenar información de cada iteración
struct IterationInfo {
  int iteration_number;
  std::vector<int> generated_nodes;
  std::vector<int> inspected_nodes;
  
  IterationInfo(int iter) : iteration_number(iter) {}
};

// Estructura para el resultado de la búsqueda
struct SearchResult {
  bool path_found;
  std::vector<int> path;
  double total_cost;
  std::vector<IterationInfo> iterations;
  
  SearchResult() : path_found(false), total_cost(0.0) {}
};

class SearchAlgorithm {
  public:
    SearchAlgorithm(const Graph* graph);
    virtual ~SearchAlgorithm();
    virtual SearchResult Search(int start, int goal) = 0;
    virtual std::string GetAlgorithmName() const = 0;
    std::string GenerateDetailedReport(const SearchResult& result, int start, int goal) const;
    bool SaveResultToFile(const SearchResult& result, int start, int goal, const std::string& filename) const;
    
  protected:
    std::vector<int> ReconstructPath(std::shared_ptr<Node> goal_node) const;
    double CalculatePathCost(const std::vector<int>& path) const;
    void AddIteration();
    void MarkNodeGenerated(int vertex);
    void MarkNodeInspected(int vertex);
    void Reset(); // Reinicia las estructuras de datos para una nueva búsqueda

    const Graph* graph_;
    std::vector<int> generated_nodes_;      // Nodos generados en la iteración actual
    std::vector<int> inspected_nodes_;      // Nodos inspeccionados en la iteración actual
    std::vector<IterationInfo> iterations_;
    int current_iteration_;
};

#endif