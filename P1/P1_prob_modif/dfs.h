#ifndef DFS_H
#define DFS_H

#include "search_algorithm.h"
#include <stack>
#include <set>

// Implementación de búsqueda en profundidad (DFS)
class DFS : public SearchAlgorithm {
  public:
    DFS(const Graph* graph);
    SearchResult Search(int start, int goal) override;
    std::string GetAlgorithmName() const override;

  private:
    std::stack<std::shared_ptr<Node>> frontier_;  // Pila de nodos por explorar
    std::set<int> explored_;                      // Conjunto de nodos ya explorados
};

#endif