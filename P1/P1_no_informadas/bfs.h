#ifndef BFS_H
#define BFS_H

#include "search_algorithm.h"
#include <queue>
#include <set>

// Implementación de búsqueda en amplitud (BFS)
class BFS : public SearchAlgorithm {
  public:
    BFS(const Graph* graph);
    SearchResult Search(int start, int goal) override;
    std::string GetAlgorithmName() const override;
  private:
    std::queue<std::shared_ptr<Node>> frontier_;  // Cola de nodos por explorar
    std::set<int> explored_;                      // Conjunto de nodos ya explorados
};

#endif