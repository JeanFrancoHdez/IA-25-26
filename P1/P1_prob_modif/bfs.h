#ifndef BFS_H
#define BFS_H

#include "search_algorithm.h"
#include <vector>
#include <set>
#include <random>

// Implementación de búsqueda en amplitud (BFS)
class BFS : public SearchAlgorithm {
  public:
    BFS(const Graph* graph);
    SearchResult Search(int start, int goal) override;
    std::string GetAlgorithmName() const override;
  private:
    std::vector<std::shared_ptr<Node>> frontier_;  // Vector de nodos por explorar
    std::set<int> explored_;                       // Conjunto de nodos ya explorados
    
    // Función para seleccionar nodo según distribución de probabilidad
    std::shared_ptr<Node> SelectNodeByProbability();
};

#endif