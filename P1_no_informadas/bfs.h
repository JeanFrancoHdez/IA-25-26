#ifndef BFS_H
#define BFS_H

#include "search_algorithm.h"
#include <queue>
#include <set>

/**
 * @brief Implementación del algoritmo de búsqueda en amplitud (BFS)
 * 
 * Utiliza una cola FIFO para explorar los nodos nivel por nivel,
 * garantizando encontrar el camino con menor número de aristas.
 */
class BFS : public SearchAlgorithm {
private:
  std::queue<std::shared_ptr<Node>> frontier_;  // Cola de nodos por explorar
  std::set<int> explored_;                      // Conjunto de nodos ya explorados

public:
  /**
   * @brief Constructor
   * @param graph Puntero al grafo sobre el que realizar la búsqueda
   */
  BFS(const Graph* graph);

  /**
   * @brief Destructor
   */
  ~BFS();

  /**
   * @brief Ejecuta la búsqueda en amplitud
   * @param start Vértice de origen (1-indexado)
   * @param goal Vértice de destino (1-indexado)
   * @return Resultado de la búsqueda
   */
  SearchResult Search(int start, int goal) override;

  /**
   * @brief Obtiene el nombre del algoritmo
   */
  std::string GetAlgorithmName() const override;

private:
  /**
   * @brief Reinicia las estructuras específicas de BFS
   */
  void ResetBFS();

  /**
   * @brief Expande un nodo generando sus sucesores
   */
  std::vector<std::shared_ptr<Node>> ExpandNode(std::shared_ptr<Node> node);
};

#endif // BFS_H