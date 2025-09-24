#ifndef DFS_H
#define DFS_H

#include "search_algorithm.h"
#include <stack>
#include <set>

/**
 * @brief Implementación del algoritmo de búsqueda en profundidad (DFS)
 * 
 * Utiliza una pila LIFO para explorar los nodos en profundidad,
 * explorando tan profundo como sea posible antes de retroceder.
 */
class DFS : public SearchAlgorithm {
private:
  std::stack<std::shared_ptr<Node>> frontier_;  // Pila de nodos por explorar
  std::set<int> explored_;                      // Conjunto de nodos ya explorados

public:
  /**
   * @brief Constructor
   * @param graph Puntero al grafo sobre el que realizar la búsqueda
   */
  DFS(const Graph* graph);

  /**
   * @brief Destructor
   */
  ~DFS();

  /**
   * @brief Ejecuta la búsqueda en profundidad
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
   * @brief Reinicia las estructuras específicas de DFS
   */
  void ResetDFS();

  /**
   * @brief Expande un nodo generando sus sucesores
   */
  std::vector<std::shared_ptr<Node>> ExpandNode(std::shared_ptr<Node> node);
};

#endif // DFS_H