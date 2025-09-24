#ifndef SEARCH_ALGORITHM_H
#define SEARCH_ALGORITHM_H

#include "graph.h"
#include "node.h"
#include <vector>
#include <string>
#include <memory>

/**
 * @brief Estructura para almacenar información de cada iteración
 */
struct IterationInfo {
  int iteration_number;
  std::vector<int> generated_nodes;
  std::vector<int> inspected_nodes;
  
  IterationInfo(int iter) : iteration_number(iter) {}
};

/**
 * @brief Estructura para almacenar el resultado de la búsqueda
 */
struct SearchResult {
  bool path_found;
  std::vector<int> path;
  double total_cost;
  std::vector<IterationInfo> iterations;
  int nodes_generated;
  int nodes_inspected;
  
  SearchResult() : path_found(false), total_cost(0.0), nodes_generated(0), nodes_inspected(0) {}
};

/**
 * @brief Clase abstracta base para algoritmos de búsqueda
 * 
 * Define la interfaz común para todos los algoritmos de búsqueda
 * no informada (BFS, DFS, etc.)
 */
class SearchAlgorithm {
protected:
  const Graph* graph_;                    // Puntero al grafo
  std::vector<int> generated_nodes_;      // Nodos generados en la iteración actual
  std::vector<int> inspected_nodes_;      // Nodos inspeccionados en la iteración actual
  std::vector<IterationInfo> iterations_; // Historia de iteraciones
  int current_iteration_;                 // Número de iteración actual

  /**
   * @brief Agrega información de la iteración actual
   */
  void AddIteration();

  /**
   * @brief Marca un nodo como generado
   */
  void MarkNodeGenerated(int vertex);

  /**
   * @brief Marca un nodo como inspeccionado
   */
  void MarkNodeInspected(int vertex);

  /**
   * @brief Verifica si un nodo ya fue generado en esta iteración
   */
  bool IsNodeGenerated(int vertex) const;

  /**
   * @brief Crea un nodo compartido
   */
  std::shared_ptr<Node> CreateNode(int vertex, std::shared_ptr<Node> parent = nullptr, 
                                   double path_cost = 0.0, int depth = 0);

public:
  /**
   * @brief Constructor
   * @param graph Puntero al grafo sobre el que se realizará la búsqueda
   */
  SearchAlgorithm(const Graph* graph);

  /**
   * @brief Destructor virtual
   */
  virtual ~SearchAlgorithm();

  /**
   * @brief Método virtual puro para ejecutar la búsqueda
   * @param start Vértice de origen (1-indexado)
   * @param goal Vértice de destino (1-indexado)
   * @return Resultado de la búsqueda
   */
  virtual SearchResult Search(int start, int goal) = 0;

  /**
   * @brief Obtiene el nombre del algoritmo
   */
  virtual std::string GetAlgorithmName() const = 0;

  /**
   * @brief Genera el reporte de resultado en formato string
   */
  std::string GenerateReport(const SearchResult& result, int start, int goal) const;

  /**
   * @brief Guarda el resultado en un archivo
   */
  bool SaveResultToFile(const SearchResult& result, int start, int goal, 
                        const std::string& filename) const;

protected:
  /**
   * @brief Reconstruye el camino desde el nodo objetivo hasta el origen
   */
  std::vector<int> ReconstructPath(std::shared_ptr<Node> goal_node) const;

  /**
   * @brief Calcula el costo total del camino
   */
  double CalculatePathCost(const std::vector<int>& path) const;

  /**
   * @brief Reinicia las estructuras de datos para una nueva búsqueda
   */
  void Reset();
};

#endif // SEARCH_ALGORITHM_H