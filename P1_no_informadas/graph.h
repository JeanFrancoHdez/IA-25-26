#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <string>
#include <fstream>

/**
 * @brief Clase que representa un grafo con matriz de adyacencia
 * 
 * Esta clase maneja un grafo no dirigido con pesos en las aristas.
 * Lee el formato específico del problema y proporciona métodos para
 * acceder a la información del grafo.
 */
class Graph {
private:
  int num_vertices_;                          // Número de vértices del grafo
  std::vector<std::vector<double>> matrix_;   // Matriz de adyacencia con costos
  int num_edges_;                             // Número de aristas del grafo

  /**
   * @brief Calcula el número de aristas del grafo
   */
  void CalculateEdges();

public:
  /**
   * @brief Constructor por defecto
   */
  Graph();

  /**
   * @brief Constructor que carga el grafo desde un archivo
   * @param filename Nombre del archivo con el formato específico
   */
  Graph(const std::string& filename);

  /**
   * @brief Destructor
   */
  ~Graph();

  /**
   * @brief Carga el grafo desde un archivo
   * @param filename Nombre del archivo
   * @return true si se cargó correctamente, false en caso contrario
   */
  bool LoadFromFile(const std::string& filename);

  /**
   * @brief Obtiene el número de vértices
   */
  int GetNumVertices() const;

  /**
   * @brief Obtiene el número de aristas
   */
  int GetNumEdges() const;

  /**
   * @brief Obtiene el costo de la arista entre dos vértices
   * @param from Vértice origen (1-indexado)
   * @param to Vértice destino (1-indexado)
   * @return Costo de la arista, -1 si no existe conexión
   */
  double GetEdgeCost(int from, int to) const;

  /**
   * @brief Verifica si existe una arista entre dos vértices
   * @param from Vértice origen (1-indexado)
   * @param to Vértice destino (1-indexado)
   * @return true si existe la arista, false en caso contrario
   */
  bool HasEdge(int from, int to) const;

  /**
   * @brief Obtiene los vecinos de un vértice
   * @param vertex Vértice (1-indexado)
   * @return Vector con los vértices vecinos
   */
  std::vector<int> GetNeighbors(int vertex) const;

  /**
   * @brief Verifica si un vértice es válido
   * @param vertex Vértice a verificar (1-indexado)
   * @return true si es válido, false en caso contrario
   */
  bool IsValidVertex(int vertex) const;

  /**
   * @brief Convierte el grafo a string para depuración
   */
  std::string ToString() const;

  /**
   * @brief Imprime la matriz de adyacencia
   */
  void PrintMatrix() const;
};

#endif // GRAPH_H