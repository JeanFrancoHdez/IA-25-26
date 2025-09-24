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
  public:
    Graph();
    Graph(const std::string& filename);
    ~Graph();
    bool LoadFromFile(const std::string& filename);
    int GetNumVertices() const;
    int GetNumEdges() const;
    double GetEdgeCost(int from, int to) const;
    bool HasEdge(int from, int to) const;
    std::vector<int> GetNeighbors(int vertex) const;
    bool IsValidVertex(int vertex) const;
    std::string ToString() const;
    void PrintMatrix() const;
    void CalculateEdges();

  private:
    int num_vertices_;                          // Número de vértices del grafo
    std::vector<std::vector<double>> matrix_;   // Matriz de adyacencia con costos
    int num_edges_;                             // Número de aristas del grafo
};

#endif // GRAPH_H