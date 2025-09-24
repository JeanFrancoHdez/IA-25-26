#ifndef NODE_H
#define NODE_H

#include <memory>

// Clase para representar un nodo en el árbol de búsqueda
class Node {
  public:
    Node(int vertex, std::shared_ptr<Node> parent = nullptr, double path_cost = 0.0);
    // Getters
    int GetVertex() const;
    std::shared_ptr<Node> GetParent() const;
    double GetPathCost() const;

  private:
    int vertex_;                    // Vértice del nodo que representa
    std::shared_ptr<Node> parent_;  // Puntero al nodo padre
    double path_cost_;              // Costo acumulado desde el nodo inicial
};

#endif