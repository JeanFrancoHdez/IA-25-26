#ifndef ASTAR_NODE_H
#define ASTAR_NODE_H

#include <memory>
#include "map.h"

/**
 * @brief Clase para representar un nodo en el algoritmo A*
 */
class AStarNode {
public:
  AStarNode(const Position& position, std::shared_ptr<AStarNode> parent = nullptr, 
    double g_cost = 0.0, double h_cost = 0.0);
  
  // Getters
  Position GetPosition() const;
  std::shared_ptr<AStarNode> GetParent() const;
  double GetGCost() const;      // g(n) - costo acumulado desde inicio
  double GetHCost() const;      // h(n) - heurística hasta objetivo
  double GetFCost() const;      // f(n) = g(n) + h(n)
  
  // Setters
  void SetGCost(double g_cost);
  void SetHCost(double h_cost);
  void SetParent(std::shared_ptr<AStarNode> parent);
  
  // Operadores para comparación (necesarios para priority_queue)
  bool operator<(const AStarNode& other) const;
  bool operator>(const AStarNode& other) const;

private:
  Position position_;                      // Posición en el laberinto
  std::shared_ptr<AStarNode> parent_;     // Nodo padre en el camino
  double g_cost_;                         // Costo acumulado g(n)
  double h_cost_;                         // Heurística h(n)
};

// Comparador para priority_queue (menor f_cost tiene mayor prioridad)
struct AStarNodeComparator {
  bool operator()(const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) const {
  return a->GetFCost() > b->GetFCost();  // Min-heap basado en f_cost
  }
};

#endif