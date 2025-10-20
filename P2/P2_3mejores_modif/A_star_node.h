#ifndef ASTAR_NODE_H
#define ASTAR_NODE_H

#include <memory>
#include "maze.h"

/**
 * @brief Clase para representar un nodo en el algoritmo A*
 */
class AStarNode {
  public:
    AStarNode(const Position& position, std::shared_ptr<AStarNode> parent = nullptr, double g_cost = 0.0, double h_cost = 0.0);
  
    Position GetPosition() const;
    std::shared_ptr<AStarNode> GetParent() const;
    double GetGCost() const;
    double GetHCost() const;
    double GetFCost() const;      // f(n) = g(n) + h(n)
  
    void SetGCost(double g_cost);
    void SetHCost(double h_cost);
    void SetParent(std::shared_ptr<AStarNode> parent);

  private:
    Position position_;
    std::shared_ptr<AStarNode> parent_;
    double g_cost_;
    double h_cost_;
};

// Comparador para establecer prioridad (a menor f_cost, mayor prioridad)
struct AStarNodeComparator {
  bool operator()(const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) const {
    return a->GetFCost() > b->GetFCost();
  }
};

#endif