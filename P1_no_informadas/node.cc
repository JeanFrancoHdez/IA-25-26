#include "node.h"

Node::Node(int vertex, std::shared_ptr<Node> parent, double path_cost)
  : vertex_(vertex), parent_(parent), path_cost_(path_cost) {
}

// Getters
int Node::GetVertex() const {
  return vertex_;
}

std::shared_ptr<Node> Node::GetParent() const {
  return parent_;
}

double Node::GetPathCost() const {
  return path_cost_;
}