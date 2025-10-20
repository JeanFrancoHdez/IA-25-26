#include "A_star_node.h"

AStarNode::AStarNode(const Position& position, std::shared_ptr<AStarNode> parent, double g_cost, double h_cost)
  : position_(position), parent_(parent), g_cost_(g_cost), h_cost_(h_cost) {
}

Position AStarNode::GetPosition() const {
  return position_;
}

std::shared_ptr<AStarNode> AStarNode::GetParent() const {
  return parent_;
}

double AStarNode::GetGCost() const {
  return g_cost_;
}

double AStarNode::GetHCost() const {
  return h_cost_;
}

double AStarNode::GetFCost() const {
  return g_cost_ + h_cost_;
}

void AStarNode::SetGCost(double g_cost) {
  g_cost_ = g_cost;
}

void AStarNode::SetHCost(double h_cost) {
  h_cost_ = h_cost;
}

void AStarNode::SetParent(std::shared_ptr<AStarNode> parent) {
  parent_ = parent;
}
