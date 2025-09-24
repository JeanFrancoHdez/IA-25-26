#include "node.h"
#include <vector>
#include <sstream>

// Constructor por defecto
Node::Node() : state_(""), parent_(nullptr), path_cost_(0.0), depth_(0) {
}

// Constructor con parámetros
Node::Node(const std::string& state, std::shared_ptr<Node> parent, 
           double path_cost, int depth)
  : state_(state), parent_(parent), path_cost_(path_cost), depth_(depth) {
}

// Constructor de copia
Node::Node(const Node& other)
  : state_(other.state_), parent_(other.parent_), 
    path_cost_(other.path_cost_), depth_(other.depth_) {
}

// Operador de asignación
Node& Node::operator=(const Node& other) {
  if (this != &other) {
    state_ = other.state_;
    parent_ = other.parent_;
    path_cost_ = other.path_cost_;
    depth_ = other.depth_;
  }
  return *this;
}

// Destructor
Node::~Node() {
}

// Getters
const std::string& Node::GetState() const {
  return state_;
}

std::shared_ptr<Node> Node::GetParent() const {
  return parent_;
}

double Node::GetPathCost() const {
  return path_cost_;
}

int Node::GetDepth() const {
  return depth_;
}

// Setters
void Node::SetState(const std::string& state) {
  state_ = state;
}

void Node::SetParent(std::shared_ptr<Node> parent) {
  parent_ = parent;
}

void Node::SetPathCost(double path_cost) {
  path_cost_ = path_cost;
}

void Node::SetDepth(int depth) {
  depth_ = depth;
}

// Operador de comparación para igualdad
bool Node::operator==(const Node& other) const {
  return state_ == other.state_;
}

// Operador de comparación para ordenación
bool Node::operator<(const Node& other) const {
  return path_cost_ < other.path_cost_;
}

// Convierte el nodo a string
std::string Node::ToString() const {
  std::ostringstream oss;
  oss << "Estado: " << state_ 
      << ", Costo: " << path_cost_ 
      << ", Profundidad: " << depth_;
  return oss.str();
}

// Obtiene el camino desde la raíz hasta este nodo
std::vector<std::string> Node::GetPath() const {
  std::vector<std::string> path;
  const Node* current = this;
  
  // Construir el camino hacia atrás usando punteros raw
  std::vector<std::string> reverse_path;
  while (current != nullptr) {
    reverse_path.push_back(current->GetState());
    current = current->GetParent().get();
  }
  
  // Invertir el camino para que vaya desde la raíz hasta este nodo
  path.reserve(reverse_path.size());
  for (auto it = reverse_path.rbegin(); it != reverse_path.rend(); ++it) {
    path.push_back(*it);
  }
  
  return path;
}