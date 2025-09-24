#include "bfs.h"
#include <iostream>

// Constructor
BFS::BFS(const Graph* graph) : SearchAlgorithm(graph) {
}

// Destructor
BFS::~BFS() {
}

// Obtiene el nombre del algoritmo
std::string BFS::GetAlgorithmName() const {
  return "Búsqueda en Amplitud (BFS)";
}

// Reinicia las estructuras específicas de BFS
void BFS::ResetBFS() {
  // Limpiar cola
  while (!frontier_.empty()) {
    frontier_.pop();
  }
  explored_.clear();
}

// Expande un nodo generando sus sucesores
std::vector<std::shared_ptr<Node>> BFS::ExpandNode(std::shared_ptr<Node> node) {
  std::vector<std::shared_ptr<Node>> successors;
  
  int current_vertex = std::stoi(node->GetState());
  std::vector<int> neighbors = graph_->GetNeighbors(current_vertex);
  
  for (int neighbor : neighbors) {
    // Solo generar sucesores que no han sido explorados
    if (explored_.find(neighbor) == explored_.end()) {
      double edge_cost = graph_->GetEdgeCost(current_vertex, neighbor);
      double new_path_cost = node->GetPathCost() + edge_cost;
      int new_depth = node->GetDepth() + 1;
      
      std::shared_ptr<Node> successor = CreateNode(neighbor, node, new_path_cost, new_depth);
      successors.push_back(successor);
      
      // Marcar como generado
      MarkNodeGenerated(neighbor);
    }
  }
  
  return successors;
}

// Ejecuta la búsqueda en amplitud
SearchResult BFS::Search(int start, int goal) {
  // Reiniciar estructuras
  Reset();
  ResetBFS();
  
  SearchResult result;
  
  // Verificar vértices válidos
  if (!graph_->IsValidVertex(start) || !graph_->IsValidVertex(goal)) {
    std::cerr << "Error: Vértices inválidos. Start: " << start << ", Goal: " << goal << std::endl;
    return result;
  }
  
  // Crear nodo inicial
  std::shared_ptr<Node> start_node = CreateNode(start);
  frontier_.push(start_node);
  MarkNodeGenerated(start);
  
  // Primera iteración con solo el nodo inicial
  AddIteration();
  
  while (!frontier_.empty()) {
    // Tomar el primer nodo de la cola
    std::shared_ptr<Node> current_node = frontier_.front();
    frontier_.pop();
    
    int current_vertex = std::stoi(current_node->GetState());
    
    // Marcar como inspeccionado
    MarkNodeInspected(current_vertex);
    explored_.insert(current_vertex);
    
    // Verificar si es el objetivo
    if (current_vertex == goal) {
      result.path_found = true;
      result.path = ReconstructPath(current_node);
      result.total_cost = CalculatePathCost(result.path);
      result.iterations = iterations_;
      
      // Contar nodos generados e inspeccionados
      std::set<int> all_generated, all_inspected;
      for (const auto& iter : iterations_) {
        for (int node : iter.generated_nodes) {
          all_generated.insert(node);
        }
        for (int node : iter.inspected_nodes) {
          all_inspected.insert(node);
        }
      }
      result.nodes_generated = all_generated.size();
      result.nodes_inspected = all_inspected.size();
      
      return result;
    }
    
    // Expandir nodo actual
    std::vector<std::shared_ptr<Node>> successors = ExpandNode(current_node);
    
    // Agregar sucesores a la cola (solo los que no están en la frontera)
    for (std::shared_ptr<Node> successor : successors) {
      int successor_vertex = std::stoi(successor->GetState());
      
      // Verificar que no esté ya en la cola
      bool already_in_frontier = false;
      std::queue<std::shared_ptr<Node>> temp_queue = frontier_;
      while (!temp_queue.empty()) {
        if (std::stoi(temp_queue.front()->GetState()) == successor_vertex) {
          already_in_frontier = true;
          break;
        }
        temp_queue.pop();
      }
      
      if (!already_in_frontier) {
        frontier_.push(successor);
      }
    }
    
    // Agregar iteración si hay cambios
    if (!successors.empty() || !frontier_.empty()) {
      AddIteration();
    }
  }
  
  // No se encontró camino
  result.iterations = iterations_;
  
  // Contar nodos generados e inspeccionados
  std::set<int> all_generated, all_inspected;
  for (const auto& iter : iterations_) {
    for (int node : iter.generated_nodes) {
      all_generated.insert(node);
    }
    for (int node : iter.inspected_nodes) {
      all_inspected.insert(node);
    }
  }
  result.nodes_generated = all_generated.size();
  result.nodes_inspected = all_inspected.size();
  
  return result;
}