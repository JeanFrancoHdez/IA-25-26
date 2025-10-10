#include "A_star.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>

AStar::AStar(Maze* maze) : maze_(maze), nodes_generated_(0), nodes_inspected_(0), iterations_(0) {}

AStar::~AStar() {}

AStarResult AStar::Search(const Position& start, const Position& goal, bool verbose) {
  Reset();
  
  AStarResult result;
  
  if (!maze_ || !maze_->IsValidPosition(start.row, start.col) || !maze_->IsValidPosition(goal.row, goal.col)) {
    return result;
  }
  
  // Crear nodo inicial
  double h_start = maze_->ManhattanHeuristic(start);
  auto start_node = std::make_shared<AStarNode>(start, nullptr, 0.0, h_start);
  
  open_list_.push(start_node);
  open_map_[PositionToKey(start)] = start_node;
  nodes_generated_++;
  
  if (verbose) {
    std::cout << "Iniciando A* desde (" << start.row << "," << start.col 
              << ") hacia (" << goal.row << "," << goal.col << ")" << std::endl;
  }
  
  while (!open_list_.empty()) {
    iterations_++;
  
    // Seleccionar nodo with menor f(n)
    auto current = open_list_.top();
    open_list_.pop();
  
    Position current_pos = current->GetPosition();
    int current_key = PositionToKey(current_pos);
  
    // Remover de open_map y añadir a closed_list
    open_map_.erase(current_key);
    closed_list_[current_key] = current;
    nodes_inspected_++;
  
    if (verbose) {
      std::cout << "Iteración " << iterations_ << ": Inspeccionando (" 
                << current_pos.row << "," << current_pos.col 
                << ") f=" << std::fixed << std::setprecision(1) << current->GetFCost()
                << " g=" << current->GetGCost() << " h=" << current->GetHCost() << std::endl;
    }
  
    // Verificar si hemos llegado al objetivo
    if (current_pos == goal) {
      result.path_found = true;
      result.path = ReconstructPath(current);
      result.total_cost = current->GetGCost();
      result.nodes_generated = nodes_generated_;
      result.nodes_inspected = nodes_inspected_;
      result.iterations = iterations_;
  
      if (verbose) {
        std::cout << "¡Camino encontrado! Costo total: " << result.total_cost << std::endl;
      }
      return result;
    }
  
    // Expandir vecinos
    auto neighbors = maze_->GetNeighbors(current_pos);
    for (const auto& neighbor_pos : neighbors) {
      int neighbor_key = PositionToKey(neighbor_pos);
  
      // Si ya está en la lista cerrada, ignorar
      if (IsInClosedList(neighbor_pos)) {
        continue;
      }
  
      // Calcular nuevo g(n)
      double movement_cost = maze_->GetMovementCost(current_pos, neighbor_pos);
      double new_g_cost = current->GetGCost() + movement_cost;
  
      // Si no está en la lista abierta, añadirlo
      if (!IsInOpenList(neighbor_pos)) {
        double h_cost = maze_->ManhattanHeuristic(neighbor_pos);
        auto neighbor_node = std::make_shared<AStarNode>(neighbor_pos, current, new_g_cost, h_cost);
    
        open_list_.push(neighbor_node);
        open_map_[neighbor_key] = neighbor_node;
        nodes_generated_++;
    
        if (verbose) {
          std::cout << "  Generando (" << neighbor_pos.row << "," << neighbor_pos.col 
                    << ") f=" << neighbor_node->GetFCost() 
                    << " g=" << new_g_cost << " h=" << h_cost << std::endl;
        }
      } else {
        // Si ya está en la lista abierta, verificar si este camino es mejor
        auto existing_node = GetFromOpenList(neighbor_pos);
        if (existing_node && new_g_cost < existing_node->GetGCost()) {
          existing_node->SetGCost(new_g_cost);
          existing_node->SetParent(current);
  
          if (verbose) {
            std::cout << "  Actualizando (" << neighbor_pos.row << "," << neighbor_pos.col 
                      << ") nuevo g=" << new_g_cost << " f=" << existing_node->GetFCost() << std::endl;
          }
  
          // Nota: En una implementación más eficiente, deberíamos reorganizar la priority_queue
          // Por simplicidad, mantenemos el nodo actualizado en open_map_
        }
      }
    }
  }
  
  // No se encontró camino
  result.nodes_generated = nodes_generated_;
  result.nodes_inspected = nodes_inspected_;
  result.iterations = iterations_;
  
  if (verbose) {
    std::cout << "No se encontró camino al objetivo" << std::endl;
  }
  return result;
}

std::vector<Position> AStar::ReconstructPath(std::shared_ptr<AStarNode> goal_node) const {
  std::vector<Position> path;
  auto current = goal_node;
  
  while (current != nullptr) {
    path.push_back(current->GetPosition());
    current = current->GetParent();
  }
  
  std::reverse(path.begin(), path.end());
  return path;
}

double AStar::CalculatePathCost(const std::vector<Position>& path) const {
  if (path.size() < 2) return 0.0;
  
  double cost = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    cost += maze_->GetMovementCost(path[i-1], path[i]);
  }
  return cost;
}

void AStar::Reset() {
  // Limpiar estructuras
  while (!open_list_.empty()) {
    open_list_.pop();
  }
  open_map_.clear();
  closed_list_.clear();
  
  // Reiniciar contadores
  nodes_generated_ = 0;
  nodes_inspected_ = 0;
  iterations_ = 0;
}

int AStar::PositionToKey(const Position& pos) const {
  return pos.row * maze_->GetCols() + pos.col;
}

bool AStar::IsInOpenList(const Position& pos) const {
  return open_map_.find(PositionToKey(pos)) != open_map_.end();
}

bool AStar::IsInClosedList(const Position& pos) const {
  return closed_list_.find(PositionToKey(pos)) != closed_list_.end();
}

std::shared_ptr<AStarNode> AStar::GetFromOpenList(const Position& pos) {
  auto it = open_map_.find(PositionToKey(pos));
  return (it != open_map_.end()) ? it->second : nullptr;
}

std::string AStar::GenerateReport(const AStarResult& result, const Position& start, const Position& goal) const {
  std::ostringstream oss;
  oss << "=== Reporte A* ===\n";
  oss << "Desde: (" << start.row << "," << start.col << ")\n";
  oss << "Hasta: (" << goal.row << "," << goal.col << ")\n";
  oss << "Camino encontrado: " << (result.path_found ? "Sí" : "No") << "\n";
  
  if (result.path_found) {
    oss << "Costo total: " << std::fixed << std::setprecision(1) << result.total_cost << "\n";
    oss << "Longitud del camino: " << result.path.size() << " casillas\n";
    oss << "Camino: ";
    for (size_t i = 0; i < result.path.size(); ++i) {
      oss << "(" << result.path[i].row << "," << result.path[i].col << ")";
      if (i < result.path.size() - 1) oss << " -> ";
    }
    oss << "\n";
  }
  
  oss << "Nodos generados: " << result.nodes_generated << "\n";
  oss << "Nodos inspeccionados: " << result.nodes_inspected << "\n";
  oss << "Iteraciones: " << result.iterations << "\n";
  
  return oss.str();
}