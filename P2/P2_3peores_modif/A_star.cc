#include "A_star.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <random>
#include <ctime>

AStar::AStar(Maze* maze, char heuristic) 
  : maze_(maze), heuristic_(heuristic), nodes_generated_(0), nodes_inspected_(0), iterations_(0) {}

AStar::~AStar() {}

AStarResult AStar::Search(const Position& start, const Position& goal, bool verbose) {
  Reset();
  
  AStarResult result;
  
  if (!maze_ || !maze_->IsValidPosition(start.row, start.col) || !maze_->IsValidPosition(goal.row, goal.col)) {
    return result;
  }
  
  // Crear nodo inicial
  double h_start = CalculateHeuristic(start);
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
  
    IterationInfo iter_info;
    iter_info.iteration = iterations_;
    
    // Capturar nodos generados
    std::vector<Position> generated_nodes;
    for (const auto& pair : open_map_) {
      generated_nodes.push_back(pair.second->GetPosition());
    }
    for (const auto& pair : closed_list_) {
      generated_nodes.push_back(pair.second->GetPosition());
    }
    iter_info.generated_nodes = generated_nodes;
    
    // Capturar nodos inspeccionados
    std::vector<Position> inspected_nodes;
    for (const auto& pair : closed_list_) {
      inspected_nodes.push_back(pair.second->GetPosition());
    }
    iter_info.inspected_nodes = inspected_nodes;
  
    auto current = open_list_.top();
    open_list_.pop();
  
    Position current_pos = current->GetPosition();
    int current_key = PositionToKey(current_pos);
  
    open_map_.erase(current_key);
    closed_list_[current_key] = current;
    nodes_inspected_++;
    
    result.iteration_details.push_back(iter_info);
  
    if (verbose) {
      std::cout << "Iteración " << iterations_ << ": Inspeccionando (" 
                << current_pos.row << "," << current_pos.col 
                << ") f=" << std::fixed << std::setprecision(1) << current->GetFCost()
                << " g=" << current->GetGCost() << " h=" << current->GetHCost() << std::endl;
    }
  
    if (current_pos == goal) {
      result.path_found = true;
      result.path = ReconstructPath(current);
      result.total_cost = current->GetGCost();
      result.nodes_generated = nodes_generated_;
      result.nodes_inspected = nodes_inspected_;
      result.iterations = iterations_;
  
      if (verbose) {
        std::cout << "¡Camino encontrado! Coste total: " << result.total_cost << std::endl;
      }
      return result;
    }
  
    auto neighbors = maze_->GetNeighbors(current_pos);
    for (const auto& neighbor_pos : neighbors) {
      int neighbor_key = PositionToKey(neighbor_pos);
  
      if (IsInClosedList(neighbor_pos)) {
        continue;
      }
  
      double movement_cost = maze_->GetMovementCost(current_pos, neighbor_pos);
      double new_g_cost = current->GetGCost() + movement_cost;
  
      if (!IsInOpenList(neighbor_pos)) {
        double h_cost = CalculateHeuristic(neighbor_pos);
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
        auto existing_node = GetFromOpenList(neighbor_pos);
        if (existing_node && new_g_cost < existing_node->GetGCost()) {
          existing_node->SetGCost(new_g_cost);
          existing_node->SetParent(current);
  
          if (verbose) {
            std::cout << "  Actualizando (" << neighbor_pos.row << "," << neighbor_pos.col 
                      << ") nuevo g=" << new_g_cost << " f=" << existing_node->GetFCost() << std::endl;
          }
        }
      }
    }
  }
  
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

double AStar::CalculateHeuristic(const Position& pos) const {
  if (heuristic_ == 'b') {
    return maze_->EuclideanHeuristic(pos);
  }
  return maze_->ManhattanHeuristic(pos);
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

AStarResult AStar::Search3Worst(const Position& start, const Position& goal, bool verbose) {
  Reset();
  
  AStarResult result;
  
  if (!maze_ || !maze_->IsValidPosition(start.row, start.col) || !maze_->IsValidPosition(goal.row, goal.col)) {
    return result;
  }
  
  // Generador de números aleatorios
  static std::mt19937 rng(std::time(nullptr));
  
  // Crear nodo inicial
  double h_start = CalculateHeuristic(start);
  auto start_node = std::make_shared<AStarNode>(start, nullptr, 0.0, h_start);
  
  open_list_.push(start_node);
  open_map_[PositionToKey(start)] = start_node;
  nodes_generated_++;
  
  if (verbose) {
    std::cout << "Iniciando A* (3 peores) desde (" << start.row << "," << start.col 
              << ") hacia (" << goal.row << "," << goal.col << ")" << std::endl;
  }
  
  while (!open_list_.empty()) {
    iterations_++;
  
    IterationInfo iter_info;
    iter_info.iteration = iterations_;
    
    // Capturar nodos generados
    std::vector<Position> generated_nodes;
    for (const auto& pair : open_map_) {
      generated_nodes.push_back(pair.second->GetPosition());
    }
    for (const auto& pair : closed_list_) {
      generated_nodes.push_back(pair.second->GetPosition());
    }
    iter_info.generated_nodes = generated_nodes;
    
    // Capturar nodos inspeccionados
    std::vector<Position> inspected_nodes;
    for (const auto& pair : closed_list_) {
      inspected_nodes.push_back(pair.second->GetPosition());
    }
    iter_info.inspected_nodes = inspected_nodes;
  
    // MODIFICACIÓN: Extraer los 3 nodos con PEOR coste (mayor f(n))
    std::vector<std::shared_ptr<AStarNode>> worst_nodes;
    std::priority_queue<std::shared_ptr<AStarNode>, 
                       std::vector<std::shared_ptr<AStarNode>>, 
                       AStarNodeComparator> temp_list = open_list_;
    
    // Extraer todos los nodos en un vector para encontrar los 3 peores
    std::vector<std::shared_ptr<AStarNode>> all_nodes;
    while (!temp_list.empty()) {
      all_nodes.push_back(temp_list.top());
      temp_list.pop();
    }
    
    // Los peores son los que tienen mayor f(n), están al final si ordenamos
    std::sort(all_nodes.begin(), all_nodes.end(), 
              [](const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) {
                return a->GetFCost() > b->GetFCost();
              });
    
    // Tomar los 3 primeros (o menos si no hay suficientes)
    int count = std::min(3, static_cast<int>(all_nodes.size()));
    for (int i = 0; i < count; ++i) {
      worst_nodes.push_back(all_nodes[i]);
    }
    
    // Seleccionar uno al azar entre los 3 peores
    std::uniform_int_distribution<int> dist(0, worst_nodes.size() - 1);
    int random_index = dist(rng);
    auto current = worst_nodes[random_index];
    
    // Eliminar el nodo seleccionado de open_list
    // Como no podemos eliminar fácilmente de la priority_queue, reconstruimos
    std::priority_queue<std::shared_ptr<AStarNode>, 
                       std::vector<std::shared_ptr<AStarNode>>, 
                       AStarNodeComparator> new_open_list;
    for (const auto& node : all_nodes) {
      if (node != current) {
        new_open_list.push(node);
      }
    }
    open_list_ = new_open_list;
  
    Position current_pos = current->GetPosition();
    int current_key = PositionToKey(current_pos);
  
    open_map_.erase(current_key);
    closed_list_[current_key] = current;
    nodes_inspected_++;
    
    result.iteration_details.push_back(iter_info);
  
    if (verbose) {
      std::cout << "Iteración " << iterations_ << ": Inspeccionando (" 
                << current_pos.row << "," << current_pos.col 
                << ") f=" << std::fixed << std::setprecision(1) << current->GetFCost()
                << " g=" << current->GetGCost() << " h=" << current->GetHCost() << std::endl;
    }
  
    if (current_pos == goal) {
      result.path_found = true;
      result.path = ReconstructPath(current);
      result.total_cost = current->GetGCost();
      result.nodes_generated = nodes_generated_;
      result.nodes_inspected = nodes_inspected_;
      result.iterations = iterations_;
  
      if (verbose) {
        std::cout << "¡Camino encontrado! Coste total: " << result.total_cost << std::endl;
      }
      return result;
    }
  
    auto neighbors = maze_->GetNeighbors(current_pos);
    for (const auto& neighbor_pos : neighbors) {
      int neighbor_key = PositionToKey(neighbor_pos);
  
      if (IsInClosedList(neighbor_pos)) {
        continue;
      }
  
      double movement_cost = maze_->GetMovementCost(current_pos, neighbor_pos);
      double new_g_cost = current->GetGCost() + movement_cost;
  
      if (!IsInOpenList(neighbor_pos)) {
        double h_cost = CalculateHeuristic(neighbor_pos);
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
        auto existing_node = GetFromOpenList(neighbor_pos);
        if (existing_node && new_g_cost < existing_node->GetGCost()) {
          existing_node->SetGCost(new_g_cost);
          existing_node->SetParent(current);
  
          if (verbose) {
            std::cout << "  Actualizando (" << neighbor_pos.row << "," << neighbor_pos.col 
                      << ") nuevo g=" << new_g_cost << " f=" << existing_node->GetFCost() << std::endl;
          }
        }
      }
    }
  }
  
  result.nodes_generated = nodes_generated_;
  result.nodes_inspected = nodes_inspected_;
  result.iterations = iterations_;
  
  if (verbose) {
    std::cout << "No se encontró camino al objetivo" << std::endl;
  }
  return result;
}