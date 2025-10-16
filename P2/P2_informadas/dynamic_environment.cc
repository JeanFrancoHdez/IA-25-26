#include "dynamic_environment.h"
#include <iostream>
#include <iomanip>
#include <fstream>

DynamicEnvironment::DynamicEnvironment(Maze* maze, double pin, double pout, char heuristic)
  : maze_(maze), astar_(new AStar(maze, heuristic)), pin_(pin), pout_(pout), max_failed_attempts_(5) {
}

DynamicEnvironment::~DynamicEnvironment() {
  delete astar_;
}

DynamicResult DynamicEnvironment::ExecuteDynamicToFile(const Position& start, const Position& goal, const std::string& filename, const std::string& original_filename) {
  std::ofstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Error: No se puede crear el archivo " << filename << std::endl;
    return DynamicResult();
  }

  file << "Busqueda A*. Funcion heuristica h(.)\n";
  file << std::setw(12) << "Instancia" << std::setw(5) << "n" << std::setw(5) << "m" 
       << std::setw(8) << "S" << std::setw(8) << "E" << std::setw(15) << "Camino" 
       << std::setw(8) << "Coste" << std::setw(12) << "Nodos Gen." << std::setw(12) << "Nodos Insp." << "\n";

  Position current_pos = start;
  DynamicResult file_result;
  file_result.complete_path.push_back(current_pos);
  int consecutive_failures = 0;
  
  while (!(current_pos == goal) && consecutive_failures < max_failed_attempts_) {
    file_result.obstacle_ratios.push_back(maze_->GetObstacleRatio());
    
    AStarResult search_result = astar_->Search(current_pos, goal, false);
    file_result.individual_searches.push_back(search_result);
    
    if (!search_result.path_found) {
      consecutive_failures++;
      if (consecutive_failures >= max_failed_attempts_) {
        file_result.failed_attempts = consecutive_failures;
        break;
      }
      maze_->UpdateDynamicEnvironment(pin_, pout_);
      continue;
    } else {
      consecutive_failures = 0;
    }
    
    if (search_result.path.size() > 1) {
      Position next_pos = search_result.path[1];
      double step_cost = maze_->GetMovementCost(current_pos, next_pos);
      
      current_pos = next_pos;
      file_result.complete_path.push_back(current_pos);
      file_result.total_cost += step_cost;
      file_result.total_steps++;
      
      if (current_pos == goal) {
        file_result.success = true;
        break;
      }
      
      maze_->UpdateDynamicEnvironment(pin_, pout_);
    }
  }
  
  int total_generated = 0, total_inspected = 0;
  for (const auto& search : file_result.individual_searches) {
    total_generated += search.nodes_generated;
    total_inspected += search.nodes_inspected;
  }
  
  // Escribir tabla
  file << std::setw(12) << original_filename;
  file << std::setw(5) << maze_->GetRows();
  file << std::setw(5) << maze_->GetCols();
  file << std::setw(8) << ("(" + std::to_string(start.row) + "," + std::to_string(start.col) + ")");
  file << std::setw(8) << ("(" + std::to_string(goal.row) + "," + std::to_string(goal.col) + ")");
  file << std::setw(15) << (file_result.success ? "Encontrado" : "No encontrado");
  file << std::setw(8) << std::fixed << std::setprecision(2) << file_result.total_cost;
  file << std::setw(12) << total_generated;
  file << std::setw(12) << total_inspected << "\n\n";

  // Reiniciar el laberinto desde su estado original para la ejecución detallada
  maze_->LoadFromFile(original_filename);

  file << "=== BÚSQUEDAS INFORMADAS - ALGORITMO A* ===\n";
  file << "Archivo: " << original_filename << "\n";
  file << "Modo: Dinámico\n";
  file << "Inicio: (" << start.row << "," << start.col << ")\n";
  file << "Objetivo: (" << goal.row << "," << goal.col << ")\n";
  file << std::string(50, '=') << "\n\n";
  
  file << "=== INICIANDO EJECUCIÓN EN ENTORNO DINÁMICO ===\n";
  file << "Desde: (" << start.row << "," << start.col << ")\n";
  file << "Hasta: (" << goal.row << "," << goal.col << ")\n";
  file << "Probabilidades: pin = " << pin_ << ", pout = " << pout_ << "\n";

  Position current_position = start;
  DynamicResult detailed_result;
  detailed_result.complete_path.push_back(current_position);
  int failures = 0;
  
  while (!(current_position == goal) && failures < max_failed_attempts_) {
    file << "\n--- PLANIFICACIÓN " << (detailed_result.total_steps + 1) << " ---\n";
    file << "Posición actual: (" << current_position.row << "," << current_position.col << ")\n";
    
    detailed_result.obstacle_ratios.push_back(maze_->GetObstacleRatio());
    
    AStarResult search_result = astar_->Search(current_position, goal, false);
    detailed_result.individual_searches.push_back(search_result);
    
    if (!search_result.path_found) {
      failures++;
      file << "¡No se encontró camino! Intento fallido #" << failures << "\n";
      file << "Actualizando entorno y reintentando...\n";
      
      if (failures >= max_failed_attempts_) {
        file << "¡Se alcanzó el máximo de intentos fallidos! (" << max_failed_attempts_ << ")\n";
        detailed_result.failed_attempts = failures;
        break;
      }
      
      maze_->UpdateDynamicEnvironment(pin_, pout_);
      continue;
    } else {
      failures = 0;
    }
    
    file << "Camino encontrado. Longitud: " << search_result.path.size() 
         << ", Coste: " << search_result.total_cost << "\n";
    file << "Iniciando navegación paso a paso...\n";
    
    if (search_result.path.size() > 1) {
      Position next_pos = search_result.path[1];
      double step_cost = maze_->GetMovementCost(current_position, next_pos);
      
      current_position = next_pos;
      detailed_result.complete_path.push_back(current_position);
      detailed_result.total_cost += step_cost;
      detailed_result.total_steps++;
      
      file << "\n--- MOVIMIENTO " << detailed_result.total_steps << " ---\n";
      file << "Posición actualizada: (" << current_position.row << "," << current_position.col << ")\n";
      file << "Coste del paso: " << step_cost << "\n";
      file << "Coste acumulado: " << std::fixed << std::setprecision(1) << detailed_result.total_cost << "\n";
      file << "Número de reintentos sin éxito: " << failures << "\n";
      file << "Proporción media de obstáculos dinámicos: " << std::fixed << std::setprecision(1) 
           << (maze_->GetObstacleRatio() * 100) << "%\n";
      file << "Número de nodos generados: " << search_result.nodes_generated << "\n";
      file << "Número de nodos inspeccionados: " << search_result.nodes_inspected << "\n";
      
      // Mostrar nodos generados e inspeccionados
      file << "--------------------------------------\n";
      if (!search_result.iteration_details.empty()) {
        auto& last_iteration = search_result.iteration_details.back();
        file << "Nodos generados: ";
        for (size_t i = 0; i < last_iteration.generated_nodes.size(); ++i) {
          file << "(" << last_iteration.generated_nodes[i].row << "," << last_iteration.generated_nodes[i].col << ")";
          if (i < last_iteration.generated_nodes.size() - 1) file << ", ";
        }
        file << std::endl;
        
        file << "Nodos inspeccionados: ";
        if (last_iteration.inspected_nodes.empty()) {
          file << "-";
        } else {
          for (size_t i = 0; i < last_iteration.inspected_nodes.size(); ++i) {
            file << "(" << last_iteration.inspected_nodes[i].row << "," << last_iteration.inspected_nodes[i].col << ")";
            if (i < last_iteration.inspected_nodes.size() - 1) file << ", ";
          }
        }
        file << std::endl;
      }
      file << "--------------------------------------\n";
      
      // Escribir el mapa directamente al archivo
      maze_->PrintWithTwoPaths(current_position, detailed_result.complete_path, search_result.path, file);
      
      if (current_position == goal) {
        detailed_result.success = true;
        file << "OBJETIVO ALCANZADO!\n";
        break;
      }
      
      maze_->UpdateDynamicEnvironment(pin_, pout_);
      file << "Entorno actualizado. Replanteando desde posicion actual...\n";
    }
  }

  if (file_result.success) {
    file << "--------------------------------------\n";
    file << "Camino: ";
    for (size_t i = 0; i < detailed_result.complete_path.size(); ++i) {
      file << "(" << detailed_result.complete_path[i].row << "," << detailed_result.complete_path[i].col << ")";
      if (i < detailed_result.complete_path.size() - 1) file << " - ";
    }
    file << "\n--------------------------------------\n";
    file << "Costo: " << std::fixed << std::setprecision(2) << detailed_result.total_cost << "\n";
    file << "--------------------------------------\n\n";
  }
  
  double avg_obstacle_ratio = 0.0;
  if (!detailed_result.obstacle_ratios.empty()) {
    for (double ratio : detailed_result.obstacle_ratios) {
      avg_obstacle_ratio += ratio;
    }
    avg_obstacle_ratio /= detailed_result.obstacle_ratios.size();
  }
  
  file << "=== RESULTADOS GENERALES ===\n";
  file << "Resultado: " << (detailed_result.success ? "Camino encontrado" : "No se encontró camino") << "\n";
  if (file_result.success) {
    file << "Coste total: " << std::fixed << std::setprecision(2) << file_result.total_cost << "\n";
    file << "Longitud del camino: " << file_result.complete_path.size() << " casillas\n";
  }
  file << "Nodos generados: " << total_generated << "\n";
  file << "Nodos inspeccionados: " << total_inspected << "\n";
  file << "Pasos realizados: " << detailed_result.total_steps << "\n";
  file << "Número de reintentos sin éxito: " << detailed_result.failed_attempts << "\n";
  file << "Proporción media de obstáculos: " << std::fixed << std::setprecision(1) 
       << (avg_obstacle_ratio * 100) << "%\n";
  
  file.close();
  return detailed_result;
}