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

  file << "=== BUSQUEDAS INFORMADAS - ALGORITMO A* ===\n";
  file << "Archivo: " << original_filename << "\n";
  file << "Modo: Dinamico\n";
  file << "Inicio: (" << start.row << "," << start.col << ")\n";
  file << "Objetivo: (" << goal.row << "," << goal.col << ")\n";
  file << std::string(50, '=') << "\n\n";
  
  file << "=== INICIANDO EJECUCION EN ENTORNO DINAMICO ===\n";
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
    
    file << "Camino encontrado. Longitud: " << search_result.path.size() << "\n";
    file << "--------------------------------------\n";
    file << "Camino planeado en esta iteración: ";
    for (size_t i = 0; i < search_result.path.size(); ++i) {
      file << "(" << search_result.path[i].row << "," << search_result.path[i].col << ")";
      if (i < search_result.path.size() - 1) file << " - ";
    }
    file << "\n";
    file << "Coste del camino planeado en esta iteración: " << std::fixed << std::setprecision(2) << search_result.total_cost << "\n";
    file << "--------------------------------------\n";
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

  if (detailed_result.success) {
    file << "--------------------------------------\n";
    file << "Camino final: ";
    for (size_t i = 0; i < detailed_result.complete_path.size(); ++i) {
      file << "(" << detailed_result.complete_path[i].row << "," << detailed_result.complete_path[i].col << ")";
      if (i < detailed_result.complete_path.size() - 1) file << " - ";
    }
    file << "\n--------------------------------------\n";
    file << "Coste: " << std::fixed << std::setprecision(2) << detailed_result.total_cost << "\n";
    file << "--------------------------------------\n\n";
  }
  
  // Calcular totales
  int total_generated = 0, total_inspected = 0;
  for (const auto& search : detailed_result.individual_searches) {
    total_generated += search.nodes_generated;
    total_inspected += search.nodes_inspected;
  }
  
  double avg_obstacle_ratio = 0.0;
  if (!detailed_result.obstacle_ratios.empty()) {
    for (double ratio : detailed_result.obstacle_ratios) {
      avg_obstacle_ratio += ratio;
    }
    avg_obstacle_ratio /= detailed_result.obstacle_ratios.size();
  }
  
  file << "=== RESULTADOS GENERALES ===\n";
  file << "Resultado: " << (detailed_result.success ? "Camino encontrado" : "No se encontro camino") << "\n";
  if (detailed_result.success) {
    file << "Coste total: " << std::fixed << std::setprecision(2) << detailed_result.total_cost << "\n";
    file << "Longitud del camino: " << detailed_result.complete_path.size() << " casillas\n";
  }
  file << "Nodos generados: " << total_generated << "\n";
  file << "Nodos inspeccionados: " << total_inspected << "\n";
  file << "Pasos realizados: " << detailed_result.total_steps << "\n";
  file << "Numero de reintentos sin exito: " << detailed_result.failed_attempts << "\n";
  file << "Proporcion media de obstaculos: " << std::fixed << std::setprecision(1) 
       << (avg_obstacle_ratio * 100) << "%\n";
  
  file.close();
  return detailed_result;
}