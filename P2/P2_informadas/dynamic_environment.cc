#include "dynamic_environment.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>

DynamicEnvironment::DynamicEnvironment(Maze* maze, double pin, double pout)
  : maze_(maze), astar_(new AStar(maze)), pin_(pin), pout_(pout), max_failed_attempts_(5) {
}

DynamicEnvironment::~DynamicEnvironment() {
  delete astar_;
}

void DynamicEnvironment::SetProbabilities(double pin, double pout) {
  pin_ = pin;
  pout_ = pout;
}

DynamicResult DynamicEnvironment::ExecuteDynamic(const Position& start, const Position& goal, bool verbose) {
  DynamicResult result;
  Position current_pos = start;
  result.complete_path.push_back(current_pos);
  
  if (verbose) {
    std::cout << "\n=== INICIANDO EJECUCIÓN EN ENTORNO DINÁMICO ===\n";
    std::cout << "Desde: (" << start.row << "," << start.col << ")\n";
    std::cout << "Hasta: (" << goal.row << "," << goal.col << ")\n";
    std::cout << "Probabilidades: pin = " << pin_ << ", pout = " << pout_ << "\n\n";
  }
  
  int consecutive_failures = 0;
  
  while (!(current_pos == goal) && consecutive_failures < max_failed_attempts_) {
    if (verbose) {
      std::cout << "\n--- PLANIFICACIÓN " << (result.total_steps + 1) << " ---\n";
      std::cout << "Posición actual: (" << current_pos.row << "," << current_pos.col << ")\n";
    }
  
    result.obstacle_ratios.push_back(maze_->GetObstacleRatio());
  
    // Ejecución de A* desde posición actual
    AStarResult search_result = astar_->Search(current_pos, goal, false);
    result.individual_searches.push_back(search_result);
  
    if (!search_result.path_found) {
      consecutive_failures++;
      if (verbose) {
        std::cout << "¡No se encontró camino! Intento fallido #" << consecutive_failures << "\n";
        std::cout << "Actualizando entorno y reintentando...\n";
      }
  
      if (consecutive_failures >= max_failed_attempts_) {
        if (verbose) {
          std::cout << "¡Se alcanzó el máximo de intentos fallidos! (" << max_failed_attempts_ << ")\n";
        }
        result.failed_attempts = consecutive_failures;
        break;
      }
  
      maze_->UpdateDynamicEnvironment(pin_, pout_);
      continue;
    }
  
    // Camino encontrado, resetear contador de fallos
    consecutive_failures = 0;
  
    if (verbose) {
      std::cout << "Camino encontrado. Longitud: " << search_result.path.size() 
                << ", Coste: " << search_result.total_cost << "\n";
      std::cout << "Iniciando navegación paso a paso...\n";
    }
  
    if (search_result.path.size() > 1) {
      Position next_pos = search_result.path[1];
      double step_cost = maze_->GetMovementCost(current_pos, next_pos);

      current_pos = next_pos;
      result.complete_path.push_back(current_pos);
      result.total_cost += step_cost;
      result.total_steps++;

      if (verbose) {
        std::cout << "\n--- MOVIMIENTO " << result.total_steps << " ---\n";
        std::cout << "Posición actualizada: (" << current_pos.row << "," << current_pos.col << ")\n";
        std::cout << "Coste del paso: " << step_cost << "\n";
        std::cout << "Coste acumulado: " << std::fixed << std::setprecision(1) << result.total_cost << "\n";
        maze_->PrintWithStep(current_pos, result.complete_path);
      }

      if (current_pos == goal) {
        result.success = true;
        if (verbose) {
          std::cout << "¡OBJETIVO ALCANZADO!\n";
        }
        break;
      }

      maze_->UpdateDynamicEnvironment(pin_, pout_);
      
      if (verbose) {
        std::cout << "Entorno actualizado. Replanteando desde posición actual...\n";
      }
    }
  }
  
  result.failed_attempts = consecutive_failures;
  return result;
}