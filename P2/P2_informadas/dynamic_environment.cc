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
  
    // Ejecutar A* desde posición actual (modo silencioso para entorno dinámico)
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
  
      // Actualizar entorno y continuar
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
  
    // Ejecutar solo el primer paso del camino planificado
    if (search_result.path.size() > 1) {
      // Mover al siguiente paso (índice 1, ya que 0 es la posición actual)
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
        maze_->PrintWithStep(current_pos, search_result.path);
      }

      // Si llegamos al objetivo, terminar
      if (current_pos == goal) {
        result.success = true;
        if (verbose) {
          std::cout << "¡OBJETIVO ALCANZADO!\n";
        }
        break;
      }

      // Actualizar entorno dinámico después del paso
      maze_->UpdateDynamicEnvironment(pin_, pout_);
      
      if (verbose) {
        std::cout << "Entorno actualizado. Replanteando desde posición actual...\n";
      }
    }
  }
  
  result.failed_attempts = consecutive_failures;
  

  
  return result;
}

std::string DynamicEnvironment::GenerateCompleteReport(const DynamicResult& result, const Position& start, const Position& goal) const {
  std::ostringstream oss;
  
  oss << "=== REPORTE COMPLETO - ENTORNO DINÁMICO ===\n\n";
  oss << "Configuración:\n";
  oss << "  Inicio: (" << start.row << "," << start.col << ")\n";
  oss << "  Objetivo: (" << goal.row << "," << goal.col << ")\n";
  oss << "  Probabilidad pin: " << pin_ << "\n";
  oss << "  Probabilidad pout: " << pout_ << "\n\n";
  
  oss << "Resultados generales:\n";
  oss << "  Éxito: " << (result.success ? "SÍ" : "NO") << "\n";
  oss << "  Pasos realizados: " << result.total_steps << "\n";
  oss << "  Costo total acumulado: " << std::fixed << std::setprecision(1) << result.total_cost << "\n\n";
  
  // Métricas agregadas
  int total_nodes_generated = 0;
  int total_nodes_inspected = 0;
  double avg_obstacle_ratio = 0.0;
  
  for (const auto& search : result.individual_searches) {
    total_nodes_generated += search.nodes_generated;
    total_nodes_inspected += search.nodes_inspected;
  }
  
  for (double ratio : result.obstacle_ratios) {
    avg_obstacle_ratio += ratio;
  }
  avg_obstacle_ratio /= result.obstacle_ratios.size();
  
  oss << "Métricas totales:\n";
  oss << "  Nodos generados (total): " << total_nodes_generated << "\n";
  oss << "  Nodos inspeccionados (total): " << total_nodes_inspected << "\n";
  oss << "  Proporción media de obstáculos: " << std::fixed << std::setprecision(3) << avg_obstacle_ratio * 100 << "%\n\n";
  
  oss << "Detalle por iteración:\n";
  for (size_t i = 0; i < result.individual_searches.size(); ++i) {
    const auto& search = result.individual_searches[i];
    oss << "  Iteración " << (i + 1) << ":\n";
    oss << "    Camino encontrado: " << (search.path_found ? "Sí" : "No") << "\n";
    if (search.path_found) {
      oss << "    Costo: " << search.total_cost << "\n";
      oss << "    Longitud: " << search.path.size() << " casillas\n";
    }
    oss << "    Nodos generados: " << search.nodes_generated << "\n";
    oss << "    Nodos inspeccionados: " << search.nodes_inspected << "\n";
    if (i < result.obstacle_ratios.size()) {
      oss << "    Ratio obstáculos: " << std::fixed << std::setprecision(1) << result.obstacle_ratios[i] * 100 << "%\n";
    }
    oss << "\n";
  }
  
  // Camino completo realizado
  if (result.success && !result.complete_path.empty()) {
    oss << "Camino completo realizado:\n  ";
    for (size_t i = 0; i < result.complete_path.size(); ++i) {
      oss << "(" << result.complete_path[i].row << "," << result.complete_path[i].col << ")";
      if (i < result.complete_path.size() - 1) oss << " -> ";
      if ((i + 1) % 8 == 0 && i < result.complete_path.size() - 1) oss << "\n  ";
    }
    oss << "\n";
  }
  
  return oss.str();
}

bool DynamicEnvironment::SaveResultsToFile(const DynamicResult& result, const std::string& filename, const Position& start, const Position& goal) const {
  std::ofstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Error: No se puede crear el archivo " << filename << std::endl;
    return false;
  }
  
  // Escribir tabla de resultados
  file << "Búsqueda A*. Función heurística h(·)\n";
  file << std::setw(10) << "Instancia" << std::setw(5) << "n" << std::setw(5) << "m" 
       << std::setw(8) << "S" << std::setw(8) << "E" << std::setw(15) << "Camino" 
       << std::setw(8) << "Coste" << std::setw(12) << "Nodos Gen." << std::setw(12) << "Nodos Insp." << "\n";
  
  for (size_t i = 0; i < result.individual_searches.size(); ++i) {
    const auto& search = result.individual_searches[i];
    file << std::setw(10) << ("M" + std::to_string(i+1));
    file << std::setw(5) << maze_->GetRows();
    file << std::setw(5) << maze_->GetCols();
    file << std::setw(8) << ("(" + std::to_string(start.row) + "," + std::to_string(start.col) + ")");
    file << std::setw(8) << ("(" + std::to_string(goal.row) + "," + std::to_string(goal.col) + ")");
    file << std::setw(15) << (search.path_found ? "Encontrado" : "No encontrado");
    file << std::setw(8) << std::fixed << std::setprecision(1) << search.total_cost;
    file << std::setw(12) << search.nodes_generated;
    file << std::setw(12) << search.nodes_inspected << "\n";
  }
  
  file << "\n" << GenerateCompleteReport(result, start, goal);
  file.close();
  
  return true;
}