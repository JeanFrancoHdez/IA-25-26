#ifndef DYNAMIC_ENVIRONMENT_H
#define DYNAMIC_ENVIRONMENT_H

#include "maze.h"
#include "A_star.h"
#include <vector>
#include <string>

struct DynamicResult {
  bool success;
  std::vector<Position> complete_path;  // Camino completo realizado
  double total_cost;
  int total_steps;
  int failed_attempts;                  // Intentos fallidos consecutivos
  std::vector<AStarResult> individual_searches;  // Resultados de cada búsqueda A*
  std::vector<double> obstacle_ratios;  // Ratio de obstáculos en cada paso
  
  DynamicResult() : success(false), total_cost(0.0), total_steps(0), failed_attempts(0) {}
};

class DynamicEnvironment {
  public:
    DynamicEnvironment(Maze* maze, double pin = 0.1, double pout = 0.1);
    ~DynamicEnvironment();
  
    // Ejecutar A* en entorno dinámico
    DynamicResult ExecuteDynamic(const Position& start, const Position& goal, bool verbose = true);
  
    // Configuración
    void SetProbabilities(double pin, double pout);
    void SetMaxFailedAttempts(int max_attempts) { max_failed_attempts_ = max_attempts; }
  
    // Generar reporte completo
    std::string GenerateCompleteReport(const DynamicResult& result, const Position& start, const Position& goal) const;
  
    // Guardar resultados en archivo
    bool SaveResultsToFile(const DynamicResult& result, const std::string& filename, const Position& start, const Position& goal) const;

  private:
    Maze* maze_;
    AStar* astar_;
    double pin_;                // Probabilidad de que casilla libre se convierta en obstáculo
    double pout_;               // Probabilidad de que obstáculo se libere
    int max_failed_attempts_;   // Máximo número de intentos fallidos consecutivos
  
    // Ejecutar un paso del agente
    bool ExecuteStep(Position& current_pos, const std::vector<Position>& planned_path, size_t& step_index, DynamicResult& result);
};

#endif