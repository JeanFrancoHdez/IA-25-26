#ifndef DYNAMIC_ENVIRONMENT_H
#define DYNAMIC_ENVIRONMENT_H

#include "maze.h"
#include "A_star.h"
#include <vector>
#include <string>
#include <fstream>

struct DynamicResult {
  bool success;
  std::vector<Position> complete_path;
  double total_cost;
  int total_steps;
  int failed_attempts;
  std::vector<AStarResult> individual_searches;
  std::vector<double> obstacle_ratios;
  
  DynamicResult() : success(false), total_cost(0.0), total_steps(0), failed_attempts(0) {}
};

class DynamicEnvironment {
  public:
    DynamicEnvironment(Maze* maze, double pin, double pout, char heuristic = 'a');
    ~DynamicEnvironment();
  
    DynamicResult ExecuteDynamicToFile(const Position& start, const Position& goal, const std::string& filename, const std::string& original_filename);

  private:
    Maze* maze_;
    AStar* astar_;
    double pin_;                // Probabilidad de que casilla libre se convierta en obstáculo
    double pout_;               // Probabilidad de que obstáculo se libere
    int max_failed_attempts_;
  
    bool ExecuteStep(Position& current_pos, const std::vector<Position>& planned_path, size_t& step_index, DynamicResult& result);
};

#endif