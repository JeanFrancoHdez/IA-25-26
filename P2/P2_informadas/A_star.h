#ifndef ASTAR_H
#define ASTAR_H

#include "maze.h"
#include "A_star_node.h"
#include <queue>
#include <unordered_map>
#include <vector>
#include <string>

struct IterationInfo {
  int iteration;
  std::vector<Position> generated_nodes;
  std::vector<Position> inspected_nodes;
};

struct AStarResult {
  bool path_found;
  std::vector<Position> path;
  double total_cost;
  int nodes_generated;
  int nodes_inspected;
  int iterations;
  std::vector<IterationInfo> iteration_details;
  
  AStarResult() : path_found(false), total_cost(0.0), nodes_generated(0), nodes_inspected(0), iterations(0) {}
};

class AStar {
  public:
    AStar(Maze* maze);
    ~AStar();
  
    AStarResult Search(const Position& start, const Position& goal, bool verbose = true);
  
  private:
    Maze* maze_;
  
    std::priority_queue<std::shared_ptr<AStarNode>, 
    std::vector<std::shared_ptr<AStarNode>>, 
    AStarNodeComparator> open_list_;
    std::unordered_map<int, std::shared_ptr<AStarNode>> closed_list_;
    std::unordered_map<int, std::shared_ptr<AStarNode>> open_map_;
  
    int nodes_generated_;
    int nodes_inspected_;
    int iterations_;
  
    // Funciones auxiliares
    std::vector<Position> ReconstructPath(std::shared_ptr<AStarNode> goal_node) const;
    double CalculatePathCost(const std::vector<Position>& path) const;
    void Reset();
    int PositionToKey(const Position& pos) const;
    bool IsInOpenList(const Position& pos) const;
    bool IsInClosedList(const Position& pos) const;
    std::shared_ptr<AStarNode> GetFromOpenList(const Position& pos);
    void UpdateOpenList(std::shared_ptr<AStarNode> node);
};

#endif