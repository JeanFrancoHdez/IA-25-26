#ifndef MAZE_H
#define MAZE_H

#include <vector>
#include <string>
#include <iostream>

struct Position {
  int row, col;
  Position(int r = 0, int c = 0) : row(r), col(c) {}
  bool operator==(const Position& other) const {
  return row == other.row && col == other.col;
  }
  bool operator!=(const Position& other) const {
  return !(*this == other);
  }
};

class Maze {
  public:
  // Valores de casillas del laberinto
  static const int FREE = 0;
  static const int OBSTACLE = 1;
  static const int START = 3;
  static const int END = 4;
  
  // Costos de movimiento
  static const int HORIZONTAL_COST = 5;
  static const int VERTICAL_COST = 5;
  static const int DIAGONAL_COST = 7;
  
  // Constructor y destructor
  Maze();
  ~Maze();
  
  // Carga y validación
  bool LoadFromFile(const std::string& filename);
  bool IsValid() const;
  
  // Acceso a datos
  int GetRows() const { return rows_; }
  int GetCols() const { return cols_; }
  int GetCell(int row, int col) const;
  void SetCell(int row, int col, int value);
  
  Position GetStart() const { return start_; }
  Position GetEnd() const { return end_; }
  void SetStart(const Position& pos);
  void SetEnd(const Position& pos);
  
  // Navegación
  bool IsValidPosition(int row, int col) const;
  bool IsFree(int row, int col) const;
  std::vector<Position> GetNeighbors(const Position& pos) const;
  int GetMovementCost(const Position& from, const Position& to) const;
  
  // Heurística Manhattan
  double ManhattanHeuristic(const Position& pos, double weight = 3.0) const;
  
  // Visualización
  void Print(const std::vector<Position>& path = {}) const;
  void PrintWithStep(const Position& current_pos, const std::vector<Position>& path = {}) const;
  
  // Entorno dinámico
  void UpdateDynamicEnvironment(double pin, double pout);
  double GetObstacleRatio() const;
  
  // Utilidades
  Position IndexToPosition(int index) const;
  int PositionToIndex(const Position& pos) const;
  
private:
  int rows_, cols_;
  std::vector<std::vector<int>> grid_;
  Position start_, end_;
  
  // Validación de entrada
  bool ValidateStartEnd() const;
  bool IsOnBorder(const Position& pos) const;
  void EnsureObstacleLimit();
};

#endif