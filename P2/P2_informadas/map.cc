#include "map.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <random>
#include <iomanip>

Maze::Maze() : rows_(0), cols_(0), start_(0, 0), end_(0, 0) {}

Maze::~Maze() {}

bool Maze::LoadFromFile(const std::string& filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
  std::cerr << "Error: No se puede abrir el archivo " << filename << std::endl;
  return false;
  }
  
  // Leer dimensiones
  file >> rows_ >> cols_;
  if (rows_ <= 0 || cols_ <= 0) {
  std::cerr << "Error: Dimensiones inválidas" << std::endl;
  return false;
  }
  
  // Inicializar grid
  grid_.resize(rows_, std::vector<int>(cols_));
  
  // Leer laberinto
  bool start_found = false, end_found = false;
  for (int i = 0; i < rows_; ++i) {
    for (int j = 0; j < cols_; ++j) {
      file >> grid_[i][j];
      
      if (grid_[i][j] == START) {
        if (start_found) {
          std::cerr << "Error: Múltiples puntos de inicio encontrados" << std::endl;
          return false;
        }
        start_ = Position(i, j);
        start_found = true;
      } else if (grid_[i][j] == END) {
        if (end_found) {
          std::cerr << "Error: Múltiples puntos de salida encontrados" << std::endl;
          return false;
        }
        end_ = Position(i, j);
        end_found = true;
      }
    }
  }
  
  file.close();
  
  // Validaciones
  if (!start_found || !end_found) {
  std::cerr << "Error: No se encontraron puntos de inicio y/o salida" << std::endl;
  return false;
  }
  
  return IsValid();
}

bool Maze::IsValid() const {
  return ValidateStartEnd();
}

bool Maze::ValidateStartEnd() const {
  // Verificar que start y end están en los bordes
  if (!IsOnBorder(start_)) {
  std::cerr << "Error: El punto de inicio debe estar en el borde del laberinto" << std::endl;
  return false;
  }
  
  if (!IsOnBorder(end_)) {
  std::cerr << "Error: El punto de salida debe estar en el borde del laberinto" << std::endl;
  return false;
  }
  
  return true;
}

bool Maze::IsOnBorder(const Position& pos) const {
  return pos.row == 0 || pos.row == rows_ - 1 || pos.col == 0 || pos.col == cols_ - 1;
}

int Maze::GetCell(int row, int col) const {
  if (!IsValidPosition(row, col)) return OBSTACLE;
  return grid_[row][col];
}

void Maze::SetCell(int row, int col, int value) {
  if (IsValidPosition(row, col)) {
  grid_[row][col] = value;
  }
}

void Maze::SetStart(const Position& pos) {
  if (IsValidPosition(pos.row, pos.col) && IsOnBorder(pos)) {
  // Limpiar anterior start
  if (IsValidPosition(start_.row, start_.col)) {
  grid_[start_.row][start_.col] = FREE;
  }
  start_ = pos;
  grid_[pos.row][pos.col] = START;
  }
}

void Maze::SetEnd(const Position& pos) {
  if (IsValidPosition(pos.row, pos.col) && IsOnBorder(pos)) {
  // Limpiar anterior end
  if (IsValidPosition(end_.row, end_.col)) {
  grid_[end_.row][end_.col] = FREE;
  }
  end_ = pos;
  grid_[pos.row][pos.col] = END;
  }
}

bool Maze::IsValidPosition(int row, int col) const {
  return row >= 0 && row < rows_ && col >= 0 && col < cols_;
}

bool Maze::IsFree(int row, int col) const {
  int cell = GetCell(row, col);
  return cell == FREE || cell == START || cell == END;
}

std::vector<Position> Maze::GetNeighbors(const Position& pos) const {
  std::vector<Position> neighbors;
  
  // 8 direcciones: horizontal, vertical y diagonal
  std::vector<std::pair<int, int>> directions = {
  {-1, 0}, {1, 0}, {0, -1}, {0, 1},        // vertical y horizontal
  {-1, -1}, {-1, 1}, {1, -1}, {1, 1}       // diagonal
  };
  
  for (const auto& dir : directions) {
  int new_row = pos.row + dir.first;
  int new_col = pos.col + dir.second;
  
  if (IsValidPosition(new_row, new_col) && IsFree(new_row, new_col)) {
  neighbors.push_back(Position(new_row, new_col));
  }
  }
  
  return neighbors;
}

int Maze::GetMovementCost(const Position& from, const Position& to) const {
  int row_diff = abs(to.row - from.row);
  int col_diff = abs(to.col - from.col);
  
  if (row_diff == 1 && col_diff == 1) {
  return DIAGONAL_COST;  // Movimiento diagonal
  } else if ((row_diff == 1 && col_diff == 0) || (row_diff == 0 && col_diff == 1)) {
  return (row_diff == 1) ? VERTICAL_COST : HORIZONTAL_COST;
  }
  
  return 0; // No debería llegar aquí
}

double Maze::ManhattanHeuristic(const Position& pos, double weight) const {
  return (abs(end_.row - pos.row) + abs(end_.col - pos.col)) * weight;
}

void Maze::Print(const std::vector<Position>& path) const {
  // Crear copia para marcar el camino
  auto display_grid = grid_;
  
  // Marcar camino con asteriscos
  for (const auto& pos : path) {
  if (pos != start_ && pos != end_) {
  display_grid[pos.row][pos.col] = -1; // Marcador especial para camino
  }
  }
  
  std::cout << "\nLaberinto actual:\n";
  for (int i = 0; i < rows_; ++i) {
  for (int j = 0; j < cols_; ++j) {
  if (display_grid[i][j] == -1) {
    std::cout << " * ";
  } else if (display_grid[i][j] == START) {
    std::cout << " S ";
  } else if (display_grid[i][j] == END) {
    std::cout << " E ";
  } else if (display_grid[i][j] == OBSTACLE) {
    std::cout << " # ";
  } else {
    std::cout << "   ";
  }
  }
  std::cout << std::endl;
  }
  std::cout << std::endl;
}

void Maze::PrintWithStep(const Position& current_pos, const std::vector<Position>& path) const {
  auto display_grid = grid_;
  
  // Marcar camino
  for (const auto& pos : path) {
  if (pos != start_ && pos != end_) {
  display_grid[pos.row][pos.col] = -1;
  }
  }
  
  // Marcar posición actual
  if (current_pos != start_ && current_pos != end_) {
  display_grid[current_pos.row][current_pos.col] = -2; // Posición actual
  }
  
  std::cout << "\nPaso actual (A = agente, * = camino):\n";
  for (int i = 0; i < rows_; ++i) {
  for (int j = 0; j < cols_; ++j) {
  if (display_grid[i][j] == -2) {
    std::cout << " A ";
  } else if (display_grid[i][j] == -1) {
    std::cout << " * ";
  } else if (display_grid[i][j] == START) {
    std::cout << " S ";
  } else if (display_grid[i][j] == END) {
    std::cout << " E ";
  } else if (display_grid[i][j] == OBSTACLE) {
    std::cout << " # ";
  } else {
    std::cout << "   ";
  }
  }
  std::cout << std::endl;
  }
  std::cout << std::endl;
}

void Maze::UpdateDynamicEnvironment(double pin, double pout) {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.0, 1.0);
  
  // Actualizar cada casilla (excepto S y E)
  for (int i = 0; i < rows_; ++i) {
  for (int j = 0; j < cols_; ++j) {
  Position pos(i, j);
  
  // No modificar S ni E
  if (pos == start_ || pos == end_) {
    continue;
  }
  
  double random_val = dis(gen);
  
  if (grid_[i][j] == FREE) {
    // Casilla libre -> puede convertirse en obstáculo
    if (random_val < pin) {
  grid_[i][j] = OBSTACLE;
    }
  } else if (grid_[i][j] == OBSTACLE) {
    // Obstáculo -> puede liberarse
    if (random_val < pout) {
  grid_[i][j] = FREE;
    }
  }
  }
  }
  
  // Asegurar que no se exceda el 25% de obstáculos
  EnsureObstacleLimit();
}

double Maze::GetObstacleRatio() const {
  int obstacle_count = 0;
  int total_cells = rows_ * cols_;
  
  for (int i = 0; i < rows_; ++i) {
  for (int j = 0; j < cols_; ++j) {
  if (grid_[i][j] == OBSTACLE) {
    obstacle_count++;
  }
  }
  }
  
  return static_cast<double>(obstacle_count) / total_cells;
}

void Maze::EnsureObstacleLimit() {
  const double MAX_OBSTACLE_RATIO = 0.25;
  
  while (GetObstacleRatio() > MAX_OBSTACLE_RATIO) {
  // Encontrar un obstáculo aleatorio y eliminarlo
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::uniform_int_distribution<> row_dis(0, rows_ - 1);
  std::uniform_int_distribution<> col_dis(0, cols_ - 1);
  
  int row = row_dis(gen);
  int col = col_dis(gen);
  
  Position pos(row, col);
  if (grid_[row][col] == OBSTACLE && pos != start_ && pos != end_) {
  grid_[row][col] = FREE;
  }
  }
}

Position Maze::IndexToPosition(int index) const {
  return Position(index / cols_, index % cols_);
}

int Maze::PositionToIndex(const Position& pos) const {
  return pos.row * cols_ + pos.col;
}