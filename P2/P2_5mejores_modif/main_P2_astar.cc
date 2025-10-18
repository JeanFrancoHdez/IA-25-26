#include "maze.h"
#include "A_star.h"
#include "dynamic_environment.h"
#include <iostream>
#include <string>
#include <fstream>
#include <iomanip>
#include <cstdlib>

void SwapStartOrEnd(Maze& maze, Position& current_pos, int cell_type, const std::string& type_name) {
  char respuesta;
  std::cout << "¿Desea cambiar las coordenadas de " << type_name << "? (s/n): ";
  std::cin >> respuesta;
  
  if (respuesta == 's' || respuesta == 'S') {
    int new_row, new_col;
    std::cout << "Introduzca las coordenadas de " << type_name << " (fila columna): ";
    std::cin >> new_row >> new_col;
    
    bool is_on_border = (new_row == 0 || new_row == maze.GetRows() - 1 || new_col == 0 || new_col == maze.GetCols() - 1);
    
    if (maze.IsValidPosition(new_row, new_col) && is_on_border) {
      Position new_pos(new_row, new_col);
      
      // SWAP: Guardar el valor de la celda destino
      int temp_value = maze.GetCell(new_row, new_col);
      
      // Poner el valor de la celda destino en la posición antigua
      maze.SetCell(current_pos.row, current_pos.col, temp_value);
      
      // Poner START o END en la nueva posición
      if (cell_type == Maze::START) {
        maze.SetStart(new_pos);
      } else {
        maze.SetEnd(new_pos);
      }
      
      // Actualizar la variable de posición
      current_pos.row = new_row;
      current_pos.col = new_col;
      std::cout << "Coordenadas de " << type_name << " actualizadas correctamente." << std::endl;
    } else {
      std::cout << "Coordenadas de " << type_name << " inválidas (debe estar en el borde del mapa). Se mantendrá la original." << std::endl;
    }
  }
}

void SaveResultToFile(const AStarResult& result, const std::string& filename, const Position& start, const Position& goal, const Maze* maze) {
  std::ofstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Error: No se puede crear el archivo " << filename << std::endl;
    return;
  }
  
  file << "Laberinto estatico resuelto.\n";
  
  // Escribir el laberinto directamente al archivo
  if (result.path_found) {
    maze->Print(result.path, file);
  } else {
    maze->Print({}, file);
  }
  
  file << "--------------------------------------\n";
  file << "Número de filas del mapa: " << maze->GetRows() << "\n";
  file << "Número de columnas del mapa: " << maze->GetCols() << "\n";
  file << "Punto de salida: (" << start.row << "," << start.col << ")\n";
  file << "Punto meta: (" << goal.row << "," << goal.col << ")\n";
  file << "--------------------------------------\n";
  
  if (result.path_found) {
    file << "Camino: ";
    for (size_t i = 0; i < result.path.size(); ++i) {
      file << "(" << result.path[i].row << "," << result.path[i].col << ")";
      if (i < result.path.size() - 1) file << " - ";
    }
    file << "\n";
    file << "--------------------------------------\n";
    file << "Coste: " << std::fixed << std::setprecision(2) << result.total_cost << "\n";
    file << "--------------------------------------\n\n";
  }
  
  file << "=== RESULTADOS GENERALES ===\n";
  file << "Resultado: " << (result.path_found ? "Camino encontrado" : "No se encontró camino") << "\n";
  if (result.path_found) {
    file << "Coste total: " << std::fixed << std::setprecision(2) << result.total_cost << "\n";
    file << "Longitud del camino: " << result.path.size() << " casillas\n";
  }
  file << "Nodos generados: " << result.nodes_generated << "\n";
  file << "Nodos inspeccionados: " << result.nodes_inspected << "\n";
  
  int static_steps = result.path_found ? (result.path.size() > 0 ? result.path.size() - 1 : 0) : 0;
  file << "Pasos realizados: " << static_steps << "\n";
  
  file.close();
}

void ShowUsage(const std::string& program_name) {
  std::cout << "Uso: " << program_name << " <archivo_laberinto> <modo> <heuristica>" << std::endl;
  std::cout << "  archivo_laberinto: Archivo con el laberinto (formato especificado)" << std::endl;
  std::cout << "  modo: 0 para entorno estático, 1 para entorno dinámico" << std::endl;
  std::cout << "  heuristica: a para Manhattan, b para Euclidiana" << std::endl;
  std::cout << std::endl;
  std::cout << "Ejemplos:" << std::endl;
  std::cout << "  " << program_name << " M_1.txt 0 a    # Ejecución estática con Manhattan" << std::endl;
  std::cout << "  " << program_name << " M_1.txt 1 b    # Ejecución dinámica con Euclidiana" << std::endl;
}

int main(int argc, char* argv[]) {
  if (argc != 4) {
    ShowUsage(argv[0]);
    return 1;
  }
  
  std::string filename = argv[1];
  int mode = std::atoi(argv[2]);
  char heuristic = argv[3][0];
  
  if (mode != 0 && mode != 1) {
    std::cerr << "Error: El modo debe ser 0 (estático) o 1 (dinámico)" << std::endl;
    ShowUsage(argv[0]);
    return 1;
  }
  
  if (heuristic != 'a' && heuristic != 'b') {
    std::cerr << "Error: La heurística debe ser 'a' (Manhattan) o 'b' (Euclidiana)" << std::endl;
    ShowUsage(argv[0]);
    return 1;
  }
  
  Maze maze;
  if (!maze.LoadFromFile(filename)) {
    std::cerr << "Error al cargar el laberinto desde " << filename << std::endl;
    return 1;
  }
  
  Position start = maze.GetStart();
  Position goal = maze.GetEnd();
  
  // Permitir cambiar coordenadas de entrada y salida
  SwapStartOrEnd(maze, start, Maze::START, "entrada");
  SwapStartOrEnd(maze, goal, Maze::END, "salida");
  
  std::cout << std::endl;
  
  std::cout << "=== BÚSQUEDAS INFORMADAS - ALGORITMO A* ===" << std::endl;
  std::cout << "Archivo: " << filename << std::endl;
  std::cout << "Modo: " << (mode == 0 ? "Estático" : "Dinámico") << std::endl;
  std::cout << "Inicio: (" << start.row << "," << start.col << ")" << std::endl;
  std::cout << "Objetivo: (" << goal.row << "," << goal.col << ")" << std::endl;
  std::cout << std::string(50, '=') << std::endl;
  
  if (mode == 0) {
    AStar astar(&maze, heuristic);
    AStarResult result = astar.Search(start, goal, false);
    
    // Guardar información detallada en archivo
    std::string output_file = "resultado_estatico_" + filename;
    SaveResultToFile(result, output_file, start, goal, &maze);
    
    std::cout << "\nInformación detallada guardada en: " << output_file << "\n" << std::endl;
    
  } else {
    double pin = 0.5;  // Probabilidad de convertir casilla en obstáculo
    double pout = 0.5; // Probabilidad de liberar un obstaculo
    
    DynamicEnvironment dynamic_env(&maze, pin, pout, heuristic);
    
    std::string output_file = "resultado_dinamico_" + filename;
    dynamic_env.ExecuteDynamicToFile(start, goal, output_file, filename);
    
    std::cout << "\nInformación detallada guardada en: " << output_file << "\n" << std::endl;
  }
  
  return 0;
}