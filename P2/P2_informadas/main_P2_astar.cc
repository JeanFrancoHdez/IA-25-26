#include "maze.h"
#include "A_star.h"
#include "dynamic_environment.h"
#include <iostream>
#include <string>
#include <fstream>
#include <iomanip>
#include <cstdlib>

void SaveResultToFile(const AStarResult& result, const std::string& filename, 
                     const std::string& original_filename, const Position& start, 
                     const Position& goal, const Maze* maze) {
  std::ofstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Error: No se puede crear el archivo " << filename << std::endl;
    return;
  }
  
  // Confeccionar la tabla de resultados
  file << "Búsqueda A*. Función heurística h(·)\n";
  file << std::setw(12) << "Instancia" << std::setw(5) << "n" << std::setw(5) << "m" 
       << std::setw(8) << "S" << std::setw(8) << "E" << std::setw(15) << "Camino" 
       << std::setw(8) << "Coste" << std::setw(12) << "Nodos Gen." << std::setw(12) << "Nodos Insp." << "\n";
  
  file << std::setw(12) << original_filename;
  file << std::setw(5) << maze->GetRows();
  file << std::setw(5) << maze->GetCols();
  file << std::setw(8) << ("(" + std::to_string(start.row) + "," + std::to_string(start.col) + ")");
  file << std::setw(8) << ("(" + std::to_string(goal.row) + "," + std::to_string(goal.col) + ")");
  file << std::setw(15) << (result.path_found ? "Encontrado" : "No encontrado");
  file << std::setw(8) << std::fixed << std::setprecision(2) << result.total_cost;
  file << std::setw(12) << result.nodes_generated;
  file << std::setw(12) << result.nodes_inspected << "\n\n";
  
  file << "\nLaberinto estático resuelto.";
  
  // Capturar la salida del laberinto
  std::streambuf* cout_backup = std::cout.rdbuf();
  std::cout.rdbuf(file.rdbuf());
  
  if (result.path_found) {
    maze->Print(result.path);
  } else {
    maze->Print();
  }
  
  std::cout.rdbuf(cout_backup);
  
  // Información detallada por iteraciones
  file << "--------------------------------------\n";
  file << "Número de filas del mapa: " << maze->GetRows() << "\n";
  file << "Número de columnas del mapa: " << maze->GetCols() << "\n";
  file << "Punto de salida: (" << start.row << "," << start.col << ")\n";
  file << "Punto meta: (" << goal.row << "," << goal.col << ")\n";
  file << "--------------------------------------\n";
  
  for (const auto& iter : result.iteration_details) {
    file << "Iteración " << iter.iteration << "\n";
    
    file << "Nodos generados: ";
    for (size_t i = 0; i < iter.generated_nodes.size(); ++i) {
      file << "(" << iter.generated_nodes[i].row << "," << iter.generated_nodes[i].col << ")";
      if (i < iter.generated_nodes.size() - 1) file << ", ";
    }
    file << "\n";
    
    file << "Nodos inspeccionados: ";
    if (iter.inspected_nodes.empty()) {
      file << "-";
    } else {
      for (size_t i = 0; i < iter.inspected_nodes.size(); ++i) {
        file << "(" << iter.inspected_nodes[i].row << "," << iter.inspected_nodes[i].col << ")";
        if (i < iter.inspected_nodes.size() - 1) file << ", ";
      }
    }
    file << "\n";
    file << "--------------------------------------\n";
  }
  
  if (result.path_found) {
    file << "Camino: ";
    for (size_t i = 0; i < result.path.size(); ++i) {
      file << "(" << result.path[i].row << "," << result.path[i].col << ")";
      if (i < result.path.size() - 1) file << " - ";
    }
    file << "\n";
    file << "--------------------------------------\n";
    file << "Costo: " << std::fixed << std::setprecision(2) << result.total_cost << "\n";
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
  std::cout << "Uso: " << program_name << " <archivo_laberinto> <modo>" << std::endl;
  std::cout << "  archivo_laberinto: Archivo con el laberinto (formato especificado)" << std::endl;
  std::cout << "  modo: 0 para entorno estático, 1 para entorno dinámico" << std::endl;
  std::cout << std::endl;
  std::cout << "Ejemplos:" << std::endl;
  std::cout << "  " << program_name << " M_1.txt 0    # Ejecución estática" << std::endl;
  std::cout << "  " << program_name << " M_1.txt 1    # Ejecución dinámica" << std::endl;
}

int main(int argc, char* argv[]) {
  if (argc != 3) {
    ShowUsage(argv[0]);
    return 1;
  }
  
  std::string filename = argv[1];
  int mode = std::atoi(argv[2]);
  
  if (mode != 0 && mode != 1) {
    std::cerr << "Error: El modo debe ser 0 (estático) o 1 (dinámico)" << std::endl;
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
  
  char respuesta_entrada, respuesta_salida;
  
  std::cout << "¿Desea cambiar las coordenadas de entrada? (s/n): ";
  std::cin >> respuesta_entrada;
  
  if (respuesta_entrada == 's' || respuesta_entrada == 'S') {
    int start_row, start_col;
    std::cout << "Introduzca las coordenadas de entrada (fila columna): ";
    std::cin >> start_row >> start_col;
    
    bool is_on_border = (start_row == 0 || start_row == maze.GetRows() - 1 || start_col == 0 || start_col == maze.GetCols() - 1);
    
    if (maze.IsValidPosition(start_row, start_col) && is_on_border) {
      int new_value = maze.GetCell(start_row, start_col);
      
      maze.SetCell(start.row, start.col, new_value);
      maze.SetCell(start_row, start_col, Maze::START);
      
      start.row = start_row;
      start.col = start_col;
      std::cout << "Coordenadas de entrada actualizadas correctamente." << std::endl;
    } else {
      std::cout << "Coordenadas de entrada inválidas (debe estar en el borde del mapa). Se mantendrá la original." << std::endl;
    }
  }
  
  std::cout << "¿Desea cambiar las coordenadas de salida? (s/n): ";
  std::cin >> respuesta_salida;
  
  if (respuesta_salida == 's' || respuesta_salida == 'S') {
    int goal_row, goal_col;
    std::cout << "Introduzca las coordenadas de salida (fila columna): ";
    std::cin >> goal_row >> goal_col;
    
    bool is_on_border = (goal_row == 0 || goal_row == maze.GetRows() - 1 || goal_col == 0 || goal_col == maze.GetCols() - 1);
    
    if (maze.IsValidPosition(goal_row, goal_col) && is_on_border) {
      int new_value = maze.GetCell(goal_row, goal_col);
      
      maze.SetCell(goal.row, goal.col, new_value);
      maze.SetCell(goal_row, goal_col, Maze::END);
      
      goal.row = goal_row;
      goal.col = goal_col;
      std::cout << "Coordenadas de salida actualizadas correctamente." << std::endl;
    } else {
      std::cout << "Coordenadas de salida inválidas (debe estar en el borde del mapa). Se mantendrá la original." << std::endl;
    }
  }
  
  std::cout << std::endl;
  
  std::cout << "=== BÚSQUEDAS INFORMADAS - ALGORITMO A* ===" << std::endl;
  std::cout << "Archivo: " << filename << std::endl;
  std::cout << "Modo: " << (mode == 0 ? "Estático" : "Dinámico") << std::endl;
  std::cout << "Inicio: (" << start.row << "," << start.col << ")" << std::endl;
  std::cout << "Objetivo: (" << goal.row << "," << goal.col << ")" << std::endl;
  std::cout << std::string(50, '=') << std::endl;
  
  if (mode == 0) {
    AStar astar(&maze);
    AStarResult result = astar.Search(start, goal, false); // Sin verbose
    
    // Guardar información detallada en archivo
    std::string output_file = "resultado_estatico_" + filename;
    SaveResultToFile(result, output_file, filename, start, goal, &maze);
    
    std::cout << "\nInformación detallada guardada en: " << output_file << "\n" << std::endl;
    
  } else {
    double pin = 0.5;  // 50% probabilidad de convertir casilla en obstáculo
    double pout = 0.5; // 50% probabilidad de liberar un obstaculo
    
    DynamicEnvironment dynamic_env(&maze, pin, pout);
    
    // Ejecutar solo para generar el archivo (sin salida por pantalla)
    std::string output_file = "resultado_dinamico_" + filename;
    dynamic_env.ExecuteDynamicToFile(start, goal, output_file, filename);
    
    std::cout << "\nInformación detallada guardada en: " << output_file << "\n" << std::endl;
  }
  
  return 0;
}