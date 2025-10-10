#include "maze.h"
#include "A_star.h"
#include "dynamic_environment.h"
#include <iostream>
#include <string>
#include <fstream>
#include <iomanip>
#include <cstdlib>

void SaveStaticResultToFile(const AStarResult& result, const std::string& filename, 
                           const Position& start, const Position& goal, const Maze* maze) {
  std::ofstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Error: No se puede crear el archivo " << filename << std::endl;
    return;
  }
  
  file << "Búsqueda A*. Función heurística h(·)\n";
  file << std::setw(10) << "Instancia" << std::setw(5) << "n" << std::setw(5) << "m" 
       << std::setw(8) << "S" << std::setw(8) << "E" << std::setw(15) << "Camino" 
       << std::setw(8) << "Coste" << std::setw(12) << "Nodos Gen." << std::setw(12) << "Nodos Insp." << "\n";
  
  file << std::setw(10) << "M1";
  file << std::setw(5) << maze->GetRows();
  file << std::setw(5) << maze->GetCols();
  file << std::setw(8) << ("(" + std::to_string(start.row) + "," + std::to_string(start.col) + ")");
  file << std::setw(8) << ("(" + std::to_string(goal.row) + "," + std::to_string(goal.col) + ")");
  file << std::setw(15) << (result.path_found ? "Encontrado" : "No encontrado");
  file << std::setw(8) << std::fixed << std::setprecision(1) << result.total_cost;
  file << std::setw(12) << result.nodes_generated;
  file << std::setw(12) << result.nodes_inspected << "\n\n";
  
  // Información detallada del resultado
  file << "=== Información Detallada ===\n";
  file << "Laberinto: " << maze->GetRows() << "x" << maze->GetCols() << "\n";
  file << "Punto de inicio: (" << start.row << "," << start.col << ")\n";
  file << "Punto objetivo: (" << goal.row << "," << goal.col << ")\n";
  file << "Resultado: " << (result.path_found ? "Camino encontrado" : "No se encontró camino") << "\n";
  
  if (result.path_found) {
    file << "Costo total: " << std::fixed << std::setprecision(1) << result.total_cost << "\n";
    file << "Longitud del camino: " << result.path.size() << " casillas\n";
    
    file << "Camino completo: ";
    for (size_t i = 0; i < result.path.size(); ++i) {
      file << "(" << result.path[i].row << "," << result.path[i].col << ")";
      if (i < result.path.size() - 1) file << " -> ";
      if ((i + 1) % 8 == 0 && i < result.path.size() - 1) file << "\n                 ";
    }
    file << "\n";
  }
  
  file << "Estadísticas de búsqueda:\n";
  file << "- Nodos generados: " << result.nodes_generated << "\n";
  file << "- Nodos inspeccionados: " << result.nodes_inspected << "\n";
  file << "- Iteraciones realizadas: " << result.iterations << "\n";
  
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
  // Verificar argumentos de línea de comandos
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
  
  // Cargar laberinto
  Maze maze;
  if (!maze.LoadFromFile(filename)) {
    std::cerr << "Error al cargar el laberinto desde " << filename << std::endl;
    return 1;
  }
  
  Position start = maze.GetStart();
  Position goal = maze.GetEnd();
  
  std::cout << "=== ALGORITMO A* PARA LABERINTOS ===" << std::endl;
  std::cout << "Archivo: " << filename << std::endl;
  std::cout << "Modo: " << (mode == 0 ? "Estático" : "Dinámico") << std::endl;
  std::cout << "Inicio: (" << start.row << "," << start.col << ")" << std::endl;
  std::cout << "Objetivo: (" << goal.row << "," << goal.col << ")" << std::endl;
  std::cout << std::string(50, '=') << std::endl;
  
  if (mode == 0) {
    // MODO ESTÁTICO
    AStar astar(&maze);
    AStarResult result = astar.Search(start, goal, false); // Sin verbose
    
    // Guardar información detallada en archivo
    std::string output_file = "resultado_estatico_" + filename;
    SaveStaticResultToFile(result, output_file, start, goal, &maze);
    
    std::cout << "\nResultado:" << std::endl;
    if (result.path_found) {
      std::cout << "Camino encontrado - Costo: " << result.total_cost << std::endl;
      std::cout << "Información detallada guardada en: " << output_file << std::endl;
      std::cout << "\nLaberinto resuelto:" << std::endl;
      maze.Print(result.path);
    } else {
      std::cout << "No se encontró camino al objetivo" << std::endl;
    }
    
  } else {
    // MODO DINÁMICO
    // Configuración por defecto para dinámico
    double pin = 0.5;  // 50% probabilidad de crear obstáculo
    double pout = 0.5; // 50% probabilidad de liberar obstáculo
    
    DynamicEnvironment dynamic_env(&maze, pin, pout);
    DynamicResult result = dynamic_env.ExecuteDynamic(start, goal, true); // Con visualización
    
    // Guardar información detallada en archivo
    std::string output_file = "resultado_dinamico_" + filename;
    dynamic_env.SaveResultsToFile(result, output_file, start, goal);
    
    std::cout << "\n=== RESUMEN FINAL ===" << std::endl;
    std::cout << "Éxito: " << (result.success ? "SÍ" : "NO") << std::endl;
    if (result.success) {
      std::cout << "Pasos realizados: " << result.total_steps << std::endl;
      std::cout << "Costo total: " << result.total_cost << std::endl;
    }
    std::cout << "Replaneaciones: " << result.total_replans << std::endl;
    std::cout << "Información detallada guardada en: " << output_file << std::endl;
  }
  
  return 0;
}