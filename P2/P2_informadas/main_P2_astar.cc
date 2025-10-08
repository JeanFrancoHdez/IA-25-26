#include "map.h"
#include "A_star.h"
#include "dynamic_environment.h"
#include <iostream>
#include <string>
#include <limits>
#include <fstream>
#include <iomanip>
#include <cstdlib>
#include <fstream>
#include <iomanip>

class MazeAStarSystem {
public:
  MazeAStarSystem() : maze_(nullptr), astar_(nullptr), dynamic_env_(nullptr) {}
  
  ~MazeAStarSystem() {
    delete dynamic_env_;
    delete astar_;
    delete maze_;
  }
  
  void Run() {
    ShowWelcome();
    
    while (true) {
      ShowMainMenu();
      int choice = GetUserChoice();
      
      switch (choice) {
        case 1:
          LoadMazeFile();
          break;
        case 2:
          ChangeStartEnd();
          break;
        case 3:
          ExecuteStaticAStar();
          break;
        case 4:
          ExecuteDynamicAStar();
          break;
        case 5:
          ShowCurrentMaze();
          break;
        case 0:
          std::cout << "¡Gracias por usar el sistema A*!\n";
          return;
        default:
          std::cout << "Opción inválida. Intente nuevamente.\n";
    }
    }
  }

private:
  Maze* maze_;
  AStar* astar_;
  DynamicEnvironment* dynamic_env_;
  
  void ShowWelcome() {
        std::cout << "\n" << std::string(60, '=') << "\n";
        std::cout << "    SISTEMA A* PARA LABERINTOS CON ENTORNOS DINÁMICOS\n";
        std::cout << std::string(60, '=') << "\n";
        std::cout << "Implementación del algoritmo A* con heurística Manhattan\n";
        std::cout << "Costos: Horizontal/Vertical = 5, Diagonal = 7\n";
        std::cout << "Heurística: h(n) = (|xE - xn| + |yE - yn|) * W, W = 3\n\n";
  }
  
  void ShowMainMenu() {
        std::cout << "\n--- MENÚ PRINCIPAL ---\n";
        std::cout << "1. Cargar archivo de laberinto\n";
        std::cout << "2. Cambiar punto de entrada (S) y salida (E)\n";
        std::cout << "3. Ejecutar A* en entorno estático\n";
        std::cout << "4. Ejecutar A* en entorno dinámico\n";
        std::cout << "5. Mostrar laberinto actual\n";
        std::cout << "0. Salir\n";
        std::cout << "Seleccione una opción: ";
  }
  
  int GetUserChoice() {
    int choice;
    while (!(std::cin >> choice)) {
        std::cout << "Entrada inválida. Ingrese un número: ";
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    std::cin.ignore(); // Limpiar buffer
    return choice;
  }
  
  void LoadMazeFile() {
    std::string filename;
        std::cout << "\nIngrese el nombre del archivo de laberinto: ";
    std::getline(std::cin, filename);
    
    delete dynamic_env_;
    delete astar_;
    delete maze_;
    
    maze_ = new Maze();
    if (maze_->LoadFromFile(filename)) {
    astar_ = new AStar(maze_);
    dynamic_env_ = new DynamicEnvironment(maze_);
    
        std::cout << "¡Laberinto cargado exitosamente!\n";
        std::cout << "Dimensiones: " << maze_->GetRows() << "x" << maze_->GetCols() << "\n";
        std::cout << "Inicio (S): (" << maze_->GetStart().row << "," << maze_->GetStart().col << ")\n";
        std::cout << "Salida (E): (" << maze_->GetEnd().row << "," << maze_->GetEnd().col << ")\n";
    maze_->Print();
    } else {
    delete maze_;
    maze_ = nullptr;
        std::cout << "Error al cargar el laberinto.\n";
    }
  }
  
  void ChangeStartEnd() {
    if (!maze_) {
        std::cout << "Primero debe cargar un laberinto.\n";
        return;
    }
    
        std::cout << "\nLaberinto actual:\n";
    maze_->Print();
    
        std::cout << "\nNOTA: Los puntos S y E deben estar en los bordes del laberinto.\n";
    
    // Cambiar punto de inicio
        std::cout << "\n¿Desea cambiar el punto de inicio (S)? (s/n): ";
    char response;
    std::cin >> response;
    std::cin.ignore();
    
    if (response == 's' || response == 'S') {
    int row, col;
        std::cout << "Ingrese nueva posición para S (fila columna): ";
    std::cin >> row >> col;
    std::cin.ignore();
    
    Position new_start(row, col);
    if (maze_->IsValidPosition(row, col)) {
      maze_->SetStart(new_start);
      std::cout << "Punto de inicio actualizado.\n";
    } else {
      std::cout << "Posición inválida o no está en el borde.\n";
    }
    }
    
    // Cambiar punto de salida
        std::cout << "\n¿Desea cambiar el punto de salida (E)? (s/n): ";
    std::cin >> response;
    std::cin.ignore();
    
    if (response == 's' || response == 'S') {
    int row, col;
        std::cout << "Ingrese nueva posición para E (fila columna): ";
    std::cin >> row >> col;
    std::cin.ignore();
    
    Position new_end(row, col);
    if (maze_->IsValidPosition(row, col)) {
      maze_->SetEnd(new_end);
      std::cout << "Punto de salida actualizado.\n";
    } else {
      std::cout << "Posición inválida o no está en el borde.\n";
    }
    }
    
        std::cout << "\nLaberinto actualizado:\n";
    maze_->Print();
  }
  
  void ExecuteStaticAStar() {
    if (!CheckMazeLoaded()) return;
    
    Position start = maze_->GetStart();
    Position goal = maze_->GetEnd();
    
        std::cout << "\n=== EJECUTANDO A* EN ENTORNO ESTÁTICO ===\n";
    
    AStarResult result = astar_->Search(start, goal);
    
        std::cout << "\n" << astar_->GenerateReport(result, start, goal) << "\n";
    
    if (result.path_found) {
    maze_->Print(result.path);
    
    // Guardar resultado en archivo
        std::cout << "¿Desea guardar el resultado en un archivo? (s/n): ";
    char save_choice;
    std::cin >> save_choice;
    std::cin.ignore();
    
    if (save_choice == 's' || save_choice == 'S') {
      std::string filename = "resultado_astar_estatico.txt";
      SaveStaticResult(result, filename, start, goal);
      std::cout << "Resultado guardado en: " << filename << "\n";
    }
    }
  }
  
  void ExecuteDynamicAStar() {
    if (!CheckMazeLoaded()) return;
    
        std::cout << "\n=== CONFIGURACIÓN ENTORNO DINÁMICO ===\n";
    
    double pin, pout;
        std::cout << "Ingrese probabilidad pin (obstáculos nuevos) [0.0-1.0]: ";
    std::cin >> pin;
        std::cout << "Ingrese probabilidad pout (liberar obstáculos) [0.0-1.0]: ";
    std::cin >> pout;
    std::cin.ignore();
    
    dynamic_env_->SetProbabilities(pin, pout);
    
        std::cout << "¿Mostrar ejecución paso a paso? (s/n): ";
    char verbose;
    std::cin >> verbose;
    std::cin.ignore();
    
    bool show_verbose = (verbose == 's' || verbose == 'S');
    
    Position start = maze_->GetStart();
    Position goal = maze_->GetEnd();
    
    DynamicResult result = dynamic_env_->ExecuteDynamic(start, goal, show_verbose);
    
        std::cout << "\n" << dynamic_env_->GenerateCompleteReport(result, start, goal) << "\n";
    
    // Guardar resultado en archivo
        std::cout << "¿Desea guardar el resultado completo en un archivo? (s/n): ";
    char save_choice;
    std::cin >> save_choice;
    std::cin.ignore();
    
    if (save_choice == 's' || save_choice == 'S') {
    std::string filename = "resultado_astar_dinamico.txt";
    dynamic_env_->SaveResultsToFile(result, filename, start, goal);
        std::cout << "Resultado guardado en: " << filename << "\n";
    }
  }
  
  void ShowCurrentMaze() {
    if (!CheckMazeLoaded()) return;
    
        std::cout << "\nLaberinto actual:\n";
        std::cout << "Dimensiones: " << maze_->GetRows() << "x" << maze_->GetCols() << "\n";
        std::cout << "Inicio (S): (" << maze_->GetStart().row << "," << maze_->GetStart().col << ")\n";
        std::cout << "Salida (E): (" << maze_->GetEnd().row << "," << maze_->GetEnd().col << ")\n";
        std::cout << "Ratio de obstáculos: " << maze_->GetObstacleRatio() * 100 << "%\n";
    maze_->Print();
  }
  
  bool CheckMazeLoaded() {
    if (!maze_) {
        std::cout << "Primero debe cargar un laberinto (opción 1).\n";
    return false;
    }
    return true;
  }
  
  void SaveStaticResult(const AStarResult& result, const std::string& filename, 
       const Position& start, const Position& goal) {
    std::ofstream file(filename);
    if (!file.is_open()) return;
    
    file << "Búsqueda A*. Función heurística h(·)\n";
    file << std::setw(10) << "Instancia" << std::setw(5) << "n" << std::setw(5) << "m" 
     << std::setw(8) << "S" << std::setw(8) << "E" << std::setw(15) << "Camino" 
     << std::setw(8) << "Coste" << std::setw(12) << "Nodos Gen." << std::setw(12) << "Nodos Insp." << "\n";
    
    file << std::setw(10) << "M1";
    file << std::setw(5) << maze_->GetRows();
    file << std::setw(5) << maze_->GetCols();
    file << std::setw(8) << ("(" + std::to_string(start.row) + "," + std::to_string(start.col) + ")");
    file << std::setw(8) << ("(" + std::to_string(goal.row) + "," + std::to_string(goal.col) + ")");
    file << std::setw(15) << (result.path_found ? "Encontrado" : "No encontrado");
    file << std::setw(8) << std::fixed << std::setprecision(1) << result.total_cost;
    file << std::setw(12) << result.nodes_generated;
    file << std::setw(12) << result.nodes_inspected << "\n\n";
    
    file << astar_->GenerateReport(result, start, goal);
    file.close();
  }
};

// Función auxiliar para guardar resultado estático (fuera de la clase)
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
  
  // Añadir información detallada del resultado
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
};

void ShowUsage(const std::string& program_name) {
  std::cout << "Uso: " << program_name << " <archivo_laberinto> <modo>" << std::endl;
  std::cout << "  archivo_laberinto: Archivo con el laberinto (formato especificado)" << std::endl;
  std::cout << "  modo: 0 para entorno estático, 1 para entorno dinámico" << std::endl;
  std::cout << std::endl;
  std::cout << "Ejemplos:" << std::endl;
  std::cout << "  " << program_name << " M_1.txt 0    # Ejecución estática" << std::endl;
  std::cout << "  " << program_name << " M_1.txt 1    # Ejecución dinámica" << std::endl;
  std::cout << std::endl;
  std::cout << "Sin argumentos se ejecuta el modo interactivo." << std::endl;
}

int main(int argc, char* argv[]) {
  // Verificar argumentos de línea de comandos
  if (argc == 3) {
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
    double pin = 0.1;  // 10% probabilidad de crear obstáculo
    double pout = 0.1; // 10% probabilidad de liberar obstáculo
    
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
  
  // Sin argumentos suficientes
  if (argc == 2) {
    ShowUsage(argv[0]);
    return 1;
  }
  
  // Sin argumentos: modo interactivo
  MazeAStarSystem system;
  system.Run();
  
  return 0;
}