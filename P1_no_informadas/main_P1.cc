#include <iostream>
#include <string>
#include <memory>
#include "graph.h"
#include "bfs.h"
#include "dfs.h"

/**
 * @brief Función para mostrar el uso del programa
 */
void ShowUsage(const std::string& program_name) {
  std::cout << "Uso: " << program_name << " <archivo_grafo> <vertice_origen> <vertice_destino> [algoritmo]" << std::endl;
  std::cout << "  archivo_grafo: Archivo con el formato específico del grafo" << std::endl;
  std::cout << "  vertice_origen: Vértice de inicio (1-index)" << std::endl;
  std::cout << "  vertice_destino: Vértice objetivo (1-index)" << std::endl;
  std::cout << "  algoritmo: 'bfs' para búsqueda en amplitud, 'dfs' para búsqueda en profundidad" << std::endl;
  std::cout << "            Si no se especifica, se ejecutan ambos algoritmos" << std::endl;
  std::cout << std::endl;
  std::cout << "Ejemplos:" << std::endl;
  std::cout << "  " << program_name << " grafo.txt 1 4" << std::endl;
  std::cout << "  " << program_name << " grafo.txt 1 4 bfs" << std::endl;
  std::cout << "  " << program_name << " grafo.txt 1 4 dfs" << std::endl;
}

// Ejecuta un algoritmo de búsqueda y muestra el resultado
void ExecuteSearch(SearchAlgorithm* algorithm, int start, int goal) {
  std::cout << "Ejecutando " << algorithm->GetAlgorithmName() << "..." << std::endl;
  
  SearchResult result = algorithm->Search(start, goal);
  
  if (result.path_found) {
    std::cout << "Camino encontrado: ";
    for (size_t i = 0; i < result.path.size(); ++i) {
      if (i > 0) std::cout << " -> ";
      std::cout << result.path[i];
    }
    std::cout << std::endl;
    std::cout << "Costo total: " << result.total_cost << std::endl;
  } else {
    std::cout << "No se encontró camino entre " << start << " y " << goal << std::endl;
  }
}

int main(int argc, char* argv[]) {
  std::cout << "=== Búsquedas BFS y DFS ===" << std::endl;
  
  // Verificar argumentos
  if (argc < 4 || argc > 5) {
    ShowUsage(argv[0]);
    return 1;
  }
  
  std::string graph_file = argv[1];
  int start_vertex, goal_vertex;
  
  try {
    start_vertex = std::stoi(argv[2]);
    goal_vertex = std::stoi(argv[3]);
  } catch (const std::exception& e) {
    std::cerr << "Error: Los vértices deben ser números enteros válidos" << std::endl;
    return 1;
  }
  
  std::string algorithm_choice = "both";
  if (argc == 5) {
    algorithm_choice = argv[4];
  }
  
  // Cargar grafo
  Graph graph(graph_file);
  
  if (graph.GetNumVertices() == 0) {
    std::cerr << "Error: No se pudo cargar el grafo" << std::endl;
    return 1;
  }
  
  std::cout << "Grafo cargado: " << graph.GetNumVertices() << " vértices, " 
            << graph.GetNumEdges() << " aristas" << std::endl;
  
  // Verificar vértices válidos
  if (!graph.IsValidVertex(start_vertex) || !graph.IsValidVertex(goal_vertex)) {
    std::cerr << "Error: Vértices inválidos. Deben estar entre 1 y " 
              << graph.GetNumVertices() << std::endl;
    return 1;
  }
  
  std::cout << "Buscar camino de " << start_vertex << " a " << goal_vertex << std::endl;
  
  // Ejecutar algoritmos según la opción elegida
  if (algorithm_choice == "bfs" || algorithm_choice == "both") {
    BFS bfs_algorithm(&graph);
    ExecuteSearch(&bfs_algorithm, start_vertex, goal_vertex);
  }
  
  if (algorithm_choice == "dfs" || algorithm_choice == "both") {
    DFS dfs_algorithm(&graph);
    ExecuteSearch(&dfs_algorithm, start_vertex, goal_vertex);
  }
  
  if (algorithm_choice != "bfs" && algorithm_choice != "dfs" && algorithm_choice != "both") {
    std::cerr << "Error: Algoritmo no reconocido. Use 'bfs', 'dfs' o déjelo vacío para ambos" << std::endl;
    return 1;
  }
  
  return 0;
}