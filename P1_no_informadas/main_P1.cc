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

void ExecuteSearch(SearchAlgorithm* algorithm, const Graph& graph, int start, int goal) {
  std::cout << "Ejecutando " << algorithm->GetAlgorithmName() << "..." << std::endl;
  
  SearchResult result = algorithm->Search(start, goal);
  
  std::string report = algorithm->GenerateDetailedReport(result, start, goal);
  std::cout << report << std::endl;
  
  std::string algorithm_name = algorithm->GetAlgorithmName();
  std::string filename = "resultado_" + algorithm_name + "_" + std::to_string(start) + "_to_" + std::to_string(goal) + ".txt";
  
  if (algorithm->SaveResultToFile(result, start, goal, filename)) {
    std::cout << "Resultado guardado en: " << filename << std::endl;
    std::cout << std::endl;
  } else {
    std::cerr << "Error al guardar el resultado en archivo" << std::endl;
  }
  
  //std::cout << std::string(60, '-') << std::endl;
}

int main(int argc, char* argv[]) {
  std::cout << "\n=== BÚSQUEDAS NO INFORMADAS ===" << std::endl;
  
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
  
  std::cout << "\nGrafo cargado: " << graph.GetNumVertices() << " vértices, " << graph.GetNumEdges() << " aristas" << std::endl;
  
  // Verificar vértices válidos
  if (!graph.IsValidVertex(start_vertex) || !graph.IsValidVertex(goal_vertex)) {
    std::cerr << "Error: Vértices inválidos. Deben estar entre 1 y " << graph.GetNumVertices() << std::endl;
    return 1;
  }
  
  std::cout << "Buscar camino de " << start_vertex << " a " << goal_vertex << std::endl;
  
  // Ejecutar algoritmos según la opción elegida
  if (algorithm_choice == "bfs" || algorithm_choice == "both") {
    BFS bfs_algorithm(&graph);
    ExecuteSearch(&bfs_algorithm, graph, start_vertex, goal_vertex);
  }
  
  if (algorithm_choice == "dfs" || algorithm_choice == "both") {
    DFS dfs_algorithm(&graph);
    ExecuteSearch(&dfs_algorithm, graph, start_vertex, goal_vertex);
  }
  
  if (algorithm_choice != "bfs" && algorithm_choice != "dfs" && algorithm_choice != "both") {
    std::cerr << "Error: Algoritmo no reconocido. Use 'bfs', 'dfs' o déjelo vacío para ambos" << std::endl;
    return 1;
  }
  
  return 0;
}