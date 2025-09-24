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
  std::cout << "  vertice_origen: Vértice de inicio (1-indexado)" << std::endl;
  std::cout << "  vertice_destino: Vértice objetivo (1-indexado)" << std::endl;
  std::cout << "  algoritmo: 'bfs' para búsqueda en amplitud, 'dfs' para búsqueda en profundidad" << std::endl;
  std::cout << "            Si no se especifica, se ejecutan ambos algoritmos" << std::endl;
  std::cout << std::endl;
  std::cout << "Ejemplos:" << std::endl;
  std::cout << "  " << program_name << " grafo.txt 1 4" << std::endl;
  std::cout << "  " << program_name << " grafo.txt 1 4 bfs" << std::endl;
  std::cout << "  " << program_name << " grafo.txt 1 4 dfs" << std::endl;
}

/**
 * @brief Ejecuta un algoritmo de búsqueda y guarda el resultado
 */
void ExecuteSearch(SearchAlgorithm* algorithm, const Graph& /* graph */, 
                   int start, int goal, const std::string& output_suffix) {
  std::cout << "Ejecutando " << algorithm->GetAlgorithmName() << "..." << std::endl;
  
  SearchResult result = algorithm->Search(start, goal);
  
  // Mostrar resultado en consola
  std::cout << algorithm->GenerateReport(result, start, goal) << std::endl;
  
  // Generar nombre de archivo de salida
  std::string algorithm_name = algorithm->GetAlgorithmName();
  std::string filename = "resultado_" + output_suffix + "_" + 
                        std::to_string(start) + "_to_" + std::to_string(goal) + ".txt";
  
  // Reemplazar espacios y caracteres especiales en el nombre del archivo
  for (char& c : filename) {
    if (c == ' ' || c == '(' || c == ')') {
      c = '_';
    }
  }
  
  // Guardar resultado en archivo
  if (algorithm->SaveResultToFile(result, start, goal, filename)) {
    std::cout << "Resultado guardado en: " << filename << std::endl;
  } else {
    std::cerr << "Error al guardar el resultado en archivo" << std::endl;
  }
  
  std::cout << std::string(60, '-') << std::endl;
}

/**
 * @brief Función principal
 */
int main(int argc, char* argv[]) {
  std::cout << "=== Programa de Búsquedas No Informadas ===" << std::endl;
  std::cout << "Implementación de BFS y DFS para encontrar caminos en grafos" << std::endl;
  std::cout << std::string(60, '=') << std::endl;
  
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
  std::cout << "Cargando grafo desde: " << graph_file << std::endl;
  Graph graph(graph_file);
  
  if (graph.GetNumVertices() == 0) {
    std::cerr << "Error: No se pudo cargar el grafo o el grafo está vacío" << std::endl;
    return 1;
  }
  
  // Mostrar información del grafo
  std::cout << "Grafo cargado exitosamente:" << std::endl;
  std::cout << "  Número de vértices: " << graph.GetNumVertices() << std::endl;
  std::cout << "  Número de aristas: " << graph.GetNumEdges() << std::endl;
  std::cout << std::endl;
  
  // Verificar vértices válidos
  if (!graph.IsValidVertex(start_vertex) || !graph.IsValidVertex(goal_vertex)) {
    std::cerr << "Error: Vértices inválidos. Deben estar entre 1 y " 
              << graph.GetNumVertices() << std::endl;
    return 1;
  }
  
  // Mostrar matriz del grafo (opcional, solo para grafos pequeños)
  if (graph.GetNumVertices() <= 10) {
    std::cout << "Matriz de adyacencia del grafo:" << std::endl;
    graph.PrintMatrix();
    std::cout << std::endl;
  }
  
  std::cout << "Búsqueda de camino desde vértice " << start_vertex 
            << " hasta vértice " << goal_vertex << std::endl;
  std::cout << std::string(60, '=') << std::endl;
  
  // Ejecutar algoritmos según la opción elegida
  if (algorithm_choice == "bfs" || algorithm_choice == "both") {
    BFS bfs_algorithm(&graph);
    ExecuteSearch(&bfs_algorithm, graph, start_vertex, goal_vertex, "bfs");
  }
  
  if (algorithm_choice == "dfs" || algorithm_choice == "both") {
    DFS dfs_algorithm(&graph);
    ExecuteSearch(&dfs_algorithm, graph, start_vertex, goal_vertex, "dfs");
  }
  
  if (algorithm_choice != "bfs" && algorithm_choice != "dfs" && algorithm_choice != "both") {
    std::cerr << "Error: Algoritmo no reconocido. Use 'bfs', 'dfs' o déjelo vacío para ambos" << std::endl;
    return 1;
  }
  
  std::cout << "Programa ejecutado exitosamente." << std::endl;
  return 0;
}