#ifndef NODE_H
#define NODE_H

#include <string>
#include <memory>
#include <vector>

/**
 * @brief Clase que representa un nodo en el espacio de búsqueda
 * 
 * Esta clase encapsula la información de un estado en el problema de búsqueda,
 * incluyendo el estado actual, el nodo padre y el costo acumulado para llegar
 * hasta este nodo.
 */
class Node {
private:
  std::string state_;         // Estado que representa este nodo
  std::shared_ptr<Node> parent_;  // Puntero al nodo padre
  double path_cost_;          // Costo acumulado desde el nodo inicial
  int depth_;                 // Profundidad del nodo en el árbol de búsqueda

public:
  /**
   * @brief Constructor por defecto
   */
  Node();

  /**
   * @brief Constructor con parámetros
   * @param state Estado que representa el nodo
   * @param parent Puntero al nodo padre (nullptr si es raíz)
   * @param path_cost Costo acumulado para llegar a este nodo
   * @param depth Profundidad del nodo en el árbol
   */
  Node(const std::string& state, std::shared_ptr<Node> parent = nullptr, 
       double path_cost = 0.0, int depth = 0);

  /**
   * @brief Constructor de copia
   */
  Node(const Node& other);

  /**
   * @brief Operador de asignación
   */
  Node& operator=(const Node& other);

  /**
   * @brief Destructor
   */
  ~Node();

  // Getters
  const std::string& GetState() const;
  std::shared_ptr<Node> GetParent() const;
  double GetPathCost() const;
  int GetDepth() const;

  // Setters
  void SetState(const std::string& state);
  void SetParent(std::shared_ptr<Node> parent);
  void SetPathCost(double path_cost);
  void SetDepth(int depth);

  /**
   * @brief Operador de comparación para igualdad
   * Dos nodos son iguales si representan el mismo estado
   */
  bool operator==(const Node& other) const;

  /**
   * @brief Operador de comparación para ordenación
   * Compara por costo del camino (útil para búsqueda de costo uniforme)
   */
  bool operator<(const Node& other) const;

  /**
   * @brief Convierte el nodo a string para depuración
   */
  std::string ToString() const;

  /**
   * @brief Obtiene la secuencia de estados desde la raíz hasta este nodo
   * @return Vector con la secuencia de estados del camino
   */
  std::vector<std::string> GetPath() const;
};

#endif // NODE_H