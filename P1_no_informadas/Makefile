# Makefile para el proyecto de Búsquedas No Informadas
# Compilador y flags
CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -O2

# Directorio de archivos objeto
OBJDIR = obj

# Archivos fuente
SOURCES = main_P1.cc node.cc graph.cc search_algorithm.cc bfs.cc dfs.cc
HEADERS = node.h graph.h search_algorithm.h bfs.h dfs.h

# Archivos objeto
OBJECTS = $(SOURCES:%.cc=$(OBJDIR)/%.o)

# Nombre del ejecutable
TARGET = busquedas_no_informadas

# Regla por defecto
.PHONY: all
all: $(TARGET)

# Crear directorio de objetos si no existe
$(OBJDIR):
	mkdir -p $(OBJDIR)

# Enlazar el ejecutable
$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^
	@echo "Compilación exitosa: $(TARGET)"

# Compilar archivos objeto
$(OBJDIR)/%.o: %.cc $(HEADERS) | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Limpiar archivos generados
.PHONY: clean
clean:
	rm -rf $(OBJDIR) $(TARGET)
	rm -f resultado_*.txt
	@echo "Archivos limpiados"

# Limpiar solo archivos objeto
.PHONY: clean-obj
clean-obj:
	rm -rf $(OBJDIR)
	@echo "Archivos objeto limpiados"

# Mostrar ayuda
.PHONY: help
help:
	@echo "Makefile para Búsquedas No Informadas"
	@echo ""
	@echo "Objetivos disponibles:"
	@echo "  all        - Compila el programa (por defecto)"
	@echo "  clean      - Elimina todos los archivos generados"
	@echo "  clean-obj  - Elimina solo los archivos objeto"
	@echo "  help       - Muestra esta ayuda"
	@echo "  test       - Ejecuta pruebas básicas"
	@echo ""
	@echo "Uso del programa:"
	@echo "  ./$(TARGET) <archivo_grafo> <origen> <destino> [algoritmo]"
	@echo "  Ejemplo: ./$(TARGET) ejemplo.txt 1 4 bfs"

# Crear archivo de ejemplo para pruebas
.PHONY: ejemplo
ejemplo:
	@echo "Creando archivo de ejemplo: ejemplo_grafo.txt"
	@echo "5" > ejemplo_grafo.txt
	@echo "1.225" >> ejemplo_grafo.txt
	@echo "1.000" >> ejemplo_grafo.txt
	@echo "-1" >> ejemplo_grafo.txt
	@echo "-1" >> ejemplo_grafo.txt
	@echo "-1" >> ejemplo_grafo.txt
	@echo "-1" >> ejemplo_grafo.txt
	@echo "2.236" >> ejemplo_grafo.txt
	@echo "1.000" >> ejemplo_grafo.txt
	@echo "-1" >> ejemplo_grafo.txt
	@echo "-1" >> ejemplo_grafo.txt
	@echo "Archivo ejemplo_grafo.txt creado"

# Ejecutar pruebas básicas
.PHONY: test
test: $(TARGET) ejemplo
	@echo "Ejecutando pruebas básicas..."
	./$(TARGET) ejemplo_grafo.txt 1 4
	@echo "Pruebas completadas"

# Información de dependencias
$(OBJDIR)/main.o: main.cc graph.h bfs.h dfs.h
$(OBJDIR)/node.o: node.cc node.h
$(OBJDIR)/graph.o: graph.cc graph.h
$(OBJDIR)/search_algorithm.o: search_algorithm.cc search_algorithm.h graph.h node.h
$(OBJDIR)/bfs.o: bfs.cc bfs.h search_algorithm.h
$(OBJDIR)/dfs.o: dfs.cc dfs.h search_algorithm.h