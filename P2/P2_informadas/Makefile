# Makefile para el proyecto A* con entornos dinámicos

# Compilador y flags
CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -O2

# Directorios
SRCDIR = .
OBJDIR = obj

# Archivos fuente
SOURCES = main_P2_astar.cc map.cc A_star.cc A_star_node.cc dynamic_environment.cc
OBJECTS = $(SOURCES:%.cc=$(OBJDIR)/%.o)

# Archivo ejecutable
TARGET = astar_laberintos

# Regla principal
all: $(TARGET)

# Crear directorio de objetos
$(OBJDIR):
	mkdir -p $(OBJDIR)

# Compilar el ejecutable
$(TARGET): $(OBJDIR) $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJECTS)
	@echo "Compilación exitosa: $(TARGET)"

# Compilar archivos objeto
$(OBJDIR)/%.o: $(SRCDIR)/%.cc
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Limpiar archivos generados
clean:
	rm -rf $(OBJDIR) $(TARGET)
	rm -f resultado_*.txt
	@echo "Archivos limpiados"

# Limpiar solo objetos
clean-obj:
	rm -rf $(OBJDIR)
	@echo "Archivos objeto limpiados"

# Ejecutar con archivo de ejemplo
test: $(TARGET)
	./$(TARGET) M_1.txt

# Ejecutar modo interactivo
run: $(TARGET)
	./$(TARGET)

# Mostrar ayuda
help:
	@echo "Makefile para A* con entornos dinámicos"
	@echo ""
	@echo "Objetivos disponibles:"
	@echo "  all        - Compilar el proyecto (por defecto)"
	@echo "  clean      - Limpiar todos los archivos generados"
	@echo "  clean-obj  - Limpiar solo archivos objeto"
	@echo "  test       - Compilar y ejecutar con M_1.txt"
	@echo "  run        - Compilar y ejecutar en modo interactivo"
	@echo "  help       - Mostrar esta ayuda"
	@echo ""
	@echo "Uso:"
	@echo "  make                    # Compilar"
	@echo "  make test              # Probar con M_1.txt"
	@echo "  make run               # Ejecutar modo interactivo"
	@echo "  ./$(TARGET)            # Modo interactivo"
	@echo "  ./$(TARGET) archivo.txt # Desde línea de comandos"

# Declarar objetivos que no son archivos
.PHONY: all clean clean-obj test run help

# Dependencias específicas
$(OBJDIR)/main_P2_astar.o: main_P2_astar.cc map.h A_star.h dynamic_environment.h
$(OBJDIR)/map.o: map.cc map.h
$(OBJDIR)/A_star.o: A_star.cc A_star.h map.h A_star_node.h
$(OBJDIR)/A_star_node.o: A_star_node.cc A_star_node.h map.h
$(OBJDIR)/dynamic_environment.o: dynamic_environment.cc dynamic_environment.h map.h A_star.h