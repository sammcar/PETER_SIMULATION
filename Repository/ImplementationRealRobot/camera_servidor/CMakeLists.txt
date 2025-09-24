cmake_minimum_required(VERSION 3.10.0)
project(> VERSION 0.1.0 LANGUAGES C CXX)

# Busca la instalación de OpenCV
find_package(OpenCV REQUIRED)

# Incluye los directorios de encabezados de OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable( udp_test main.cpp)


# Vincula las bibliotecas de OpenCV con el ejecutable
target_link_libraries(udp_test ${OpenCV_LIBS})