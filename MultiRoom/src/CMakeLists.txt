# Purpose: CMake fragment for the source directory.
# It appends sources/headers and locates required dependencies (OpenCV).

# --- Sources set ---
LIST(APPEND SOURCES
  ../src/specificworker.cpp
  ../src/room_detector.cpp          # Room feature extraction
  ../src/hungarian.cpp              # Hungarian algorithm wrapper
  ../src/ransac_line_detector.cpp   # RANSAC-based 2D line segment detector
)

# Dependiendo de cómo use RoboComp este archivo, a veces es mejor
# poner las cabeceras externas en HEADERS o simplemente incluirlas,
# pero si necesitas que aparezca en el IDE, déjalo aquí.
# Asegúrate de que la variable de entorno ROBOCOMP esté definida.
if(DEFINED ENV{ROBOCOMP})
  LIST(APPEND HEADERS # Generalmente mejor en HEADERS que en SOURCES si es .h
    $ENV{ROBOCOMP}/classes/abstract_graphic_viewer/abstract_graphic_viewer.h
  )
  include_directories($ENV{ROBOCOMP}/classes/abstract_graphic_viewer)
else()
  message(WARNING "La variable de entorno ROBOCOMP no está definida. Puede faltar abstract_graphic_viewer.h")
endif()

# --- Headers set ---
LIST(APPEND HEADERS
  ../src/specificworker.h
  ../src/room_detector.h
  ../src/hungarian.h
  ../src/ransac_line_detector.h
  ../src/common_types.h
  ../src/munkres.hpp
)

# --- Find Dependencies ---
# Find OpenCV (REQUIRED lanzará error si no lo encuentra, no hace falta if(OpenCV_FOUND) redundante para el error)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

INCLUDE($ENV{ROBOCOMP}/cmake/modules/opencv4.cmake)

# Si llega aquí, OpenCV fue encontrado.
include_directories(${OpenCV_INCLUDE_DIRS})
list(APPEND LIBS ${OpenCV_LIBS})

# --- Compiler Definitions ---
# no-char8_t to avoid a problem with ICE Connection lib.
add_definitions(-fmax-errors=1 -fno-char8_t)

SET(LIBS ${LIBS} tbb Qt6PrintSupport)
