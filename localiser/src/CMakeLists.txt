# Purpose: CMake fragment for the source directory. It appends sources/headers and locates
# required dependencies (OpenCV) so the top-level generated CMake can build the `localiser`
# executable.
# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
  # Corner detection and matching implementation files:
  ../src/room_detector.cpp          # Room feature extraction: RANSAC lines → corners
  ../src/hungarian.cpp              # Hungarian algorithm wrapper for corner matching
  ../src/ransac_line_detector.cpp   # RANSAC-based 2D line segment detector
    $ENV{ROBOCOMP}/classes/abstract_graphic_viewer/abstract_graphic_viewer.h

)

# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
  # Corner detection and matching header files:
  ../src/room_detector.h            # Room_Detector class declaration
  ../src/hungarian.h                # Hungarian class declaration
  ../src/ransac_line_detector.h     # RansacLineDetector class declaration
  ../src/common_types.h             # Shared types: LineSegment, Corners, Match, etc.
  ../src/munkres.hpp                # Header-only Munkres algorithm implementation
)


# Find OpenCV (used by room_detector.cpp/h)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
if(OpenCV_FOUND)
  include_directories(${OpenCV_INCLUDE_DIRS})
  list(APPEND LIBS ${OpenCV_LIBS})
endif()


LIST(APPEND LIBS ${LIBS})

# no-char8_t to avoid a problem with ICE Connection lib.
add_definitions(-fmax-errors=1 -fno-char8_t)

