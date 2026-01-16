# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
  ../src/qcustomplot.cpp
  ../src/door_detector.cpp
  ../src/graph.cpp
  ../src/hungarian.cpp
  ../src/ransac_line_detector.cpp
  ../src/room_detector.cpp
  ../src/time_series_plotter.cpp
  ../src/localiser/localiser.cpp
  ../src/navigation_state_machine.cpp
)

# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
  ../src/qcustomplot.h
  ../src/door_detector.h
  ../src/graph.h
  ../src/hungarian.h
  ../src/image_processor.h
  ../src/ransac_line_detector.h
  ../src/room_detector.h
  ../src/time_series_plotter.h
  ../src/localiser/localiser.h
  ../src/common_types.h
  ../src/nominal_room.h
  ../src/navigation_state_machine.h
)


find_package(OpenCV REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS PrintSupport)
find_package(TBB REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_LIST_DIR})

LIST(APPEND LIBS ${LIBS} ${OpenCV_LIBS} Qt6::PrintSupport TBB::tbb)

# no-char8_t to avoid a problem with ICE Connection lib.
add_definitions(-fmax-errors=1 -fno-char8_t)

