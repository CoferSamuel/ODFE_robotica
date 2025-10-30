# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
    $ENV{ROBOCOMP}/classes/abstract_graphic_viewer/abstract_graphic_viewer.h

)

# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
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

