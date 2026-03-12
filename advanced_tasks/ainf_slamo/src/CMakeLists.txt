find_package(Qt6 COMPONENTS Xml PrintSupport REQUIRED)
find_package(Qt6 COMPONENTS 3DCore 3DRender 3DExtras QUIET)

# Find PyTorch – use the active conda environment's torch installation
list(APPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}/lib/python3.10/site-packages/torch/share/cmake")

# ---- Torch / CUDA handling ----
# Point to conda toolkit precisely:
set(CUDAToolkit_ROOT "$ENV{CONDA_PREFIX}/targets/x86_64-linux" CACHE PATH "CUDA toolkit root" FORCE)
set(CUDA_TOOLKIT_ROOT_DIR "$ENV{CONDA_PREFIX}/targets/x86_64-linux" CACHE PATH "CUDA toolkit root" FORCE)
set(USE_CUDA OFF CACHE BOOL "Disable CUDA for this component" FORCE)
set(CAFFE2_USE_CUDA OFF CACHE BOOL "Disable Caffe2 CUDA" FORCE)
set(CMAKE_DISABLE_FIND_PACKAGE_CUDA OFF CACHE BOOL "Allow CUDA detection for CUDA-enabled libtorch" FORCE)
set(TORCH_CUDA_ARCH_LIST "8.6" CACHE STRING "CUDA arch list for LibTorch" FORCE)


find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")


find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
  ../src/room_concept_ai.cpp
  ../src/room_model.cpp
  ../src/pointcloud_center_estimator.cpp
  ../src/polygon_path_planner.cpp
  ../src/trajectory_controller.cpp
)

# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
  ../src/room_concept_ai.h
  ../src/room_model.h
  ../src/polygon_path_planner.h
  ../src/trajectory_controller.h
)


LIST(APPEND LIBS ${LIBS} "${TORCH_LIBRARIES}" tbb Qt6PrintSupport Qt6Xml
            Eigen3::Eigen Boost::system Boost::filesystem )

if(TARGET Qt6::3DCore AND TARGET Qt6::3DRender AND TARGET Qt6::3DExtras)
  LIST(APPEND LIBS Qt63DCore Qt63DRender Qt63DExtras)
endif()

# no-char8_t to avoid a problem with ICE Connection lib.
add_definitions(-O3 -fmax-errors=1 -fno-char8_t)