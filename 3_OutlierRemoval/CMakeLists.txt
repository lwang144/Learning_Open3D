# Requirement of cmake version
cmake_minimum_required(VERSION 3.2)
project(OutlierRemoval) 
set (CMAKE_CXX_STANDARD 11)

# Set directory of executable files
set(CMAKE_RUNTIME)

# set the directory of executable files
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

# Find and include Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Find and include Open3D
find_package(Open3D REQUIRED)
# link_directories must be before add_executable
link_directories(${Open3D_LIBRARY_DIRS})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${Open3D_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${Open3D_EXE_LINKER_FLAGS}")

# Name of executable file and path of source file
add_executable(${PROJECT_NAME}
	${CMAKE_CURRENT_SOURCE_DIR} src/${PROJECT_NAME}.cpp)
# Open3D libraries
target_link_libraries(${PROJECT_NAME} ${LIBRARIES} ${Open3D_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${Open3D_INCLUDE_DIRS})
