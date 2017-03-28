cmake_minimum_required(VERSION 3.6)
project(localSearch)

set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -ggdb -DNDEBUG")
set(CMAKE_BUILD_TYPE "Debug")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE})


##############################################################################
# Gurobi
##############################################################################
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)
find_package(GUROBI)
if (NOT GUROBI_FOUND)
    message("Gurobi not found. If you want to use Gurobi, set the GUROBI_HOME variable")
else()
    message("Gurobi found with include: ${GUROBI_INCLUDE_DIRS} and library: ${GUROBI_LIBRARIES}")
    include_directories(${GUROBI_INCLUDE_DIRS})
endif()
##############################################################################


## REAL LINES
include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory (src)
## DEBUG/RELEASE LINES
add_executable(main ${PROJECT_SOURCE_DIR}/src/main.cpp)
target_link_libraries (main SOURCE_FILES)



## TESTING LINES
#add_subdirectory (tests)
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/tests/)
#include_directories(${PROJECT_SOURCE_DIR}/tests/include)
#add_executable(runTest ${PROJECT_SOURCE_DIR}/tests/testMain.cpp)
#target_link_libraries (runTest SOURCE_FILES TEST_SOURCE_FILES)


## GUROBI LINK LIBRARY
if(GUROBI_FOUND)
    target_link_libraries(main ${GUROBI_CPP_LIBRARY} ${GUROBI_LIBRARY})
endif()
