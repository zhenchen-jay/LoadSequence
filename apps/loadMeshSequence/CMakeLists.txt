################################################################################
# General Information
################################################################################

cmake_minimum_required(VERSION 3.1)
project(loadMeshSequence)

# Add your project files
file(GLOB SRCFILES 
        *.cpp)
	  

add_executable(${PROJECT_NAME}_bin ${SRCFILES})
target_link_libraries( ${PROJECT_NAME}_bin 
        polyscope 
        TBB::tbb 
        tbb_static 
        tbbmalloc_static
        igl::core 
        igl_restricted::triangle 
        igl_copyleft::comiso 
        CLI11::CLI11)

# Use C++14
set_target_properties(${PROJECT_NAME}_bin PROPERTIES CXX_STANDARD 17)
set_target_properties(${PROJECT_NAME}_bin PROPERTIES CXX_STANDARD_REQUIRED ON)

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME}_bin OpenMP::OpenMP_CXX)
endif()