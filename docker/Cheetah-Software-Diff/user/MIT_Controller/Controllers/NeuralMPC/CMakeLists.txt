FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_library (NeuralMPC SHARED ${headers} ${sources} )
target_link_libraries (NeuralMPC biomimetics lcm qpOASES)

