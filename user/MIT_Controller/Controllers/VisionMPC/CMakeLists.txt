FILE(GLOB_RECURSE headers ${GLOB_DEPENDS} *.hpp)
FILE(GLOB_RECURSE sources ${GLOB_DEPENDS} *.cpp)

add_library (VisionMPC SHARED ${headers} ${sources} )
target_link_libraries (VisionMPC biomimetics lcm qpOASES)

