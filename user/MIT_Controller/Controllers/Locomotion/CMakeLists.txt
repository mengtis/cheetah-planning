FILE(GLOB_RECURSE headers ${GLOB_DEPENDS} *.hpp)
FILE(GLOB_RECURSE sources ${GLOB_DEPENDS} *.cpp)

find_package(jsoncpp CONFIG REQUIRED)

add_library(Locomotion SHARED ${headers} ${sources})
target_link_libraries(Locomotion biomimetics lcm qpOASES)
target_link_libraries(Locomotion jsoncpp_lib)
