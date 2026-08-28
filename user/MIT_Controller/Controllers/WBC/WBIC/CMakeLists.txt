FILE(GLOB_RECURSE headers ${GLOB_DEPENDS} *.hpp)
FILE(GLOB_RECURSE sources ${GLOB_DEPENDS} *.cpp)


add_library(WBIC SHARED ${sources} ${headers})
target_link_libraries(WBIC Goldfarb_Optimizer)
#target_link_libraries(WBIC JCQP)
