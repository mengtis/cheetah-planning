include_directories("./")
include_directories("./Controllers")
include_directories("../../robot/include")
include_directories("../../common/include/")
include_directories("../../common/FootstepPlanner")
include_directories("../../third-party/")
include_directories(SYSTEM "../../third-party/qpOASES/include")
include_directories("../../third-party/ParamHandler")

include_directories("../../lcm-types/cpp")
include_directories("/usr/local/include/lcm/")   # lcm includes
include_directories("/usr/local/include/eigen3")   # lcm includes
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

find_package(jsoncpp CONFIG REQUIRED)


# Scope this deliberately: WBC, WBC_Ctrl, VisionMPC and Locomotion are built
# as separate libraries below and linked into mit_ctrl. A recursive glob over
# "./*.cpp" also pulls their sources in here, compiling ~22 translation units
# a second time and duplicating their symbols in the executable.
file(GLOB sources ${GLOB_DEPENDS} "*.cpp")
file(GLOB_RECURSE controller_sources ${GLOB_DEPENDS}
"FSM_States/*.cpp"
"Controllers/BackFlip/*.cpp"
"Controllers/BalanceController/*.cpp"
"Controllers/convexMPC/*.cpp")
list(APPEND sources ${controller_sources})

add_subdirectory(Controllers/WBC)
add_subdirectory(Controllers/WBC_Ctrl)
add_subdirectory(Controllers/VisionMPC)
add_subdirectory(Controllers/Locomotion)

add_executable(mit_ctrl ${sources})
target_link_libraries(mit_ctrl robot biomimetics)
target_link_libraries(mit_ctrl dynacore_param_handler qpOASES)
target_link_libraries(mit_ctrl Goldfarb_Optimizer osqp)
target_link_libraries(mit_ctrl WBC_Ctrl)
target_link_libraries(mit_ctrl VisionMPC)
target_link_libraries(mit_ctrl Locomotion)


target_link_libraries(mit_ctrl jsoncpp_lib)
