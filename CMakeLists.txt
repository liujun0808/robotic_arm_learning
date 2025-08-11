cmake_minimum_required(VERSION 3.10)
project(force_control)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release) 

list(APPEND CMAKE_PREFIX_PATH "/opt/pinocchio;/opt/eigenpy;/opt/qpOASES")  # 帮助CMake找到Pinocchio，为了保证不使用ros中的eigenpy
set(Python_ADDITIONAL_VERSIONS 3.8 3.9 3.10 3.11)  # 强制Python3 版本 因为安装pinocchio eigenpy时没有指定python3去安装 使用的py2但是又没有安装py2版本的numpy
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(pinocchio REQUIRED)
find_package(eigenpy REQUIRED) 
find_package(jsoncpp REQUIRED)
find_package(glfw3 REQUIRED)
find_package(qpOASES REQUIRED)
find_package(ompl REQUIRED)
set(QPOASES_INCLUDE_DIR "/opt/qpOASES/include")
set(QPOASES_LIBRARY "/opt/qpOASES/lib/libqpOASES.a")  # 或者 .a

# 添加的头文件路径
include_directories(
    /opt/mujoco/include
    ${Python3_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIRS}
    ${QPOASES_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/sim_interface/include
    ${CMAKE_SOURCE_DIR}/control_algorithm/include
    ${OMPL_INCLUDE_DIRS}
)


# 指定链接库的搜索路径
link_directories(
  /opt/mujoco/lib
  # /opt/pinocchio/lib
)

# 构建为静态库（libsim_interface.a） 其中 PUBLIC 控制库的依赖传播行为
# sim_interface
file(GLOB SIM_SRC ${CMAKE_SOURCE_DIR}/sim_interface/src/*.cpp)
add_library(sim_interface STATIC ${SIM_SRC})
target_link_libraries(sim_interface PUBLIC mujoco glfw)

# control_algorithm
file(GLOB CON_SRC ${CMAKE_SOURCE_DIR}/control_algorithm/src/*.cpp)
add_library(control_algorithm STATIC ${CON_SRC})
target_link_libraries(control_algorithm PUBLIC sim_interface pinocchio::pinocchio jsoncpp_lib ${QPOASES_LIBRARY} ${OMPL_LIBRARIES})

# 生成可执行文件 
add_executable(demo_motor_param ${CMAKE_SOURCE_DIR}/demo/demo_motor_param.cpp)
target_link_libraries(demo_motor_param sim_interface control_algorithm)

add_executable(demo_ik ${CMAKE_SOURCE_DIR}/demo/demo_ik.cpp)
target_link_libraries(demo_ik sim_interface control_algorithm)

add_executable(demo_dyn ${CMAKE_SOURCE_DIR}/demo/demo_dyn.cpp)
target_link_libraries(demo_dyn sim_interface control_algorithm)

add_executable(demo_traj_plan ${CMAKE_SOURCE_DIR}/demo/demo_traj_plan.cpp)
target_link_libraries(demo_traj_plan sim_interface control_algorithm)

