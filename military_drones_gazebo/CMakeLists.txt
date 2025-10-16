cmake_minimum_required(VERSION 3.8)
project(military_drones_gazebo)

find_package(ament_cmake REQUIRED)
find_package(military_drones_description REQUIRED)

find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-common5 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
message(STATUS "Compiling against Gazebo Harmonic (gz-sim${GZ_SIM_VER})")

include_directories(include)

add_library(BasicSystem SHARED src/BasicSystem.cc)
target_include_directories(BasicSystem PRIVATE include)
target_link_libraries(BasicSystem PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

add_library(FullSystem SHARED src/FullSystem.cc)
target_include_directories(FullSystem PRIVATE include)
target_link_libraries(FullSystem PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

install(
  TARGETS BasicSystem FullSystem
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
