cmake_minimum_required(VERSION 3.0.2)
project(erl_localization)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES erl_localization_core
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(erl_localization_core
  src/reflector_detector.cpp
  src/cylinder_fitter.cpp
  src/pose_calculator.cpp
)

target_link_libraries(erl_localization_core
  ${catkin_LIBRARIES}
)

add_executable(erl_node
  src/erl_node.cpp
)

target_link_libraries(erl_node
  erl_localization_core
  ${catkin_LIBRARIES}
)

install(TARGETS erl_localization_core erl_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES config/erl.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
