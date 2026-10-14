cmake_minimum_required(VERSION 3.10)
project(erl_localization LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin QUIET)

if(catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES erl_localization_core
  )
endif()

add_library(erl_localization_core
  src/reflector_detector.cpp
  src/cylinder_fitter.cpp
  src/pose_calculator.cpp
)

target_include_directories(erl_localization_core
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(erl_node
  src/erl_node.cpp
)

target_link_libraries(erl_node PRIVATE erl_localization_core)

if(catkin_FOUND)
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
else()
  install(TARGETS erl_localization_core erl_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})
  install(FILES config/erl.yaml DESTINATION share/${PROJECT_NAME}/config)
endif()

enable_testing()
add_test(
  NAME erl_node_skeleton_smoke
  COMMAND erl_node --config ${CMAKE_CURRENT_SOURCE_DIR}/config/erl.yaml
)
