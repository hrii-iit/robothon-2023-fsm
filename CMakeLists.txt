cmake_minimum_required(VERSION 3.0.0)
project(hrii_task_board_fsm)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  eigen_conversions
  hrii_gri_interface
  hrii_robothon_msgs
  hrii_trajectory_planner
  message_generation
)

find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS
   hrii_trajectory_planner
   hrii_gri_interface
   hrii_robothon_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/fake_perception_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


add_executable(${PROJECT_NAME}_main_fsm src/main_fsm.cpp)
set_target_properties(${PROJECT_NAME}_main_fsm PROPERTIES OUTPUT_NAME "main_fsm")
add_dependencies(${PROJECT_NAME}_main_fsm ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_main_fsm ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_homing_fsm src/tasks/homing_fsm.cpp)
set_target_properties(${PROJECT_NAME}_homing_fsm PROPERTIES OUTPUT_NAME "homing_fsm")
add_dependencies(${PROJECT_NAME}_homing_fsm ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_homing_fsm ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_board_detection_fsm src/tasks/board_detection_fsm.cpp)
set_target_properties(${PROJECT_NAME}_board_detection_fsm PROPERTIES OUTPUT_NAME "board_detection_fsm")
add_dependencies(${PROJECT_NAME}_board_detection_fsm ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_board_detection_fsm ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_press_button_fsm src/tasks/press_button_fsm.cpp)
set_target_properties(${PROJECT_NAME}_press_button_fsm PROPERTIES OUTPUT_NAME "press_button_fsm")
add_dependencies(${PROJECT_NAME}_press_button_fsm ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_press_button_fsm ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_move_slider_fsm src/tasks/move_slider_fsm.cpp)
set_target_properties(${PROJECT_NAME}_move_slider_fsm PROPERTIES OUTPUT_NAME "move_slider_fsm")
add_dependencies(${PROJECT_NAME}_move_slider_fsm ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_move_slider_fsm ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_move_plug_fsm src/tasks/move_plug_fsm.cpp)
set_target_properties(${PROJECT_NAME}_move_plug_fsm PROPERTIES OUTPUT_NAME "move_plug_fsm")
add_dependencies(${PROJECT_NAME}_move_plug_fsm ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_move_plug_fsm ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_open_door_fsm src/tasks/open_door_fsm.cpp)
set_target_properties(${PROJECT_NAME}_open_door_fsm PROPERTIES OUTPUT_NAME "open_door_fsm")
add_dependencies(${PROJECT_NAME}_open_door_fsm ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_open_door_fsm ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_probe_circuit_fsm src/tasks/probe_circuit_fsm.cpp)
set_target_properties(${PROJECT_NAME}_probe_circuit_fsm PROPERTIES OUTPUT_NAME "probe_circuit_fsm")
add_dependencies(${PROJECT_NAME}_probe_circuit_fsm ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_probe_circuit_fsm ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_stow_probe_cable_fsm src/tasks/stow_probe_cable_fsm.cpp)
set_target_properties(${PROJECT_NAME}_stow_probe_cable_fsm PROPERTIES OUTPUT_NAME "stow_probe_cable_fsm")
add_dependencies(${PROJECT_NAME}_stow_probe_cable_fsm ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_stow_probe_cable_fsm ${catkin_LIBRARIES})
