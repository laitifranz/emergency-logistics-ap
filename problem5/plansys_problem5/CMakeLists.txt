cmake_minimum_required(VERSION 3.5)
project(plansys2_problem5)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    plansys2_msgs
    plansys2_executor
)

add_executable(move_robot_action_node src/move_robot_action_node.cpp)
ament_target_dependencies(move_robot_action_node ${dependencies})

add_executable(move_robot_with_carrier_action_node src/move_robot_with_carrier_action_node.cpp)
ament_target_dependencies(move_robot_with_carrier_action_node ${dependencies})

add_executable(attach_carrier_to_robot_action_node src/attach_carrier_to_robot_action_node.cpp)
ament_target_dependencies(attach_carrier_to_robot_action_node ${dependencies})

add_executable(detach_carrier_from_robot_action_node src/detach_carrier_from_robot_action_node.cpp)
ament_target_dependencies(detach_carrier_from_robot_action_node ${dependencies})

add_executable(load_box_on_carrier_action_node src/load_box_on_carrier_action_node.cpp)
ament_target_dependencies(load_box_on_carrier_action_node ${dependencies})

add_executable(unload_box_from_carrier_action_node src/unload_box_from_carrier_action_node.cpp)
ament_target_dependencies(unload_box_from_carrier_action_node ${dependencies})

add_executable(fill_box_action_node src/fill_box_action_node.cpp)
ament_target_dependencies(fill_box_action_node ${dependencies})

add_executable(deliver_action_node src/deliver_action_node.cpp)
ament_target_dependencies(deliver_action_node ${dependencies})

add_executable(delivery_or_refactored_possible_action1_action_node src/delivery_or_refactored_possible_action1_action_node.cpp)
ament_target_dependencies(delivery_or_refactored_possible_action1_action_node ${dependencies})

add_executable(delivery_or_refactored_possible_action2_action_node src/delivery_or_refactored_possible_action2_action_node.cpp)
ament_target_dependencies(delivery_or_refactored_possible_action2_action_node ${dependencies})

install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

install(TARGETS
  move_robot_action_node
  move_robot_with_carrier_action_node
  attach_carrier_to_robot_action_node
  detach_carrier_from_robot_action_node
  load_box_on_carrier_action_node
  unload_box_from_carrier_action_node
  fill_box_action_node
  deliver_action_node
  delivery_or_refactored_possible_action1_action_node
  delivery_or_refactored_possible_action2_action_node
  
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()