cmake_minimum_required(VERSION 3.5)
project(plansys2_house_problem)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_bt_actions REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(dependencies
    rclcpp
    rclcpp_action
    geometry_msgs
    tf2_geometry_msgs
    nav2_msgs
    plansys2_msgs
    plansys2_domain_expert
    plansys2_executor
    plansys2_planner
    plansys2_problem_expert
    plansys2_pddl_parser
    ament_index_cpp
    plansys2_bt_actions
)

include_directories(include)

add_library(close_tap_node SHARED src/behavior_tree_nodes/CloseTap.cpp)
list(APPEND plugin_libs close_tap_node)

add_library(drop_sponge_node SHARED src/behavior_tree_nodes/DropSponge.cpp)
list(APPEND plugin_libs drop_sponge_node)

add_library(grab_sponge_node SHARED src/behavior_tree_nodes/GrabSponge.cpp)
list(APPEND plugin_libs grab_sponge_node)

add_library(move_node SHARED src/behavior_tree_nodes/Move.cpp)
list(APPEND plugin_libs move_node)

add_library(open_tap_node SHARED src/behavior_tree_nodes/OpenTap.cpp)
list(APPEND plugin_libs open_tap_node)

add_library(rinse_node SHARED src/behavior_tree_nodes/Rinse.cpp)
list(APPEND plugin_libs rinse_node)

add_library(ask_destination_node SHARED src/behavior_tree_nodes/AskDestination.cpp)
list(APPEND plugin_libs ask_destination_node)

add_library(ask_open_door_node SHARED src/behavior_tree_nodes/AskOpenDoor.cpp)
list(APPEND plugin_libs ask_open_door_node)

add_library(drop_trash_node SHARED src/behavior_tree_nodes/DropTrash.cpp)
list(APPEND plugin_libs drop_trash_node)

add_library(open_door_node SHARED src/behavior_tree_nodes/OpenDoor.cpp)
list(APPEND plugin_libs open_door_node)

add_library(pick_trash_node SHARED src/behavior_tree_nodes/PickTrash.cpp)
list(APPEND plugin_libs pick_trash_node)

add_library(pass_door_node SHARED src/behavior_tree_nodes/PassDoor.cpp)
list(APPEND plugin_libs pass_door_node)

add_library(pillow_placement_node SHARED src/behavior_tree_nodes/PillowPlacement.cpp)
list(APPEND plugin_libs pillow_placement_node)

add_library(place_cutlery_node SHARED src/behavior_tree_nodes/PlaceCutlery.cpp)
list(APPEND plugin_libs place_cutlery_node)

add_library(place_dishes_node SHARED src/behavior_tree_nodes/PlaceDishes.cpp)
list(APPEND plugin_libs place_dishes_node)

add_library(place_blanket_node SHARED src/behavior_tree_nodes/PlaceBlanket.cpp)
list(APPEND plugin_libs place_blanket_node)

add_library(place_glasses_node SHARED src/behavior_tree_nodes/PlaceGlasses.cpp)
list(APPEND plugin_libs place_glasses_node)

add_library(present_person_node SHARED src/behavior_tree_nodes/PresentPerson.cpp)
list(APPEND plugin_libs present_person_node)

add_library(scrub_node SHARED src/behavior_tree_nodes/Scrub.cpp)
list(APPEND plugin_libs scrub_node)

add_library(stretch_sheet_node SHARED src/behavior_tree_nodes/StretchSheet.cpp)
list(APPEND plugin_libs stretch_sheet_node)

add_library(recharge_node SHARED src/behavior_tree_nodes/Recharge.cpp)
list(APPEND plugin_libs recharge_node)



foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(house_controller_node src/house_controller_node.cpp)
ament_target_dependencies(house_controller_node ${dependencies})

add_executable(nav2_sim_node src/nav2_sim_node.cpp)
ament_target_dependencies(nav2_sim_node ${dependencies})

install(DIRECTORY launch pddl behavior_trees_xml config DESTINATION share/${PROJECT_NAME})

install(TARGETS
  
  house_controller_node
  nav2_sim_node
  ${plugin_libs}
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
