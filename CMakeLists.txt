cmake_minimum_required(VERSION 3.0.2)
project(navigation_position_refinement)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        actionlib
        actionlib_msgs
        roscpp
        tf2
        tf2_geometry_msgs
        tf2_ros
)

add_action_files(
        DIRECTORY action
        FILES
        PositionRefinement.action
        BlindMovement.action
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
        actionlib_msgs
        std_msgs
)

catkin_package(

        # INCLUDE_DIRS include
        # LIBRARIES navigation_position_refinement
        CATKIN_DEPENDS actionlib actionlib_msgs roscpp tf2 tf2_geometry_msgs tf2_ros

        # DEPENDS system_lib
)

# ##########
# # Build ##
# ##########
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(position_refinement src/position_refinement_node.cpp)
add_dependencies(position_refinement ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(position_refinement
        ${catkin_LIBRARIES}
)

add_executable(blind_movement src/blind_movement.cpp)
add_dependencies(blind_movement ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(blind_movement
        ${catkin_LIBRARIES}
)

add_executable(test_tf src/test_node.cpp)
add_dependencies(test_tf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_tf
        ${catkin_LIBRARIES}
)
