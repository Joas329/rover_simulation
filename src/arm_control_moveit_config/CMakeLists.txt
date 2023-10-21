cmake_minimum_required(VERSION 3.22)
project(arm_control_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})



#####################
## Component Nodes ##
#####################

# # Add and export library to run as a ROS node component, and receive commands via topics
# add_library(${SERVO_COMPONENT_NODE} SHARED src/servo_node.cpp)
# ament_target_dependencies(${SERVO_COMPONENT_NODE} ${THIS_PACKAGE_INCLUDE_DEPENDS})
# target_link_libraries(${SERVO_COMPONENT_NODE} ${SERVO_LIB_NAME})
# rclcpp_components_register_nodes(${SERVO_COMPONENT_NODE} "moveit_servo::ServoNode")

# # Add executable for using a controller
# add_library(${SERVO_CONTROLLER_INPUT} SHARED src/teleop_demo/joystick_servo_example.cpp)
# ament_target_dependencies(${SERVO_CONTROLLER_INPUT} ${THIS_PACKAGE_INCLUDE_DEPENDS})
# rclcpp_components_register_nodes(${SERVO_CONTROLLER_INPUT} "moveit_servo::JoyToServoPub")

# ######################
# ## Executable Nodes ##
# ######################

# # An executable node for the servo server
# add_executable(${SERVO_NODE_MAIN_NAME} src/servo_node_main.cpp)
# target_link_libraries(${SERVO_NODE_MAIN_NAME} ${SERVO_COMPONENT_NODE})
# ament_target_dependencies(${SERVO_NODE_MAIN_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

# # An example of pose tracking
# add_executable(${POSE_TRACKING_DEMO_NAME} src/cpp_interface_demo/pose_tracking_demo.cpp)
# target_link_libraries(${POSE_TRACKING_DEMO_NAME} ${POSE_TRACKING})
# ament_target_dependencies(${POSE_TRACKING_DEMO_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

# # Add executable to publish fake servo commands for testing/demo purposes
# add_executable(${FAKE_SERVO_CMDS_NAME} test/publish_fake_jog_commands.cpp)
# ament_target_dependencies(${FAKE_SERVO_CMDS_NAME}
#   rclcpp
#   geometry_msgs
#   std_srvs
# )


#############
## Install ##
#############

# Install Libraries
# install(
#   TARGETS
#     ${SERVO_LIB_NAME}
#     ${SERVO_LIB_NAME}_parameters
#     ${POSE_TRACKING}
#     ${SERVO_COMPONENT_NODE}
#     ${SERVO_CONTROLLER_INPUT}
#   EXPORT export_${PROJECT_NAME}
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

# # Install Binaries
# install(
#   TARGETS
#     ${SERVO_NODE_MAIN_NAME}
#     ${CPP_DEMO_NAME}
#     ${POSE_TRACKING_DEMO_NAME}
#     ${FAKE_SERVO_CMDS_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )


# Install include, launch, config directories
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
