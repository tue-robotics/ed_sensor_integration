cmake_minimum_required(VERSION 2.8.3)
project(ed_sensor_integration_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------

add_message_files(
  FILES
    GUIAction.msg
    ImageBinary.msg
)

add_service_files(
  FILES
    GetImage.srv
    Update.srv
    RayTrace.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

# ------------------------------------------------------------------------------------------------
#                                        CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs
)
