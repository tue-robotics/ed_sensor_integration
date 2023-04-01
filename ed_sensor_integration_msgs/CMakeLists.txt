cmake_minimum_required(VERSION 3.0.2)
project(ed_sensor_integration_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
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
    PlaceArea.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
)

# ------------------------------------------------------------------------------------------------
#                                        CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime
)
