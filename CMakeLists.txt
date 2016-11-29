cmake_minimum_required(VERSION 2.8.3)
project(ed_sensor_integration)

find_package(catkin REQUIRED COMPONENTS
  cb_planner_msgs_srvs
  ed
  geometry_msgs
  kdl_parser
  message_generation
  visualization_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}") # C++ 11

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
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
)

# ------------------------------------------------------------------------------------------------
#                                        CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ed_kinect
  # DEPENDS
  CATKIN_DEPENDS geometry_msgs message_runtime
)

# ------------------------------------------------------------------------------------------------
#                                           BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
  include
  third_party
  ${catkin_INCLUDE_DIRS}
)

# ------------------------------------------------------------------------------------------------
#                                         Third party
# ------------------------------------------------------------------------------------------------

##########     munkres          ##########
# Sources.
set (
    MunkresCppLib_SOURCES
    third_party/munkres/munkres.cpp
)
# Headers.
set (
    MunkresCppLib_HEADERS
    third_party/munkres/matrix.h
    third_party/munkres/matrix.cpp
    third_party/munkres/munkres.h
)
# Library.
add_library (
    munkres_cpp STATIC
    ${MunkresCppLib_SOURCES}
)

# ------------------------------------------------------------------------------------------------
#                                         LIBRARIES
# ------------------------------------------------------------------------------------------------

add_library(ed_association
  src/association_matrix.cpp
  include/ed_sensor_integration/association_matrix.h
  include/ed_sensor_integration/hungarian_method_association_matrix.h
)
target_link_libraries(ed_association munkres_cpp)

add_library(ed_kinect
    src/kinect/image_buffer.cpp
    include/ed/kinect/image_buffer.h
    src/kinect/fitter.cpp
    include/ed/kinect/fitter.h
    src/kinect/beam_model.cpp
    include/ed/kinect/beam_model.h
    src/kinect/mesh_tools.cpp
    include/ed/kinect/mesh_tools.h
    src/kinect/segmenter.cpp
    include/ed/kinect/segmenter.h
    src/kinect/association.cpp
    include/ed/kinect/association.h
    src/kinect/updater.cpp
    include/ed/kinect/updater.h
    include/ed/kinect/entity_update.h
    src/kinect/renderer.cpp
    include/ed/kinect/renderer.h
)
target_link_libraries(ed_kinect ed_association ${catkin_LIBRARIES})


# ------------------------------------------------------------------------------------------------
#                                          PLUGINS
# ------------------------------------------------------------------------------------------------

add_library(ed_kinect_plugin
    src/kinect/kinect_plugin.cpp
    src/kinect/kinect_plugin.h
)
target_link_libraries(ed_kinect_plugin ed_kinect ${catkin_LIBRARIES})
add_dependencies(ed_kinect_plugin ${PROJECT_NAME}_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})

# ------------------------------------------------------------------------------------------------

add_library(ed_laser_plugin
    src/laser/plugin.cpp
    src/laser/plugin.h
)
target_link_libraries(ed_laser_plugin ed_association ${catkin_LIBRARIES})

# ------------------------------------------------------------------------------------------------

add_library(ed_clearer_plugin
    src/clearer/clearer_plugin.cpp
    src/clearer/clearer_plugin.h
)
target_link_libraries(ed_clearer_plugin ${catkin_LIBRARIES})

# ------------------------------------------------------------------------------------------------
#                                           TOOLS
# ------------------------------------------------------------------------------------------------

add_executable(ed_image_saver tools/image_saver.cpp)
target_link_libraries(ed_image_saver ${catkin_LIBRARIES})
add_dependencies(ed_image_saver ${PROJECT_NAME}_gencpp)

add_executable(ed_segmenter tools/segmenter.cpp)
target_link_libraries(ed_segmenter ed_kinect)
