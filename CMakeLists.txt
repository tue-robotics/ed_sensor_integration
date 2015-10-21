cmake_minimum_required(VERSION 2.8.3)
project(ed_sensor_integration)

find_package(catkin REQUIRED COMPONENTS
    ed
    kdl_parser
    blackboard
    cb_planner_msgs_srvs
)

# find_package(Boost REQUIRED COMPONENTS system program_options)
# find_package(PCL REQUIRED)
# find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------

# Generate messages
add_message_files(
   FILES
   ImageBinary.msg
   GUIAction.msg
)

# Generate services
add_service_files(
  FILES
  GetImage.srv
  Update.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    geometry_msgs
#    ...
)

# ------------------------------------------------------------------------------------------------
#                                        CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES bla
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# ------------------------------------------------------------------------------------------------
#                                           BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# ------------------------------------------------------------------------------------------------
#                                         LIBRARIES
# ------------------------------------------------------------------------------------------------

add_library(ed_association
    src/association_matrix.cpp
    include/ed_sensor_integration/association_matrix.h
)

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
add_dependencies(ed_kinect_plugin ${PROJECT_NAME}_gencpp)

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

add_executable(ed_segmenter tools/segmenter.cpp)
target_link_libraries(ed_segmenter ed_kinect)

add_executable(ed_old_segmenter tools/old_segmenter.cpp)
target_link_libraries(ed_old_segmenter ${catkin_LIBRARIES})
#target_link_libraries(ed_old_segmenter ed_kinect)

