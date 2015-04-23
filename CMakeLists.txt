cmake_minimum_required(VERSION 2.8.3)
project(ed_sensor_integration)

find_package(catkin REQUIRED COMPONENTS
    ed
    kdl_parser
)

# find_package(Boost REQUIRED COMPONENTS system program_options)
# find_package(PCL REQUIRED)
# find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------

# Generate messages
# add_message_files(
#    FILES
#    message1.msg
#    ...
# )

# Generate services
# add_service_files(
#    FILES
#    service1.srv
#    ...
# )

# Generate added messages and services with any dependencies listed here
# generate_messages(
#    DEPENDENCIES
#    geometry_msgs
#    ...
# )

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES bla
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

file(GLOB_RECURSE HEADER_FILES include/*.h)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# ------------------------------------------------------------------------------------------------

add_library(ed_sensor_integration_properties
    src/properties/convex_hull_info.cpp
    src/properties/convex_hull_calc.cpp
    src/properties/pose_info.cpp
)
target_link_libraries(ed_sensor_integration_properties ${catkin_LIBRARIES})

# ------------------------------------------------------------------------------------------------

add_library(kinect_association
    src/kinect/almodules/point_normal_alm.cpp
    src/kinect/almodules/polygon_height_alm.cpp
    src/kinect/almodules/world_model_renderer.cpp
    ${HEADER_FILES}
)
add_dependencies(kinect_association ${PROJECT_NAME}_gencpp)

add_library(kinect_segmentation
    src/kinect/segmodules/euclidean_clustering_sm.cpp
    src/kinect/segmodules/floodfill_sm.cpp
)
add_dependencies(kinect_segmentation ${PROJECT_NAME}_gencpp)

target_link_libraries(kinect_segmentation kinect_association)

add_library(ed_kinect_plugin
    src/kinect/plugin.cpp
    src/kinect/plugin.h
    src/kinect/sampling_projector_localizer.cpp
    src/kinect/sampling_render_localizer.cpp
    src/kinect/visualization.cpp
)
target_link_libraries(ed_kinect_plugin kinect_segmentation kinect_association ${catkin_LIBRARIES})

# ------------------------------------------------------------------------------------------------

add_library(ed_kinect2_plugin
    src/kinect2/plugin.cpp
    src/kinect2/plugin.h
    src/kinect2/visualization.cpp
    src/kinect2/visualization.h
    src/kinect2/association_matrix.cpp
    src/kinect2/association_matrix.h
)
target_link_libraries(ed_kinect2_plugin ed_sensor_integration_properties ${catkin_LIBRARIES})

# ------------------------------------------------------------------------------------------------

add_library(ed_laser_plugin
    src/laser/plugin.cpp
    src/laser/plugin.h
)
target_link_libraries(ed_laser_plugin ed_sensor_integration_properties ${catkin_LIBRARIES})

#add_library(ed_sensor_integration_plugin
#	src/sensor_integration_plugin.cpp
#	src/color_table.cpp
#)
#target_link_libraries(ed_sensor_integration_plugin ${catkin_LIBRARIES})

# ------------------------------------------------------------------------------------------------

add_executable(test_convex_hull test/test_convex_hull.cpp)
target_link_libraries(test_convex_hull ed_sensor_integration_properties)
