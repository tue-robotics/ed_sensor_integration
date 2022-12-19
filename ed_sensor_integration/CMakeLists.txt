cmake_minimum_required(VERSION 3.0.2)
project(ed_sensor_integration)

add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree)
find_package(catkin REQUIRED COMPONENTS
  code_profiler
  ed
  ed_sensor_integration_msgs
  geolib2
  geometry_msgs
  image_geometry
  rgbd
  rgbd_image_buffer
  roscpp
  sensor_msgs
  tf2_ros
  tue_config
  tue_filesystem
  visualization_msgs
)

# ------------------------------------------------------------------------------------------------
#                                        CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ed_kinect
  CATKIN_DEPENDS code_profiler ed ed_sensor_integration_msgs geolib2 image_geometry rgbd rgbd_image_buffer roscpp tue_config visualization_msgs
  DEPENDS OpenCV PCL
)

# ------------------------------------------------------------------------------------------------
#                                           BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
  include
  SYSTEM
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
add_dependencies(ed_kinect ${catkin_EXPORTED_TARGETS})

add_library(ed_laser
  src/laser/updater.cpp
  include/ed/laser/updater.h
  include/ed/laser/entity_update.h
)
target_link_libraries(ed_laser ed_association ${catkin_LIBRARIES})
add_dependencies(ed_laser ${catkin_EXPORTED_TARGETS})

# ------------------------------------------------------------------------------------------------
#                                          PLUGINS
# ------------------------------------------------------------------------------------------------

add_library(ed_kinect_plugin
    src/kinect/kinect_plugin.cpp
    src/kinect/kinect_plugin.h
    src/kinect/ray_tracer.cpp
    src/kinect/ray_tracer.h
)
target_link_libraries(ed_kinect_plugin ed_kinect ${catkin_LIBRARIES})
add_dependencies(ed_kinect_plugin ${catkin_EXPORTED_TARGETS})

# ------------------------------------------------------------------------------------------------

add_library(ed_laser_plugin
    src/laser/laser_plugin.cpp
    src/laser/laser_plugin.h
)
target_link_libraries(ed_laser_plugin ed_laser ed_association ${catkin_LIBRARIES})

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
add_dependencies(ed_image_saver ${catkin_EXPORTED_TARGETS})

add_executable(ed_segmenter tools/segmenter.cpp)
target_link_libraries(ed_segmenter ed_kinect)

add_executable(ed_fitter_data tools/fitter_viz_data.cpp)
target_link_libraries(ed_fitter_data ed_kinect)

add_executable(ed_fitter_live tools/fitter_viz_live.cpp)
target_link_libraries(ed_fitter_live ed_kinect)

# ------------------------------------------------------------------------------------------------
#                                           TESTS
# ------------------------------------------------------------------------------------------------
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_furniture_fitting test/test_furniture_fit.cpp)
  target_link_libraries(test_furniture_fitting ed_kinect ${catkin_LIBRARIES})

  catkin_add_gtest(test_laser_fitting test/test_laser_segmenter.cpp)
  target_link_libraries(test_laser_fitting ed_laser ${catkin_LIBRARIES})
endif ()



