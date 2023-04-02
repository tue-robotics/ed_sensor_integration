cmake_minimum_required(VERSION 3.0.2)
project(ed_sensor_integration)

add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

find_package (Python COMPONENTS Interpreter REQUIRED)
execute_process(
 COMMAND ${Python_EXECUTABLE} -c "import torch; print(torch.__path__[0])"
 OUTPUT_VARIABLE TORCH_LIBRARY_PATH
 OUTPUT_STRIP_TRAILING_WHITESPACE
)
message(WARNING "TORCH_LIBRARY_PATH: ${TORCH_LIBRARY_PATH}")

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree)
find_package(Torch 1.10 REQUIRED
  PATHS "${TORCH_LIBRARY_PATH}/share/cmake"
)
message(WARNING "TORCH_INCLUDE_DIRS: ${TORCH_INCLUDE_DIRS}")
find_package(catkin REQUIRED COMPONENTS
  code_profiler
  ed
  ${PROJECT_NAME}_msgs
  geolib2
  geometry_msgs
  image_geometry
  rgbd
  rgbd_image_buffer
  rosconsole_bridge
  roscpp
  sensor_msgs
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
  CATKIN_DEPENDS code_profiler ed ${PROJECT_NAME}_msgs geolib2 image_geometry rgbd rgbd_image_buffer roscpp tue_config visualization_msgs
  DEPENDS OpenCV PCL Torch
)

# ------------------------------------------------------------------------------------------------
#                                           BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
  include
  SYSTEM
  ${OpenCV_INCLUDE_DIRS}
  ${TORCH_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# ------------------------------------------------------------------------------------------------
#                                         LIBRARIES
# ------------------------------------------------------------------------------------------------

add_library(${PROJECT_NAME}_console_bridge
  # Foward geolib2 and tue_filesystem logging to rosconsole
  src/rosconsole_bridge.cpp
)

add_library(ed_association
  include/${PROJECT_NAME}/association_matrix.h
  src/association_matrix.cpp
)

add_library(ed_kinect
    include/ed/kinect/association.h
    include/ed/kinect/beam_model.h
    include/ed/kinect/entity_update.h
    include/ed/kinect/fitter.h
    include/ed/kinect/mesh_tools.h
    include/ed/kinect/renderer.h
    include/ed/kinect/segmenter.h
    include/ed/kinect/updater.h

    src/kinect/association.cpp
    src/kinect/beam_model.cpp
    src/kinect/fitter.cpp
    src/kinect/mesh_tools.cpp
    src/kinect/renderer.cpp
    src/kinect/segmenter.cpp
    src/kinect/updater.cpp
)
target_link_libraries(ed_kinect ${PROJECT_NAME}_console_bridge ed_association ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(ed_kinect ${catkin_EXPORTED_TARGETS})

add_library(ed_laser
  include/ed/laser/entity_update.h
  include/ed/laser/updater.h
  src/laser/updater.cpp
)
target_link_libraries(ed_laser ${PROJECT_NAME}_console_bridge ed_association ${catkin_LIBRARIES})
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
target_link_libraries(ed_kinect_plugin ${PROJECT_NAME}_console_bridge ed_kinect ${catkin_LIBRARIES})
add_dependencies(ed_kinect_plugin ${catkin_EXPORTED_TARGETS})

# ------------------------------------------------------------------------------------------------

add_library(ed_laser_plugin
    src/laser/laser_plugin.cpp
    src/laser/laser_plugin.h
)
target_link_libraries(ed_laser_plugin ${PROJECT_NAME}_console_bridge ed_laser ed_association ${catkin_LIBRARIES})

# ------------------------------------------------------------------------------------------------

add_library(ed_clearer_plugin
    src/clearer/clearer_plugin.cpp
    src/clearer/clearer_plugin.h
)
target_link_libraries(ed_clearer_plugin ${catkin_LIBRARIES})

# ------------------------------------------------------------------------------------------------
#                                           TOOLS
# ------------------------------------------------------------------------------------------------

add_executable(test_torch tools/test_torch.cpp)
target_link_libraries(test_torch ${TORCH_LIBRARIES})

add_executable(test_yolo tools/test_yolo.cpp)
target_link_libraries(test_yolo ${TORCH_LIBRARIES})

add_executable(ed_image_saver tools/image_saver.cpp)
target_link_libraries(ed_image_saver ${catkin_LIBRARIES})
add_dependencies(ed_image_saver ${catkin_EXPORTED_TARGETS})

add_executable(ed_segmenter tools/segmenter.cpp)
target_link_libraries(ed_segmenter ed_kinect ${catkin_LIBRARIES})

add_executable(ed_fitter_data tools/fitter_viz_data.cpp)
target_link_libraries(ed_fitter_data ${PROJECT_NAME}_console_bridge ed_kinect ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(ed_fitter_live tools/fitter_viz_live.cpp)
target_link_libraries(ed_fitter_live ${PROJECT_NAME}_console_bridge ed_kinect ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

# ------------------------------------------------------------------------------------------------
#                                          Install
# ------------------------------------------------------------------------------------------------

install(
  FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(
  TARGETS
    ed_association
    ed_clearer_plugin
    ed_kinect
    ed_kinect_plugin
    ed_laser
    ed_laser_plugin
    ${PROJECT_NAME}_console_bridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  TARGETS
    ed_fitter_data
    ed_fitter_live
    ed_image_saver
    ed_segmenter
    test_torch
    test_yolo
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ------------------------------------------------------------------------------------------------
#                                           TESTS
# ------------------------------------------------------------------------------------------------
if (CATKIN_ENABLE_TESTING)
  find_package(catkin_lint_cmake REQUIRED)
  catkin_add_catkin_lint_test("-W2 --ignore HEADER_OUTSIDE_PACKAGE_INCLUDE_PATH")

  catkin_add_gtest(test_furniture_fitting test/test_furniture_fit.cpp)
  target_link_libraries(test_furniture_fitting ${PROJECT_NAME}_console_bridge ed_kinect ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

  catkin_add_gtest(test_laser_fitting test/test_laser_segmenter.cpp)
  target_link_libraries(test_laser_fitting ${PROJECT_NAME}_console_bridge ed_laser ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
endif ()
