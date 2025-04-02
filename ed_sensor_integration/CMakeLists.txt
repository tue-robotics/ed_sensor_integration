cmake_minimum_required(VERSION 3.5)
project(ed_sensor_integration)

#add_compile_options(-Wall -Werror=all)
#add_compile_options(-Wextra -Werror=extra)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree)
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
  DEPENDS OpenCV PCL
)

# ------------------------------------------------------------------------------------------------
#                                           BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
  include
  SYSTEM
  ${OpenCV_INCLUDE_DIRS}
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

add_executable(ed_image_saver tools/image_saver.cpp)
target_link_libraries(ed_image_saver ${catkin_LIBRARIES})
add_dependencies(ed_image_saver ${catkin_EXPORTED_TARGETS})

add_executable(ed_segmenter tools/segmenter.cpp)
target_link_libraries(ed_segmenter ed_kinect ${catkin_LIBRARIES})

add_executable(ed_fitter_data tools/fitter_viz_data.cpp)
target_link_libraries(ed_fitter_data ${PROJECT_NAME}_console_bridge ed_kinect ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(ed_fitter_live tools/fitter_viz_live.cpp)
target_link_libraries(ed_fitter_live ${PROJECT_NAME}_console_bridge ed_kinect ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

set(PROJECT_SOURCES
        /home/amigo/Documents/repos/hero_sam/inference_utils.h
        /home/amigo/Documents/repos/hero_sam/inference_utils.cpp
        /home/amigo/Documents/repos/hero_sam/inference.h
        /home/amigo/Documents/repos/hero_sam/inference.cpp
)

add_library(${PROJECT_NAME} ${PROJECT_SOURCES})
target_link_libraries(ed_kinect ${PROJECT_NAME} ${catkin_LIBRARIES})
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
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# -------------- ONNXRuntime  ------------------#
set(ONNXRUNTIME_VERSION 1.21.0)
set(ONNXRUNTIME_ROOT "/home/amigo/Documents/repos/hero_sam/onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}")

include_directories(${ONNXRUNTIME_ROOT}/include)
# Link OpenCV libraries along with ONNX Runtime
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so)
# and put it in the same folder of the executable file
configure_file(/home/amigo/Documents/repos/hero_sam/coco.yaml ${CMAKE_CURRENT_BINARY_DIR}/coco.yaml COPYONLY)

# Copy yolov8n.onnx file to the same folder of the executable file
configure_file(/home/amigo/Documents/repos/hero_sam/yolo11m.onnx ${CMAKE_CURRENT_BINARY_DIR}/yolo11m.onnx COPYONLY)
target_link_libraries(ed_kinect ${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so ${catkin_LIBRARIES})
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
