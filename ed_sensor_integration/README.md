# Plugins
### ED Kinect plugin
Compares recent sensor image with an image rendered from the world model. Based on this comparison:
* the pose of existing entities is updated
* new entities are added
* entities are removed from the world model

Interfaces:
* foo
* bar

### ED Laser plugin
* `ed_laser_plugin`

Interfaces:
* foo
* bar

### ED Clearer plugin
* `ed_clearer_plugin`

Interfaces:
* foo
* bar

# Tools
* `ed_image_saver`: Shows RGBD image stream and allows to save images. Example usage (N.B.: use robot name as a namespace):
```
rosrun ed_sensor_integration ed_image_saver __ns:=hero
```
and once you see an image in a CV window popping up, press the space bar to save it. The tool will write a json and rgbd file to the current folder with a timestamp as filename.
* `ed_segmenter`: Iterates over snapshots in a directory, performs a Kinect update and displays the result. To run:
```
rosrun ed_sensor_integration ed_segmenter <stamp>.json
```
 This includes the entire 'update' sequence, including (but not limited to):
    * Updating entity position
    * Updating sensor pose
    * Removing the background
    * Clearing convex hulls that are no longer there
    * Clustering
    * Merging overlapping convex hulls
    * Associate and update entities
    * Removing entities that are not associated

# Issues with current toolchain
* The json resulting from `ed_image_saver` only contains sensor pose, rgbd filename and timestamp, not the world model. This makes sense in a way (it saves images, not the world model). However, the `segmenter` tool cannot be used easily because the right data is not present.
* `Updater.update` is an long messy function, making it difficult to test standalone parts.
* No unit tests
* World model cannot be serialized entirely. Therefore it's difficult to create realistic situations.

# Approach
* Create a simple world model (check, e.g., [this test](https://github.com/tue-robotics/ed/blob/master/test/test_wm.cpp) for inspiration) in a unit test containing a piece of furniture
    * Move the piece of furniture
    * Render an image from this (for inspiration, check the [fast simulator](https://github.com/tue-robotics/fast_simulator/blob/master/src/Kinect.cpp) for inspiration)
    * Try to update the entity pose based on this image. When does/doesn't it move?
