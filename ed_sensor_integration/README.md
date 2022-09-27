# ED sensor integration

## Plugins

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

## Tools
Several tools are available for testing the features of this repository. These can be found in the [tools](tools) along with their documentation.

## Issues with current toolchain

* The json resulting from `ed_image_saver` only contains sensor pose, rgbd filename and timestamp, not the world model. This makes sense in a way (it saves images, not the world model). However, the `segmenter` tool cannot be used easily because the right data is not present.
* `Updater.update` is an long messy function, making it difficult to test standalone parts.
* No unit tests
* World model cannot be serialized entirely. Therefore it's difficult to create realistic situations.

## Approach

* Create a simple world model (check, e.g., [this test](https://github.com/tue-robotics/ed/blob/master/test/test_wm.cpp) for inspiration) in a unit test containing a piece of furniture
  * Move the piece of furniture
  * Render an image from this (for inspiration, check the [fast simulator](https://github.com/tue-robotics/fast_simulator/blob/master/src/Kinect.cpp) for inspiration)
  * Try to update the entity pose based on this image. When does/doesn't it move?
