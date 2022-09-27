# Tools

### image saver

* `ed_image_saver`: Shows RGBD image stream and allows to save images. Example usage (N.B.: use robot name as a namespace):

```bash
rosrun ed_sensor_integration ed_image_saver __ns:=hero
```

and once you see an image in a CV window popping up, press the space bar to save it. The tool will write a json and rgbd file to the current folder with a timestamp as filename.

### fitter

* `ed_fitter_live`: Shows RGB image stream and a visualisation to show the fitting process. Example usage (N.B.: use robot name as a namespace):

```bash
rosrun ed_sensor_integration ed_fitter_live <worldmodel_name> <entity_id> <rgbd_topic>
```

This visualises an image like the one below. With the original pose of the entity in blue and the newly fitted pose in yellow. The data used to fit the object is shown in purple.

![RGB image shown during fitter](https://user-images.githubusercontent.com/19806646/192571873-c9b42db6-8611-454b-b77f-8cae570de626.png)
![Visualisation of the fitting](https://user-images.githubusercontent.com/19806646/192572064-e583e295-8fcd-4a09-83ac-a71ff79d871d.png)

* `ed_fitter_data`: Similar to the above if one wishes to visualise the fitting using a saved rgbd image. Iterates over snapshots in a directory, performs a Fit and displays the result. To run:

```bash
rosrun ed_sensor_integration ed_fitter_data <stamp>.json <worldmodel_name> <entity_id>
```

### segmenter

* `ed_segmenter`: Iterates over snapshots in a directory, performs a Kinect update and displays the result. To run:

```bash
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
