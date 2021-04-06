# SRC2-object-detection
roslaunch src2_object_detection scout_inference.launch
# Instructions for launching the Approach base service:
rosrun  src2_object_detection inference.py
roslaunch driving_control scout_driving_control_all.launch
roslaunch laser_tools_src2 scout_laser_tools_R3.launch
rosrun range_to_base rangeToBaseService.py
rosrunrc2_object_detection approach_base_station_service.py

rosservice call /approach_base_station "approach_base_station:
  data: true"

# For visualizing the bounding boxes from the classifier:
rosrun src2_object_detection plot_boxes_real_time.py
rosrun image_view image_view image:=/small_scout_1/BoundingBox/Image

# Disparity images frp, wvu vo (used to obtaint point cloud)
roslaunch wvu_vo_ros vo_pipeline.launch


Dependencies:

```python2.7 -m pip install Pillow```

```python2.7 -m pip install setuptools```

```python2.7 -m pip install tensorflow==2.1.0```

```python2.7 -m pip install keras==2.3.1```
