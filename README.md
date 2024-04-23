# Robotics Homework 2

Launch the bridge
```sh
ros2 launch thymioid main.launch device:="tcp:host=localhost;port=33333" simulation:=True name:=thymio0
```

Launch the homework tasks:


## Task 1 (done)
```sh
# Launch task 1
ros2 launch robotics_hw2 task1.launch.xml
```
Implementation code can be found in `task1_node.py`


## Task 2 (done)
```sh
# Launch task 2
ros2 launch robotics_hw2 task2.launch.xml
```
Implementation code can be found in `task2_node.py`


## Task 3 (done)
```sh
# Launch last compulsory task
ros2 launch robotics_hw2 compulsory.launch.xml
```
Implementation code can be found in `task3_node.py`

Multiple issues were encountered while doing the last task.
Firstly, odometry was used to turn the robot around, it proved to be
inconsistent and erroneous as the robot sometimes failed to fully turn around.
Furthermore, when relying on odometry to move 2m away from the wall the
distance sometimes was calculated wrongly, resulting in robot going past the
target point.
