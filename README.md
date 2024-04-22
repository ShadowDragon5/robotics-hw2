# README

Launch the bridge
```sh
ros2 launch thymioid main.launch device:="tcp:host=localhost;port=33333" simutaion:=True name:=thymio0
```

Launch the homework tasks:
```sh
# Launch task 1
ros2 launch robotics_hw2 task1.launch.xml

# Launch task 2
ros2 launch robotics_hw2 task2.launch.xml

# Launch last compulsory task
ros2 launch robotics_hw2 compulsory.launch.xml
```
