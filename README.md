# MIE443 Contest 1: Autonomous Robot Search of an Environment - Group 18
## Group Members
**Farhan Wadia - 1003012606**

**Henry Cueva Barnuevo - 1003585122**

**Yilin Huang - 1003145232**

## Execution Commands
1. Place this repository in the `catkin_ws/src` folder of the file system.

2. Launch the simulated world in Gazebo by entering the below command in a terminal window:
```bash
roslaunch mie443_contest1 turtlebot_world.launch world:=practice
```
3. Launch gmapping in another terminal window:
```bash
roslaunch mie443_contest1 gmapping.launch
```
4. Launch RVIZ to visualize the map in a seperate terminal window (optional):
```bash
roslaunch turtlebot_rviz_launchers view_navigation.launch
```
5. Run the contest1.cpp file by entering the below command in a seperate terminal window:
```bash
rosrun mie443_contest1 contest1
```
6. At the end of the contest, use another terminal window to save the map:
```bash
rosrun map_server map_saver -f /home/turtlebot/contest1map_group18
```
The output map will be saved as `contest1map_group18.yaml` and `contest1map_group18.pgm` in the home directory of the file system after running this command.
