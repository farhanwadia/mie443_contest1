# MIE443 Contest 1: Autonomous Robot Search of an Environment - Group 18
## Group Members
Farhan Wadia

Henry Cueva

Yilin Huang

## Execution Commands
1. Begin by launching the simulated world
```bash
roslaunch mie443_contest1 turtlebot_world.launch world:=practice
```
2. Launch gmapping in another terminal
```bash
roslaunch mie443_contest1 gmapping.launch
```
3. Launch RVIZ to visualize map in a seperate terminal (optional)
```bash
roslaunch turtlebot_rviz_launchers view_navigation.launch
```
4. Run the contest1.cpp file by entering in a seperate terminal:
```bash
rosrun mie443_contest1 contest1
```
5. At the end of the contest, use another terminal to save the map:
```bash
rosrun map_server map_saver -f /home/turtlebot/
```

## Git Commands
```bash

# add new files
git add <your file>

# commit your change
git commit -m "add your msg here"

# push your change to remote repository (online)
git push remote <branch name>

# create a branch
git checkout -b <branch name>

# move to a different branch
git checkout <branch name>

# check my status
git status

# uncommit change/unadd files
git reset

# merge one branch into another
git checkout <branch to merge into>
git merge <branch to merge>
git push origin <branch to merge into>
```
