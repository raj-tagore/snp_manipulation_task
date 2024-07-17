# To run this simple simulation in gazebo

- First create a catkin workspace
- clone this repository in the src folder
```sh
git clone https://github.com/raj-tagore/snp_manipulation_task
```
- then run `catkin_make`
- After this you can launch the gazebo simulation using the following command
```sh
roslaunch snp_manipulation_task arm_ur5.launch world:=`rospack find snp_manipulation_task`/worlds/arm_empty.world  x:=-0.10 y:=0 z:=0.515
```
- This should launch the ros server and spawn a robot
- run this command to have the robot execute the pick and place task
```sh
rosrun snp_manipulation_task pick_place_task.py
```