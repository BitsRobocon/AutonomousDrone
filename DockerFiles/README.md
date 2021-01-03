
# How to setup the work env in docker

## Install and setup stuff

1. Install docker if it is not installed yet.
2. Clone this folder.
3. Inside this folder, build this docker file as:
`$ docker build --build-arg user=$USER -t work_env .`
It will download ros, gazebo, px4 and some utilities, so it might take time to install.
4. ONLY NEED TO DO THIS ONCE.
Create a new container with the image:
`$ docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix work_env bash`

    In another terminal, run `docker ps`. copy the top-most container ID(or container name).
save this container ID(or container_NAME) somewhere.

5. To exit from this docker container in the original bash shell, type `$ exit` at the prompt.

## Reopening the same container
1. Do `$ docker ps` to check if the container is running.
If not, start it with `$ docker start container_ID` or `$ docker start container_NAME`.

2. Connect to the container with:
`$ docker attach -it container_NAME bash`
or
`$ docker exec -it container_NAME bash`

3. To connect to the same container in another bash shell, type:
`$ docker exec -it container_NAME bash`

## Some more details
If everything works correctly, you should get a prompt of the type
`username@container_ID$ `
on running the container, where username is the same as of the user you logged in with.
The root password for this user is the same as the username. To change it, simply type
`$ passwd`
at the prompt.

The catkin workspace is in the home directoy, `~/catkin_ws`.

## Known issues if you have a GPU
The easiest way to confirm that it is a GPU issue is by running the following commands.
```bash
$ sudo apt-get install mesa-utils
$ glxgears
```
`glxgears` is a demo that draws three rotating gears, and prints out frame rate information to stdout.
If the command is executed successfully then it would imply this is not a GLX issue, that is not a OpenGL issue.
```bash
X Error of failed request:  BadValue (integer parameter out of range for operation)
```
*Possible solution*: [AskUbuntu-1](https://askubuntu.com/questions/893922/ubuntu-16-04-gives-x-error-of-failed-request-badvalue-integer-parameter-out-o), [AskUbuntu-2](https://askubuntu.com/questions/1226170/x-error-of-failed-request-badvalue-integer-parameter-out-of-range-for-operatio)

```bash
No matching fbConfigs or visuals found
libGL error: failed to load driver: swrast
```
*Possible solution*: https://github.com/jessfraz/dockerfiles/issues/253

If you are still unable to resolve the issue, [switch back to built-in Nouveau driver]((https://linuxconfig.org/how-to-uninstall-the-nvidia-drivers-on-ubuntu-20-04-focal-fossa-linux)) which you might have possibly replaced with Nvidia proprietary drivers.

You will need to recreate an image that is repeat the steps from
> Create a new container with the image:
`$ docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix work_env bash`

## Issues with dbus
If while running **rviz** (`rosrun rviz rviz`), you get a error as below: ([similar](https://answers.ros.org/question/301056/ros2-rviz-in-docker-container/))
```bash
D-Bus not built with -rdynamic so unable to print a backtrace
```

You need to create the image by using below command
```bash
$ docker run --net=host --priviliged -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix work_env bash
```

## Success?
There is no thumb rule to determine if the docker container has been successfully setup though you can quickly open below programs to verify if they are running.
* Rviz
```bash
$ roscore
# In a separate terminal
$ roslaunch rviz rviz
```

* Gazebo
```bash
$ gazebo
```

After successfully creating the container (that is gazebo & rviz opening up) without any errors. Follow below steps on the bash of container.

# All steps within the docker container
**Note**: Your username and hostname for the container shall vary from mine (which is `hardik@Inspiron-5580`)

Install your favourite code editor, for this setup I have used **nano**
```bash
hardik@Inspiron-5580:~$ sudo apt-get update && sudo apt-get install nano
```

```bash
# The docker container should contain the following dirs
hardik@Inspiron-5580:~$ ls
PX4-Autopilot  catkin_ws
```
## `Build px4 package`
```bash
hardik@Inspiron-5580:~$ mv -r ~/PX4-Autopilot/ ~/catkin_ws/src/PX4-Autopilot/
hardik@Inspiron-5580:~$ cd ~/catkin_ws/src/PX4-Autopilot/
hardik@Inspiron-5580:~/catkin_ws/src/PX4-Autopilot$ catkin build --this
```
## `Build robot_setup_tf package`
```bash
hardik@Inspiron-5580:~$ nano ~/catkin_ws/src/AutonomousDrone/robot_setup_tf/CMakeLists.txt
```
In the text editor opened comment the following lines and then save and close the file.
1. Line 139
```
# add_executable(gazebo_link_pose_publisher scripts/gazebo_link_pose_publisher.cpp)
```
2. Line 156-158
```
# target_link_libraries(gazebo_link_pose_publisher
#   ${catkin_LIBRARIES}
# )
```
## `Build other packages`
```bash
hardik@Inspiron-5580:~$ cd catkin_ws/
hardik@Inspiron-5580:~/catkin_ws$ catkin build
```
## Check setup so far:
```
hardik@Inspiron-5580:~/catkin_ws$ source ~/catkin_ws/devel/setup.bash
roslaunch velodyne_description example.launch
```
> The velodyne sensor should be displayed in both **rviz** and **gazebo**, cool now close them.

## `Moving requisite px4files`
```bash
hardik@Inspiron-5580:~$ mkdir ~/.gazebo/models
hardik@Inspiron-5580:~$ mv -r ~/catkin_ws/src/AutonomousDrone/px4_files/willowgarage ~/.gazebo/models/willowgarage
hardik@Inspiron-5580:~$ mv ~/catkin_ws/src/AutonomousDrone/px4_files/willowgarage.world ~/catkin_ws/src/PX4-Autopilot/Tools/sitl_gazebo/worlds/willowgarage.world
hardik@Inspiron-5580:~$ mv ~/catkin_ws/src/AutonomousDrone/px4_files/mavros_posix_sitl.launch ~/catkin_ws/src/PX4-Autopilot/launch/mavros_posix_sitl.launch
hardik@Inspiron-5580:~$ mv ~/catkin_ws/src/AutonomousDrone/px4_files/posix_sitl.launch ~/catkin_ws/src/PX4-Autopilot/launch/posix_sitl.launch
hardik@Inspiron-5580:~$ mv ~/catkin_ws/src/AutonomousDrone/px4_files/if750a.sdf ~/catkin_ws/src/PX4-Autopilot/Tools/sitl_gazebo/models/if750a/if750a.sdf
```
## `Setting up enviroment variables`
```bash
hardik@Inspiron-5580:~$ nano ~/catkin_ws/devel/setup_env.bash
```
In the text editor opened, copy and paste the following lines. Save and close the file.
```bash
# Inside setup_env.bash paste this
source ~/catkin_ws/devel/setup.bash
declare FIRMWARE_PATH="${HOME}/catkin_ws/src/PX4-Autopilot"
declare VELODYNE_PATH="${HOME}/catkin_ws/src/AutonomousDrone/sensors/velodyne_simulator/velodyne_description/models"
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${VELODYNE_PATH}
source ${FIRMWARE_PATH}/Tools/setup_gazebo.bash ${FIRMWARE_PATH} ${FIRMWARE_PATH}/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${FIRMWARE_PATH}
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${FIRMWARE_PATH}/Tools/sitl_gazebo
```
In order to source the environment variables, run below command.
```bash
hardik@Inspiron-5580:~$ source ~/catkin_ws/devel/setup_env.bash
```

In separate terminals, run the following commands and gazebo and rviz should show up.
```bash
hardik@Inspiron-5580:~$ roslaunch px4 mavros_posix_sitl.launch vehicle:=if750a
```
```bash
hardik@Inspiron-5580:~$ rosrun rviz rviz
```

You can follow setting up [autonomous drone environment](https://drive.google.com/file/d/1nochEnGr1CX2Fpboi6eZv5NZcueceh49/view) from 1:23:00 to understand how to proceed further.
