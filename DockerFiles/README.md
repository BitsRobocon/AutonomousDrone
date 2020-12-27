
# How to setup the work env in docker

## Install and setup stuff

1. Install docker if it is not installed yet.
2. Clone this folder.
3. Inside this folder, build this docker file as:  
`$ docker build --build-arg user=$USER -t work_env .`  
It will download ros, gazebo, px4 and some utilities, so it might take time to install.
4. ONLY NEED TO DO THIS ONCE.  
Create a new container with the image:  
`$ docker run --it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix work_env bash` 

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

