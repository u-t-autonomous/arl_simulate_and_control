# arl_simulator_control
This repository contains the utility tools to interact with the **Phoenix** simulator.
## Docker Usage
Use `docker ps` to list the active containers with their **Container ID**'s.
If there is no active container create one in `phoenix-r1` directory as follows:
```
#Allow access to X11 (potentially unsafe, use with caution)
xhost +si:localuser:root

catkin-docker
```
If there is one, you can get a bash shell in the container using `docker exec -it <container_id> /bin/bash`.
To delete a container use `docker rm <container_id>`.
Finally after each start of a new bash shell complete the set up using `source docker-build/install/setup.bash`.
## Running the simulator
```
phxlaunch phoenix_unity_launch experiment.xlaunch launch_unity:=true environment:=floodedgrounds
```
To change the spawn location of the robot set the following arguments:
```
phxlaunch phoenix_unity_launch experiment.xlaunch x:=10 y:=10 z:=1 yaw:=0
