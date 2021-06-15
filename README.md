# arl_simulator_control
This repository contains the utility tools to interact with the **Phoenix** simulator.
## Installation of ARL Autonomy Stack (Phoenix)
In order to install Phoenix you first need access to the Gitlab repo of Phoenix. Please ask Garrett Warnell for the authorization. After having your access, follow the instructions in the [gitlab repo](https://gitlab.sitcore.net/aimm/phoenix-r1) to install the Phoenix simulator and its prerequisites. After the basic Phoenix installation, you should follow the instructions in [this page](https://gitlab.sitcore.net/aimm/phoenix-r1/-/blob/master/src/simulation/unity/README.md) to install and activate Unity game engine for the Phoenix simulator.
## Docker Usage
You will run the simulator in the docker space. Hence, knowing how to control docker images and containers is essential. In this section, I present some of the most necessary commands to interact with docker space.

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
```
After running the simulator, bunch of windows will pop up. You will mainly need to use the Unity game engine and RVIZ windows. On the Unity window, if you press <kbd>t</kbd> + <kbd>[</kbd> the camera will focus on the robot agent. By pressing <kbd>rigth mouse</kbd> and moving cursor, you can change the orientation of the camera. By pressing <kbd>rigth mouse</kbd> and playing with arrowkeys you can change the relative position of the camera with respect to the robot. On RVIZ window, by pressing <kbd>t</kbd> you can activate `goto_region` node and send it waypoint commands using the cursor.

### Tips for the lejeune environment

To match the orientation of the ground truth map and the orientation of the simulation, use `yaw:=3.1` while running the simulator.

```
phxlaunch phoenix_unity_launch experiment.xlaunch x:=10 y:=10 z:=1 yaw:=0 environment:=lejeune_emout
```

## Map extraction from the Unity game engine

To extract a map or gridworld from the existing Unity environments, you need to follow three steps.

### 1. Scene OBJ Exporter(you need to do it on a Windows machine)
Using this scene obj exporter we will extract the Unity environment to a point cloud like looking file (.obj).

Exporter sometimes doesn't work when imported after the asset files if the unity project is too loaded. Follow the following order:

* First open a new Unity project.
* Then import the [exporter](https://assetstore.unity.com/packages/tools/utilities/scene-obj-exporter-22250) from the asset store on Unity.
* Then move the asset files under the `Assets` folder in the `\Documents\Unity\"project_name"\`.

### 2. .obj to .pts conversion
You need this step to convert .obj file to a simpler point cloud data format. 
* On [cloudcompare](http://www.cloudcompare.org) application, open the .obj file and select the mesh you want to convert to .pts file.
* Use 'Edit > Mesh > Sample Points'.
* Then select **only .sampled** from DBtree on the left panel of cloudcompare.
* After selection, save it as ASCII(.xyz, .pts) file.

### 3. .pts to gridworld conversion
To obtain a gridworld out of a .pts file, you can do all the processing using the functions in `position_shifter.py` file. 
* First find the extremities of the point cloud data in .pts file using `extremum_points_txt` function and decide on the boundaries of your gridworld.
* Then convert your point cloud to a gridworld using `costmap_gen_txt(.pts file, right_end, left_end, upper_end, lower_end, grid_space)`.

## Working with costmaps
On Phoenix, the simulator creates a costmap by itself using the observations of the robot and an internal reward function mechanism. To extract this costmap, you can use `costmap_subs.py` file and get a 2D costmap array in .npy format. Then you can also combine the information in the costmap file into your gridworld using `costmap_gen_txt` function in the `position_shifter.py` file as follows
```
costmap_gen_txt(.pts file, right_end, left_end, upper_end, lower_end, grid_space,costmap_file = costmap file)
```

## Waypoint control of the robot

To perform the waypoint control of the robot, you need to use the `action_client.py` by feeding it a trajectory file which contains x-y coordinates for the waypoints. You can use the format in `traj_uPOMDP-uncertain-1.txt` file or you can change the input format in `action_client.py` file.

## Extra

You can find the extracted point clouds for the two main environments of Phoenix under `point_clouds` directory. However, they have different formats. The example runs in the `position_shifter.py` are written regarding this format difference.

Also a ground truth overhead map of the lejeune_emout environment can be found in this [link](https://gitlab.sitcore.net/aimm/phoenix-r1/-/wikis/Tutorials/Miscellaneous/Creating-Semantic-and-Obstacle-Maps-from-Unity).

You can download the asset files for the lejeune environment from this [gitlab link](https://gitlab.sitcore.net/arl/robotics-simulation/arl-unity-lejeune-emout).
