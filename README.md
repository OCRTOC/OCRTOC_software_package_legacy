# Open Cloud Robot Table Organization Challenge (OCRTOC)
This is the OCRTOC software package. For more information on OCRTOC please visit the homepage: [www.ocrtoc.org](http://www.ocrtoc.org)

To take part in OCRTOC, you need to develop your own solution using this software package. After uploading your solution to the competition platform, the performance of your solution will be evaluated.

For the simulation stage, we support two simulators: Gazebo and Sapien. You can choose either of them to test your solution on your local machine. On the competition platform we use both simulators to evaluate your solution. As long as your solution works fine with either of the two simulators, it is good enough for the qualification of the real robot stage.

For the real robot stage, hardware drivers will be provided in this software package (around the end of August). Your solution will be tested on real robot hardware. The software interfaces for sensor readings and robot control are the same for both simulation and the real robot hardware. So you will not encounter interface issues when transferring your solution from simulation to the real robot.


## The docker image
In order for your solution to be executable both on your local machine and our competition platform (with cloud computing), we provide you a docker image with a lot of pre-installed software. You need to develop and test your solution within this docker image.

**Operation system**

We highly recommend you to use Ubuntu 18.04 as your operation system, because the provided software package and docker image are guaranteed to work on this OS.

**Install dependencies**

1. Install docker engine on Ubuntu. [https://docs.docker.com/engine/install/ubuntu/](https://docs.docker.com/engine/install/ubuntu/)
1. Install Nvidia container runtime [https://www.celantur.com/blog/run-cuda-in-docker-on-linux/](https://www.celantur.com/blog/run-cuda-in-docker-on-linux/)

**Get the docker image**

The docker image is not part of the OCRTOC software package and needs be downloaded separately. You can use either of the following addresses to download the docker image:

```bash
# Mirror address of China (ShangHai)
sudo docker pull registry.cn-shanghai.aliyuncs.com/tcc-public/ocrtoc:release1.0
# Mirror address of the United States (Silicon Valley)
sudo docker pull registry.us-west-1.aliyuncs.com/tcc-public/ocrtoc:release1.0
```

The content of the docker image is as follows:

- Operation System: Ubuntu 18.04
- ROS melodic-desktop-full
- CUDA 10.2
- Gazebo 9.14.0
- Sapien 0.6
- A lot of simulated scenes for testing purposes, available at /root/ocrtoc_materials/scenes
- Object mesh models with texture information, available at /root/ocrtoc_materials/models



## File Structure of the OCRTOC software package

- **description**: The description files for hardware simulation. (**No modifiction allowed**)
- **gazebo_simulator**: Setups and scripts for the Gazebo simluator. (**No modifiction allowed**)
- **sapien_simluator**: Setups and scripts for the Sapien simluator. (**No modifiction allowed**)
- **ocrtoc_task**: Scripts for task execution and evaluation. (**No modifiction allowed**)
- **ocrtoc_solution**: Setups and scripts for building the solution. Sample code is given in ocrtoc_solution/scripts/commit_solution.py (**Do not change the file name!**). You can modify this file to develop your own solution. In addition you can also add any new software modules to this folder. Make sure that your commit_solution.py serves as the main function of your solution and it incorporates all the needed software modules to do the task. After your solution is uploaded to the competition platform, we will run your solution by ocrtoc_solution/launch/commit_solution.launch (**Do not change the file name!**).

## Interfaces
**ROS topic**

- /clock
- /tf*
- /arm_controller/*
- /gripper_controller/*
- /kinect/*
- /realsense/*
- /joint_group_position_controller/command
- /joint_states

**ROS service**

- /arm_controller/query_state
- /controller_manager/*

**Note**: In the simulation contest of OCRTOC, you are not allowed to use simulator topics to read the 6D pose of the objects in the simulated scenes. You must perform perception using the given object models and visual sensor readings. **Violation of this rule leads to disqualification**.


## Task definition

The target configuration of a task is defined as a list of 6D poses in a given coordinate system (frame) with corresponding object models and scale factors.

In general, we manage tasks using the ROS actionlib functionality. The script ocrtoc_solution/scripts/commit_solution.py will receive tasks in form of a callback function. Each time when a task is received, the corresponding callback function (execute_callback in commit_solution.py) is called.


**Task format**

```
string scene_id                   // the id of the loaded scene
string frame_id                   // the coordinate system, in which the 6D poses of the objects are defined
string[] object_list              // the list of object model names, by which you can get the model from the file system
float64[] scale_list              // the list of scale factors, using which the object model is scaled isotropically to generate the scene, typical values are: 0.5, 1, 2
geometry_msgs/Pose[] pose_list    // the list of 6D object poses (in x,y,z and quaternion) defined in frame_id
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
---
# result definition
string status
---
# feedback
string text
```
**Your solution**

After you obtained the task information, you need to implement the core functionality of your solution in the commit_solution.py script to finish the task (object organization). An intuitive way of doing this would be like: 1) recognize the objects in the scene so that you know where to place them; 2) plan the needed motion; 3) execute the planned motion. For some tasks, you may need some kind of active percepion method due to clutters and occlusions in the scene.

**Evaluation**

After you finished the task, you need to publish the actionlib result topic. The format of this result is a string. We do not parse the content of this string. Instead, it is only used to activate our callback function for evaluation. So you can write anything reasonable into this string, such as "done", "finished" and so on. If you do not publish the actionlib result at all, your solution will be terminated after a predefined timeout (e.g. 10 minutes), and then the evaluation will start automatically. We highly recommend you to publish the actionlib result topic, once your solution has finished the execution. This helps us compute the execution time of your solution. If two teams have the same performance, the team consuming less execution time will be ranked higher.


## Use the OCRTOC software package on your local machine

1. **Download the OCRTOC software package**
```bash
# For example, save at $HOME dir.
cd $HOME
git clone git@github.com:OCRTOC/OCRTOC_software_package.git
```

2. **Create a docker container**
```bash
sudo docker run -i -d --gpus all --name ocrtoc_container \
        -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $HOME/OCRTOC_software_package:/root/ocrtoc_ws/src \
        registry.cn-shanghai.aliyuncs.com/tcc-public/ocrtoc:release1.0
sudo xhost +local:`sudo docker inspect --format='{{ .Config.Hostname }}' ocrtoc_container`
```

3. **Launch a simulator and a testing scene**
```bash
# Load a testing scene, e.g. scene 1-1/input.world
sudo docker exec -it ocrtoc_container bash
cd /root/ocrtoc_ws
rm -rf build devel
catkin_make
# You can choose gazebo or sapien for testing
# Start gazebo
roslaunch ocrtoc_task bringup_simulator.launch simulator:=gazebo gui:=true scene:=1-1
# Or start sapien
roslaunch ocrtoc_task bringup_simulator.launch simulator:=sapien gui:=true scene:=1-1
```

4. **Launch your solution**
```bash
sudo docker exec -it ocrtoc_container bash
roslaunch ocrtoc_solution commit_solution.launch
```

5. **Trigger task and evaluation**
```bash
sudo docker exec -it ocrtoc_container bash
# For gazebo
roslaunch ocrtoc_task trigger_and_evaluation.launch simulator:=gazebo scene:=1-1
# For sapien
roslaunch ocrtoc_task trigger_and_evaluation.launch simulator:=sapien scene:=1-1
```

## Submit your solution for the simulation contest
You are required to submit your solution by uploading a docker image to the [competition platform](https://tianchi.aliyun.com/competition/entrance/531815/introduction).

**Please make sure**
- Install all the dependencies into the docker image you submit.
- Disable GUI in your solution, and delete unnecessary print.
- Remove your source code and leave the "catkin_make install" results in you docker image.
- Your solution can be fully launched by the ocrtoc_solution/launch/commit_solution.launch


**Submission example**
```
# 1. Install anything you need in your docker image, such as pytorch.
sudo docker exec -it ocrtoc_container bash
pip install torch torchvision

# 2. After your code is compiled, remove unnecessary files in the container.
cd /root/ocrtoc_ws
catkin_make install
rm -rf build devel

# 3. Verify whether your solution can be launched in the following way
# In terminal 1
sudo docker exec -it ocrtoc_container bash
source /root/catkin_ws/install.setup.bash
roslaunch ocrtoc_task bringup_simulator.launch simulator:=gazebo gui:=true scene:=1-1
# In terminal 2
sudo docker exec -it ocrtoc_container bash
source /root/catkin_ws/install.setup.bash
roslaunch ocrtoc_solution commit_solution.launch
# In terminal 3
sudo docker exec -it ocrtoc_container bash
source /root/catkin_ws/install.setup.bash
roslaunch ocrtoc_task trigger_and_evaluation.launch simulator:=gazebo scene:=1-1

# 4. Export a docker image by the docker container.
sudo docker commit ocrtoc_container your_submission_docker_image_name

# 5.Submit you docker image.
# We will provide a submission instruction on the competition platform later.
```

## Related documentations
[SAPIEN GUI Documentation](sapien_simulator/README.md)

## Known issues
- The reset time button in the GUI of the gazebo simulator should not be used. It will cause the robot to explode. Resetting can be done by ROS service /gazebo/reset_simulation instead.

- In general, the physics engine used in Sapien works more stable than that in Gazebo (open dynamics engine). If you encounter weird issues while using Gazebo, please use Sapien instead.

## Technical support
If you encounter technical issues regarding this software package, please contact us at info@ocrtoc.org
