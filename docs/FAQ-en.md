**Q1:** How many models will be presented during the Real Robot Stage?
- There 33 known objects stored [ocrtoc_materials_real_stage/models](https://github.com/OCRTOC/OCRTOC_software_package/tree/master/ocrtoc_materials_real_stage/models). Meanwhile 10-20 Unknown Objects will be presented during the contest.

**Q2:** What is a Puzzling Object?
- A Puzzling Object is present at the scene while absent from “target.world”. There won’t be any puzzling objects in the Level-1/Level-2 scene.

**Q3:** What is an Unknown Object?
- An Unknown Object was absent during the trial stage but will be presented during the contest stage. Participants are not allowed to collect the data of unknown objects but the related mesh file can be visited during the contest.

**Q4:** How many objects will be included in each task?
- In Level-1/Level-2, there are five target objects in each task. In Level-3/Level-4/Level-5, there are ten target objects in each task.

**Q5:** Will there be any officially provided data of objects?
- We are trying to generate a dataset that meets the most teams needs.

**Q6:** Will the camera postion/orientation in Real Robot Stage same with the Simulation Stage?
- Generally they are the same, but the exact position and orientation won’t be strictly the same values.

**Q7:** How to access the intrinsic/extrinsic parameters of cameras?
- To access the intrinsic parameters, subscribe the camera_info topic. To access the extrinsic parameters, read from [ocrtoc_task/config](https://github.com/OCRTOC/OCRTOC_software_package/tree/master/ocrtoc_task/config), or query by ROS tf API. It is not recommended to hard-code intrinsic/extrinsic values.

**Q8:** what are the differences between the real robot stage and the simulation stage?
- Showed in the [README](https://github.com/OCRTOC/OCRTOC_software_package/tree/master#information-about-the-real-robot-stage)

**Q9:** Which urdf file are used in the real robot stage?
- [ur5e_joint_limited_robot.urdf.xacro](https://github.com/OCRTOC/OCRTOC_software_package/blob/master/description/ur_e_description/urdf/ur5e_joint_limited_robot.urdf.xacro)

**Q10:** Will there be any officially provided moveit cfg?
- You can try the [ur5_e_robotiq_2f_85_moveit_planning_execution.launch](https://github.com/OCRTOC/OCRTOC_software_package/blob/master/ur5_e_robotiq_2f_85_moveit_config/launch/ur5_e_robotiq_2f_85_moveit_planning_execution.launch) or [all.launch](https://github.com/OCRTOC/OCRTOC_software_package/blob/master/ur5_e_robotiq_2f_85_moveit_config/launch/all.launch). Maybe you will encounter with some errors about robotiq tf, you can ignore them.

**Q11:** Can Participants modify the realsense/kinect/moveit config/launch files?
- Yes.