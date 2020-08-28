## [V1.1] release date: 2020-08-28
**Changes in v1.1**
- Added 20 object models and 100 trial scenes (available at ocrtoc_materials_patch). You can use the script ocrtoc_materials_patch/patch.sh to copy the new models and scenes into your docker image. The new object models will be re-used in the simulation contest and in the real robot stage.
- Added the task evaluation module (search "Trigger task and evaluation" in readme). Your solution will be evaluated by the same software module in the simulation contest.
- Changed ambient light strength from 0.3 to 0.7 in all trial scenes.
- Fixed camera_info of the Kinect camera for the Sapien simulator.
- Fixed gripper issues for the Sapien simulator. Now the robotiq 2f-85 gripper model uses an explicitly-built prismatic joint. Before the fix, the gripper model relied on the kinematics constraints of three revolute joints, which sometimes failed to maintain the mimic joint status due to certain external forces.

**Known issues in v1.1**
- The gripper model in the Gazebo simulator is not fixed. If you experience any weird problems while using Gazebo, please use Sapien instead.


