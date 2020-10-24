**Q1:** 真机阶段有多少个模型？
- 有33个已知物体存放在[ocrtoc_materials_real_stage/models](https://github.com/OCRTOC/OCRTOC_software_package/tree/master/ocrtoc_materials_real_stage/models)，还有10-20个未知物体用于决赛阶段。

**Q2:** 什么是干扰物体？
- 干扰物体是场景中存在，但是并不在target.world里存在的物体。level-1/level-2的场景里没有干扰物体。

**Q3:** 什么是未知物体？
- 未知物体是在trial阶段不提供，但是contest阶段会用到的物体。参赛者无法采集未知物体的数据，但是可以在程序运行时访问到mesh文件。

**Q4:** 每个任务会包含多少物体？
- level-1/level-2每个任务5个target物体，level-3/level-4/level-5每个任务10个target物体。

**Q5:** 官方是否会提供物体的数据？
- 我们正在尝试生成一份满足大部分队伍使用的数据。

**Q6:** 真机阶段相机位置与仿真阶段一致吗？
- 大概位置一致，但是具体的位置和朝向无法完全一致。

**Q7:** 如何获取相机的内参与外参？
- 内参可以通过subscribe camera_info的topic，外参可以读取[ocrtoc_task/config](https://github.com/OCRTOC/OCRTOC_software_package/tree/master/ocrtoc_task/config)的标定结果，也可以通过ROS tf API查询。而且不建议在代码里hard-code内参和外参的数值。

**Q8:** 真机阶段与仿真阶段接口有哪些不同？
- 相机: 真机阶段提供了更丰富的image topic，你可以能需要修改topic name、frame_id、encoding。
- 手臂：真机阶段默认不启动arm_controller，你需要启动自己想要的controller，如果不启动，ur默认的controller name是pos_traj_controller和scaled_pos_traj_controller。如果你使用moveit控制，无需更改controller接口。
- 手臂：每个关节的位置限制[-pi, pi], 速度上限45deg/s。
- 夹爪：“max_effort”需要在30N-100N之间。

**Q9:** 真机阶段的urdf文件是哪个？
- [ur5e_joint_limited_robot.urdf.xacro](https://github.com/OCRTOC/OCRTOC_software_package/blob/master/description/ur_e_description/urdf/ur5e_joint_limited_robot.urdf.xacro)

**Q10:** 代码库里的moveit pkg是否可用？
- 可以用。可以启动[ur5_e_robotiq_2f_85_moveit_planning_execution.launch](https://github.com/OCRTOC/OCRTOC_software_package/blob/master/ur5_e_robotiq_2f_85_moveit_config/launch/ur5_e_robotiq_2f_85_moveit_planning_execution.launch)或者[all.launch](https://github.com/OCRTOC/OCRTOC_software_package/blob/master/ur5_e_robotiq_2f_85_moveit_config/launch/all.launch)。运行时可能会提示robotiq tf问题，但是不影响使用。

**Q11:** 是否可以更改realsense/kinect启动launch，是否可以更改moveit cfg？
- 可以更改。

