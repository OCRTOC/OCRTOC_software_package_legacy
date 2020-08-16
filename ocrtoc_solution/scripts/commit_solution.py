#! /usr/bin/env python

import actionlib
import rospy

import ocrtoc_task.msg
from control_msgs.msg import GripperCommandActionGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class CommitSolution(object):
    def __init__(self, name):
        # Init action.
        self.action_name = name
        self.action_server = actionlib.SimpleActionServer(
            self.action_name, ocrtoc_task.msg.CleanAction,
            execute_cb=self.execute_callback, auto_start=False)
        self.action_server.start()
        rospy.loginfo(self.action_name + " is running.")

        self.arm_cmd_pub = rospy.Publisher(
            rospy.resolve_name('arm_controller/command'),
            JointTrajectory, queue_size=10)
        self.gripper_cmd_pub = rospy.Publisher(
            rospy.resolve_name('gripper_controller/gripper_cmd/goal'),
            GripperCommandActionGoal, queue_size=10)

        # create messages that are used to publish feedback/result.
        self.feedback = ocrtoc_task.msg.CleanFeedback()
        self.result = ocrtoc_task.msg.CleanResult()

        # get models directory.
        materials_path = rospy.get_param('~materials_dir',
                                         '/root/ocrtoc_materials')
        self.models_dir = materials_path + '/models'
        rospy.loginfo("Models dir: " + self.models_dir)

    def execute_callback(self, goal):
        rospy.loginfo("Get clean task.")
        print(goal)
        ##### User code example starts #####
        # In the following, an example is provided on how to use the predefined software APIs
        # to get the target configuration, to control the robot, to set the actionlib result.
        # Example: load model.
        rospy.loginfo("Load models")
        for object_name in goal.object_list:
            object_model_dir = self.models_dir + '/' + object_name
            rospy.loginfo("Object model dir: " + object_model_dir)
        rospy.sleep(1.0)

        # Example: control ur5e by topic
        arm_cmd = JointTrajectory()
        arm_cmd.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', \
                               'elbow_joint', 'wrist_1_joint', \
                               'wrist_2_joint', 'wrist_3_joint']
        waypoint = JointTrajectoryPoint()
        waypoint.positions = [-0.68, -0.63, 0.69, -0.88, -0.53, -0.19]
        waypoint.time_from_start.secs = 2.0
        arm_cmd.points.append(waypoint)
        self.arm_cmd_pub.publish(arm_cmd)
        rospy.loginfo("Pub arm_cmd")
        rospy.sleep(1.0)

        # Example: control 2f85 by topic
        gripper_cmd = GripperCommandActionGoal()
        gripper_cmd.goal.command.position = 0.5
        gripper_cmd.goal.command.max_effort = 0.0
        self.gripper_cmd_pub.publish(gripper_cmd)
        rospy.loginfo("Pub gripper_cmd")
        rospy.sleep(1.0)

        # Example: set status "Aborted" and quit.
        if self.action_server.is_preempt_requested():
            self.result.status = "Aborted"
            self.action_server.set_aborted(self.result)
            return

        # Example: send feedback.
        self.feedback.text = "write_feedback_text_here"
        self.action_server.publish_feedback(self.feedback)
        rospy.loginfo("Pub feedback")
        rospy.sleep(1.0)

        # Example: set status "Finished" and quit.
        self.result.status = "Finished"
        rospy.loginfo("Done.")
        self.action_server.set_succeeded(self.result)
        ##### User code example ends #####


if __name__ == '__main__':
    rospy.init_node('commit_solution')
    commit_solution = CommitSolution('commit_solution')
    rospy.spin()
