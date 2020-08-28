#! /usr/bin/env python

import csv
import numpy as np
import sys
import xml.etree.ElementTree as ET

import actionlib
import rospy
import tf

from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from geometry_msgs.msg import Pose
import ocrtoc_task.msg
import task_evaluation


def paraphrase_world(world_file, goal, alias_list):
    root = ET.parse(world_file).getroot()
    for child in root.findall('world/model'):
        alias = child.attrib['name']
        if alias == "ground_plane" or alias == "table":
            continue
        alias_list.append(alias)
        for uri in child.findall('link/visual/geometry/mesh/uri'):
            model_path = uri.text
            model_name = (model_path.split('://')[-1]).split('/')[0]
            goal.object_list.append(model_name)
        for scale in child.findall('link/visual/geometry/mesh/scale'):
            goal.scale_list.append(float(scale.text.split(' ')[0]))
        for pose in child.findall('pose'):
            model_pose = Pose()
            var_list = map(float, pose.text.split(' '))
            model_pose.position.x = var_list[0]
            model_pose.position.y = var_list[1]
            model_pose.position.z = var_list[2]
            quaternion = tf.transformations.quaternion_from_euler(
                var_list[3], var_list[4], var_list[5])
            model_pose.orientation.x = quaternion[0]
            model_pose.orientation.y = quaternion[1]
            model_pose.orientation.z = quaternion[2]
            model_pose.orientation.w = quaternion[3]
            goal.pose_list.append(model_pose)


def load_objects_db(db_file):
    csv_file = open(db_file)
    objects_db = csv.DictReader(csv_file)
    objects_dict = {}
    for item in objects_db:
        for key in item.keys():
            try:
                item[key] = float(item[key])
            except:
                pass
        objects_dict[item['object']] = item
    return objects_dict


if __name__ == '__main__':
    rospy.init_node('task_manager')

    task_manager = actionlib.SimpleActionClient(
        '/commit_solution', ocrtoc_task.msg.CleanAction)
    rospy.wait_for_service('get_model_state')
    get_model_state = rospy.ServiceProxy('get_model_state', GetModelState)

    # get scenes directory
    materials_path = rospy.get_param('~materials_dir',
                                     '/root/ocrtoc_materials')
    scenes_dir = materials_path + '/scenes'
    rospy.loginfo("scenes dir: " + scenes_dir)
    try:
        task_name = rospy.get_param('~scene')
    except:
        print("Usage:")
        print("      rosrun ocrtoc_task trigger_and_evaluation.py _scene:=task_id")
        sys.exit()
    rospy.loginfo("task name: " + task_name)
    task_path = scenes_dir + "/" + task_name + "/target.world"
    rospy.loginfo("task path: " + task_path)

    # Waits until the user solution has started up and started
    task_manager.wait_for_server()

    alias_list = []

    # 1.Creates a actionlib goal from scene task configuration.
    goal = ocrtoc_task.msg.CleanGoal()
    goal.scene_id = task_name
    goal.frame_id = 'table'
    paraphrase_world(task_path, goal, alias_list)
    print(goal)

    # 2.Sends the goal to the user solution.
    start_time = rospy.get_time()
    while start_time < 0.001:
        start_time = rospy.get_time()
    rospy.loginfo("Start: " + str(start_time))
    task_manager.send_goal(goal)

    # 3.Waits for the user solution to finish performing the action.
    finished_before_timeout = False
    threshold = 600.0  # seconds.
    finished_before_timeout = \
        task_manager.wait_for_result(rospy.Duration(threshold))
    if finished_before_timeout:
        rospy.loginfo("Action done.")
    else:
        rospy.loginfo("Action did not finish before the time out. Canceling")
        task_manager.cancel_all_goals()
        rospy.sleep(2.0)
    rospy.loginfo(task_manager.get_result())
    end_time = rospy.get_time()
    while end_time < 0.001:
        end_time = rospy.get_time()
    rospy.loginfo("End: " + str(end_time))

    # 4.Call evaluate function.
    rospy.loginfo("Get result and evaluate it.")
    target_pose_list = goal.pose_list
    result_pose_list = []
    request_msg = GetModelStateRequest()
    for model in alias_list:
        request_msg.model_name = model
        response_msg = get_model_state(request_msg)
        result_pose_list.append(response_msg.pose)
    db_file = materials_path + '/objects.csv'
    objects_db = load_objects_db(db_file)
    max_error_threshold = 5.

    objects_pose_error = []
    for index in range(len(alias_list)):
        print '-' * 80
        print alias_list[index]
        print goal.object_list[index]
        print goal.scale_list[index]
        print result_pose_list[index]
        print target_pose_list[index]

        # estimated
        result_p = result_pose_list[index].position
        result_q = result_pose_list[index].orientation
        R_est = np.array(tf.transformations.quaternion_matrix(
                         [result_q.x, result_q.y, result_q.z, result_q.w])
                         [0:3, 0:3])
        t_est = np.array([result_p.x, result_p.y, result_p.z])

        # gt
        target_p = target_pose_list[index].position
        target_q = target_pose_list[index].orientation
        R_gt = np.array(tf.transformations.quaternion_matrix(
                        [target_q.x, target_q.y, target_q.z, target_q.w])
                        [0:3, 0:3])
        t_gt = np.array([target_p.x, target_p.y, target_p.z])

        # scaled model info
        model_info = objects_db[goal.object_list[index]].copy()
        for key in model_info.keys():
            if type(model_info[key]) == float:
                model_info[key] = goal.scale_list[index] * model_info[key]

        objects_pose_error.append(task_evaluation.object_pose_error(
            R_est, t_est, R_gt, t_gt, model_info, max_error_threshold))

    scene_pose_error = \
        task_evaluation.scene_pose_error(np.array(objects_pose_error))
    simulator_name = rospy.get_namespace().replace('/', '')
    time_cost = end_time - start_time
    rospy.loginfo("Simulator: " + simulator_name)
    rospy.loginfo("Task name: " + task_name)
    rospy.loginfo("Scene pose error: " + str(scene_pose_error) + ' meters')
    rospy.loginfo("Time cost: " + str(time_cost) + ' seconds')
    rospy.loginfo("Done.")
