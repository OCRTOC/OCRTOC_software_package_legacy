#!/usr/bin/python3
import numpy as np
import sys
import xml.etree.ElementTree as ET
from transforms3d.euler import euler2quat
import os
import time

# The path below comes from a docker
sys.path.append("/workspace/sapien/build")
import rospkg
import rospy
import pysapien_ros1.core as sapien
import pysapien_ros1.ros1 as sr

RENDER_HZ = 8


def load_sapien_sdf(sdf_file, scene, table_height):
    model_path = os.getenv('SAPIEN_MODEL_PATH')
    assert model_path, 'SAPIEN_MODEL_PATH environment variable is required'
    if model_path[-1] != '/':
        model_path += '/'
    sdf = ET.parse(sdf_file).getroot()
    world = sdf.find('world')
    actors = []
    for l in world.findall('light'):
        assert l.attrib['type'] == 'point'
        color = [float(x) / 3.14 for x in l.find('diffuse').text.split()]
        position = np.array([float(x) for x in l.find('pose').text.split()][:3])
        position[2] += table_height
        scene.add_point_light(position, color)
    for sdf_model in world.findall('model'):
        builder = scene.create_actor_builder()
        sdf_link = sdf_model.find('link')
        sdf_pose = sdf_model.find('pose')
        sdf_inertial = sdf_link.find('inertial')
        assert sdf_inertial is not None
        cs = sdf_link.findall('collision')
        vs = sdf_link.findall('visual')
        for col in cs:
            sdf_geom = col.find('geometry')
            sdf_mesh = sdf_geom.find('mesh')
            sdf_uri = sdf_mesh.find('uri')
            sdf_scale = sdf_mesh.find('scale')
            assert sdf_uri is not None and sdf_scale is not None
            filename = sdf_uri.text.replace('model://', model_path)
            scale = [float(x) for x in sdf_scale.text.strip().split()]
            assert len(scale) == 3
            assert os.path.isfile(filename), filename
            friction = float(col.find('surface').find('friction').find('ode').find('mu').text)
            assert friction == 0.5  # will all be 0.5
            builder.add_multiple_convex_shapes_from_file(filename, scale=scale)
        for v in vs:
            sdf_geom = v.find('geometry')
            sdf_mesh = sdf_geom.find('mesh')
            sdf_uri = sdf_mesh.find('uri')
            sdf_scale = sdf_mesh.find('scale')
            assert sdf_uri is not None and sdf_scale is not None
            filename = sdf_uri.text.replace('model://', model_path)
            scale = [float(x) for x in sdf_scale.text.strip().split()]
            assert len(scale) == 3
            assert os.path.isfile(filename), filename
            builder.add_visual_from_file(filename, scale=scale)
        sdf_mass = sdf_inertial.find('mass')
        sdf_pose = sdf_inertial.find('pose')
        sdf_inertia = sdf_inertial.find('inertia')
        assert sdf_mass is not None and sdf_pose is not None and sdf_inertia is not None
        mass = float(sdf_mass.text)
        xyzrpy = [float(x) for x in sdf_pose.text.strip().split()]
        assert len(xyzrpy) == 6
        ixx = float(sdf_inertia.find('ixx').text)
        iyy = float(sdf_inertia.find('iyy').text)
        izz = float(sdf_inertia.find('izz').text)
        ixy = float(sdf_inertia.find('ixy').text)
        ixz = float(sdf_inertia.find('ixz').text)
        iyz = float(sdf_inertia.find('iyz').text)
        assert ixy == ixz == iyz == 0
        builder.set_mass_and_inertia(mass, sapien.Pose(xyzrpy[:3], euler2quat(*xyzrpy[3:])), [ixx, ixy, ixz])
        model_pose = sdf_model.find('pose')
        model = builder.build(name=sdf_model.attrib['name'])
        xyzrpy = np.array([float(x) for x in model_pose.text.strip().split()])
        xyzrpy[2] += table_height
        model.set_pose(sapien.Pose(xyzrpy[:3], euler2quat(*xyzrpy[3:])))
        model.set_velocity([0, 0, 0])
        model.set_damping(1, 1)
        actors.append(model)
    return actors


def setup_table(scene: sapien.Scene, height, table_physical_material):
    table_size = np.array([1, 0.8, 0.01]) / 2
    table_pose = np.array([0, 0, height - 0.01])
    table_vis_material = sapien.PxrMaterial()
    table_vis_material.roughness = 0.025
    table_vis_material.specular = 0.95
    table_vis_material.metallic = 0.6
    rgbd = np.array([171, 171, 171, 255])
    table_vis_material.set_base_color(rgbd / 255)
    builder = scene.create_actor_builder()
    builder.add_box_visual_complex(sapien.Pose(table_pose), table_size, table_vis_material)
    builder.add_box_shape(sapien.Pose(table_pose), table_size, table_physical_material)
    table = builder.build_static("table")
    table.set_pose(sapien.Pose([0, 0, 0], [-0.7071, 0, 0, 0.7071]))

    table_leg_position1 = [0.45, 0.35, height / 2]
    table_leg_position2 = [-0.45, -0.35, height / 2]
    table_leg_position3 = [-0.45, 0.35, height / 2]
    table_leg_position4 = [0.45, -0.35, height / 2]
    table_leg_size = np.array([0.025, 0.025, height / 2 - 0.01])
    builder = scene.create_actor_builder()
    builder.add_box_visual_complex(sapien.Pose(table_leg_position1), table_leg_size)
    builder.add_box_visual_complex(sapien.Pose(table_leg_position2), table_leg_size)
    builder.add_box_visual_complex(sapien.Pose(table_leg_position3), table_leg_size)
    builder.add_box_visual_complex(sapien.Pose(table_leg_position4), table_leg_size)
    legs = builder.build_static("table_leg")
    legs.set_pose(table.get_pose())

    return [table, legs]


def main():
    # Parse ROS path and args
    materials_path = rospy.get_param('~materials_dir',
                                     '/root/ocrtoc_materials')
    args = parse_arg()
    print(args)
    if args.paused and not args.gui:
        raise RuntimeError(
            "Argument paused is only useful when GUI is activated. It is only for debug purpose. "
            "Your program will directly end when using paused:=true with gui:=false")

    current_path = rospkg.RosPack().get_path('sapien_simulator')
    engine = sapien.Engine()
    optifuser_config = sapien.OptifuserConfig()
    optifuser_config.use_shadow = False
    renderer = sapien.OptifuserRenderer(glsl_dir=os.path.join(current_path, "./glsl_shader/130"),
                                        glsl_version="130",
                                        config=optifuser_config)
    engine.set_renderer(renderer)
    controller = sapien.OptifuserController(renderer)

    # Load scene and ground
    scene_config = sapien.SceneConfig()
    scene_config.solver_iterations = 25
    scene_config.solver_velocity_iterations = 2
    scene_config.enable_pcm = False
    scene_config.default_restitution = 0
    scene_config.default_dynamic_friction = 0.5
    scene_config.default_static_friction = 0.5
    scene = engine.create_scene(scene_config)
    scene.set_timestep(1 / 250)
    ground_material = sapien.PxrMaterial()
    ground_color = np.array([202, 164, 114, 256]) / 256
    ground_material.set_base_color(ground_color)
    ground_material.specular = 0.5
    scene.add_ground(-0.8, render_material=ground_material)

    if args.gui:
        controller.set_current_scene(scene)
        controller.set_camera_position(2.5, 0, 3)
        controller.set_camera_rotation(3.14, -0.7)
        controller.show_window()

    # Load table
    table_height = 0.0

    # Load sdf
    os.environ.update({
        "SAPIEN_MODEL_PATH": os.path.join(materials_path, "models")})
    sdf_objects = load_sapien_sdf(args.world_name, scene, table_height)

    # scene.set_shadow_light([0, -1, -1], [1, 1, 1])
    scene.set_ambient_light((0.5, 0.5, 0.5))

    sr.init_spd_logger()
    scene_manager = sr.SceneManager(scene, "")
    loader = scene_manager.create_robot_loader()
    loader.fix_root_link = True
    gripper_material = engine.create_physical_material(1.2, 0.8, 0.01)
    urdf_config = {
        "link": {
            "robotiq_2f_85_left_pad": {"material": gripper_material, "patch_radius": 0.5, "min_patch_radius": 0.01},
            "robotiq_2f_85_right_pad": {"material": gripper_material, "patch_radius": 0.5,
                                        "min_patch_radius": 0.01}}}

    # Load robot
    robot, manager = loader.load_from_parameter_server("", urdf_config, 125)
    init_qpos = np.array([-1.57, -1.57, 1.57, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    robot.set_qpos(init_qpos)
    robot.set_drive_target(init_qpos)
    manager.set_drive_property(3000, 500, 1000, [0, 1, 2, 3, 4, 5])
    manager.set_drive_property(200, 50, 300, [6, 7])
    manager.set_drive_property(100, 40, 300, [8, 9, 10, 11])
    scene_manager.start_all_ros_camera(30)
    scene_manager.start_get_model_service("/sapien/get_model_state", sdf_objects)
    scene.step()

    # Start
    start_time = time.time()
    scene_manager.start()
    step = 0
    timestep = scene.get_timestep()
    next_step_time = time.time() + timestep
    mimic_joints = robot.get_active_joints()[20:24]
    if args.gui:
        if args.paused:
            while not controller.should_quit:
                scene.update_render()
                controller.render()

            while True:
                step_and_render(manager, scene, controller, step, next_step_time)
                mimic_joint(robot, mimic_joints)
                next_step_time += timestep
                step += 1
        else:
            while not controller.should_quit:
                step_and_render(manager, scene, controller, step, next_step_time)
                mimic_joint(robot, mimic_joints)
                next_step_time += timestep
                step += 1
    else:
        try:
            while True:
                step_only(manager, scene, next_step_time)
                mimic_joint(robot, mimic_joints)
                next_step_time += timestep
                step += 1
        except KeyboardInterrupt:
            print("Simulation stopped by user")


scene = None


def step_and_render(manager, scene, controller, step, next_step_time):
    manager.balance_passive_force()
    now = time.time()
    while now < next_step_time:
        time.sleep(1e-4)
        now = time.time()
    scene.step()
    scene.update_render()
    if step % RENDER_HZ == 0:
        controller.render()


def step_only(manager, scene, next_step_time):
    manager.balance_passive_force()
    now = time.time()
    while now < next_step_time:
        time.sleep(1e-4)
        now = time.time()
    scene.step()
    scene.update_render()


def mimic_joint(robot, mimic_joints):
    left_target = robot.get_qpos()[7] * 17.86
    right_target = robot.get_qpos()[6] * 17.86
    mimic_joints[0].set_drive_target(right_target)
    mimic_joints[1].set_drive_target(right_target)
    mimic_joints[2].set_drive_target(left_target)
    mimic_joints[3].set_drive_target(left_target)


def parse_arg():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", action="store_true", help="show gui for visualization")
    parser.add_argument("--paused", action="store_true", help="start simulator in a paused mode")
    parser.add_argument("--world_name", type=str, help="scene name for loading")
    print(sys.argv[1:-2])
    return parser.parse_args(sys.argv[1:-2])


if __name__ == '__main__':
    import sys

    sr.ros_init("iros_pipeline", sys.argv)

    main()
