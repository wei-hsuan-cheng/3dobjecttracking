# SPDX-License-Identifier: MIT
# Launch the M3T tracker on a synthetic RGB-D sequence.
#
# Examples:
#   ros2 launch m3t_ros2 m3t.launch.py object:=cylinder modalities:=region,depth
#   ros2 launch m3t_ros2 m3t.launch.py object:=mustard modalities:=region,depth,texture rviz:=true

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

M3T_ROOT_DEFAULT = "/root/ocs2_ros2_ws/src/3dobjecttracking/M3T"

# object -> (body yaml stem, mesh relative to M3T root, embedded materials)
OBJECTS = {
    "triangle": ("triangle", "data/_body/triangle.obj", False),
    "box": ("box", "data/_body/box.obj", False),
    "cylinder": ("cylinder", "data/_body/cylinder.obj", False),
    "mustard": ("ycb_mustard_bottle",
                "temp/ycb_mustard/006_mustard_bottle/google_16k/textured.obj",
                True),
}


def launch_setup(context, *args, **kwargs):
    m3t = LaunchConfiguration("m3t_root").perform(context)
    obj = LaunchConfiguration("object").perform(context)
    modalities = LaunchConfiguration("modalities").perform(context)
    track_rate = float(LaunchConfiguration("track_rate").perform(context))
    publish_rate = float(LaunchConfiguration("publish_rate").perform(context))
    log_period = float(LaunchConfiguration("log_period").perform(context))
    seq = LaunchConfiguration("sequence_dir").perform(context) or \
        os.path.join(m3t, "temp", "rgbd_" + obj)

    stem, mesh_rel, embedded = OBJECTS.get(obj, (obj, "data/_body/%s.obj" % obj, False))
    body_metafile = os.path.join(m3t, "data", "_body", stem + ".yaml")
    mesh_resource = "file://" + os.path.join(m3t, mesh_rel)

    tracker = Node(
        package="m3t_ros2", executable="m3t_tracker_node",
        name="m3t_tracker_node", output="screen",
        parameters=[{
            "sequence_dir": seq,
            "body_metafile": body_metafile,
            "modalities": modalities,
            "mesh_resource": mesh_resource,
            "mesh_scale": 1.0,
            "mesh_use_embedded_materials": embedded,
            "track_rate": track_rate,
            "publish_rate": publish_rate,
            "log_period": log_period,
            "world_frame": "camera",
            "publish_gt": True,
            "publish_keypoints": True,
            "publish_overlay": True,
        }],
    )
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        arguments=["-d", os.path.join(
            get_share(), "rviz", "m3t.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    return [tracker, rviz]


def get_share():
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory("m3t_ros2")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("m3t_root", default_value=M3T_ROOT_DEFAULT),
        DeclareLaunchArgument("object", default_value="cylinder"),
        DeclareLaunchArgument("modalities", default_value="region,depth"),
        DeclareLaunchArgument("sequence_dir", default_value=""),
        DeclareLaunchArgument("track_rate", default_value="0.0",
                              description="solve loop Hz; 0 = as fast as possible"),
        DeclareLaunchArgument("publish_rate", default_value="30.0"),
        DeclareLaunchArgument("log_period", default_value="2.0"),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
