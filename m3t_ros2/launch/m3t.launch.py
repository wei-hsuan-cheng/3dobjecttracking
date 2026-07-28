# SPDX-License-Identifier: MIT
# Two-node M3T tracking:
#   m3t_image_publisher_node  — reads the sequence, publishes color/depth (+info) + GT (the "camera")
#   m3t_tracker_node          — subscribes to those topics and tracks
# Replace node 1 with a real camera driver by matching the topic names.
#
#   ros2 launch m3t_ros2 m3t.launch.py object:=cylinder modalities:=region,depth rviz:=true
#   ros2 launch m3t_ros2 m3t.launch.py object:=mustard modalities:=region,depth,texture track_rate:=0

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

M3T_ROOT_DEFAULT = "/root/ocs2_ros2_ws/src/3dobjecttracking/M3T"

OBJECTS = {
    "triangle": ("triangle", "data/_body/triangle.obj", False),
    "box": ("box", "data/_body/box.obj", False),
    "cylinder": ("cylinder", "data/_body/cylinder.obj", False),
    "mustard": ("ycb_mustard_bottle",
                "temp/ycb_mustard/006_mustard_bottle/google_16k/textured.obj", True),
}


def launch_setup(context, *args, **kwargs):
    m3t = LaunchConfiguration("m3t_root").perform(context)
    obj = LaunchConfiguration("object").perform(context)
    modalities = LaunchConfiguration("modalities").perform(context)
    seq = LaunchConfiguration("sequence_dir").perform(context) or \
        os.path.join(m3t, "temp", "rgbd_" + obj)
    stem, mesh_rel, embedded = OBJECTS.get(obj, (obj, "data/_body/%s.obj" % obj, False))
    body_metafile = os.path.join(m3t, "data", "_body", stem + ".yaml")
    mesh_resource = "file://" + os.path.join(m3t, mesh_rel)

    topics = {
        "color_topic": LaunchConfiguration("color_topic").perform(context),
        "depth_topic": LaunchConfiguration("depth_topic").perform(context),
        "color_info_topic": LaunchConfiguration("color_info_topic").perform(context),
        "depth_info_topic": LaunchConfiguration("depth_info_topic").perform(context),
    }

    source = Node(
        package="m3t_ros2", executable="m3t_image_publisher_node",
        name="m3t_image_publisher", output="screen",
        parameters=[{
            "sequence_dir": seq, "body_metafile": body_metafile,
            "publish_rate": float(LaunchConfiguration("source_rate").perform(context)),
            "loop": True, "world_frame": "camera", "publish_gt": True,
            "mesh_resource": mesh_resource, "mesh_scale": 1.0,
            "mesh_use_embedded_materials": embedded, **topics,
        }],
    )
    tracker = Node(
        package="m3t_ros2", executable="m3t_tracker_node",
        name="m3t_tracker_node", output="screen",
        parameters=[{
            "body_metafile": body_metafile, "modalities": modalities,
            "temp_dir": seq, "depth_scale": 0.001,
            "track_rate": float(LaunchConfiguration("track_rate").perform(context)),
            "publish_rate": float(LaunchConfiguration("publish_rate").perform(context)),
            "log_period": float(LaunchConfiguration("log_period").perform(context)),
            "world_frame": "camera", "mesh_resource": mesh_resource, "mesh_scale": 1.0,
            "mesh_use_embedded_materials": embedded,
            "use_gt_initial_pose": LaunchConfiguration("use_gt_initial_pose").perform(context) == "true",
            "gt_frame": "object_gt",
            "publish_overlay": True, "publish_keypoints": True, **topics,
        }],
    )
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        arguments=["-d", os.path.join(get_package_share_directory("m3t_ros2"), "rviz", "m3t.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    return [source, tracker, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("m3t_root", default_value=M3T_ROOT_DEFAULT),
        DeclareLaunchArgument("object", default_value="cylinder"),
        DeclareLaunchArgument("modalities", default_value="region,depth"),
        DeclareLaunchArgument("sequence_dir", default_value=""),
        DeclareLaunchArgument("source_rate", default_value="30.0",
                              description="node 1 image publish rate (camera fps)"),
        DeclareLaunchArgument("track_rate", default_value="0.0",
                              description="node 2 solve-loop Hz; 0 = as fast as possible"),
        DeclareLaunchArgument("publish_rate", default_value="30.0"),
        DeclareLaunchArgument("log_period", default_value="2.0"),
        DeclareLaunchArgument("use_gt_initial_pose", default_value="true",
                              description="seed init pose from GT TF (true) or static yaml (false, real-camera)"),
        DeclareLaunchArgument("color_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_raw"),
        DeclareLaunchArgument("color_info_topic", default_value="/camera/color/camera_info"),
        DeclareLaunchArgument("depth_info_topic", default_value="/camera/depth/camera_info"),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
