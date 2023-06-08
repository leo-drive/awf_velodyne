import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration


def velodyne_driver_params(device_ip=None,
                           frame_id="velodyne",
                           model="VLP16",
                           pcap=None,
                           port=2368,
                           npackets=None,
                           read_fast=False,
                           read_once=False,
                           repeat_delay=0.0,
                           rpm=600.0,
                           scan_phase=0.0,
                           sensor_timestamp=False) -> list:
    params = [
        {'frame_id': frame_id},
        {'model': model},
        {'port': port},
        {'read_fast': read_fast},
        {'read_once': read_once},
        {'repeat_delay': repeat_delay},
        {'rpm': rpm},
        {'scan_phase': scan_phase},
        {'sensor_timestamp': sensor_timestamp},
    ]

    if device_ip is not None:
        params.append({'device_ip': device_ip})

    if pcap is not None:
        params.append({'pcap': pcap})

    if npackets is not None:
        params.append({'npackets': npackets})

    return params


def velodyne_cloud_params(
        calibration=os.path.join(get_package_share_directory("velodyne_pointcloud"), "params/VLP16db.yaml"),
        max_range=130.0, min_range=0.4,
        num_points_threshold=300,
        invalid_intensity=None,
        scan_phase=300.0,
        scan_period=100,
        invalid_rings=None,
        invalid_distance=0.0) -> list:
    if invalid_intensity is None:
        invalid_intensity = [0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    params = [
        {'calibration': calibration},
        {'max_range': max_range},
        {'min_range': min_range},
        {'num_points_threshold': num_points_threshold},
        {'invalid_intensity': invalid_intensity},
        {'scan_phase': scan_phase},
        {'scan_period': scan_period},
    ]

    if invalid_rings is not None:
        params.append({'invalid_rings': invalid_rings})
        params.append({'invalid_distance': invalid_distance})

    return params


def create_node(node_descs: list, namespace: str, driver_params: list, cloud_params: list):
    node_descs.append(ComposableNode(
        package='velodyne_driver',
        plugin='velodyne_driver::VelodyneDriver',
        name=f"velodyne_driver_node_{namespace}",
        parameters=driver_params,
        remappings=[
            ("velodyne_packets", f"/{namespace}/velodyne_packets"),
        ]
    ))
    node_descs.append(ComposableNode(
        package='velodyne_pointcloud',
        plugin='velodyne_pointcloud::Convert',
        name=f"velodyne_cloud_node_{namespace}",
        parameters=cloud_params,
        remappings=[
            ("velodyne_points", f"/{namespace}/velodyne_points"),
            ("velodyne_points_ex", f"/{namespace}/velodyne_points_ex"),
            ("velodyne_model_marker", f"/{namespace}/velodyne_model_marker"),
        ]
    ))


def generate_launch_description():
    node_descs_left = []
    create_node(node_descs_left, "middle_left", velodyne_driver_params(port=2370),
                velodyne_cloud_params(invalid_rings=[0, 1, 2], invalid_distance=3.0))

    container_left = ComposableNodeContainer(
        name='velodyne_container',
        namespace='middle_left',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=node_descs_left,
        output='screen',
    )

    # node_descs_right = []
    # create_node(node_descs_right, "middle_right", velodyne_driver_params(port=2372),
    #             velodyne_cloud_params(invalid_rings=[0, 1, 2], invalid_distance=3.0))
    #
    # container_right = ComposableNodeContainer(
    #     name='velodyne_container',
    #     namespace='middle_right',
    #     package='rclcpp_components',
    #     executable='component_container',
    #     composable_node_descriptions=node_descs_right,
    #     output='screen',
    # )

    # return LaunchDescription([container_left, container_right])
    return LaunchDescription([container_left])
