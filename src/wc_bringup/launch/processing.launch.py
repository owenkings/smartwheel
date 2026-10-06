"""Dual processing only: never starts a physical sensor or navigation node."""
import json
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def configure(context):
    path = Path(LaunchConfiguration('session_config').perform(context)).resolve()
    config = json.loads(path.read_text())
    if config.get('sensor_mode') != 'dual':
        raise RuntimeError('Formal mapping requires dual sensors')
    if config['source_mode'] == 'real' and (
            config['calibration'].get('status') != 'VALIDATED' or
            config.get('quality_profile', {}).get('status') != 'VALIDATED'):
        raise RuntimeError('Live extrinsics/time/quality are not validated')
    common = {'frame_id':'rig_link', 'odom_frame_id':'odom', 'publish_tf':True,
              'wait_for_transform':0.2, 'use_sim_time':config.get('execution_mode')=='replay',
              'subscribe_scan':False, 'subscribe_scan_cloud':True,
              'qos':1,
              'scan_voxel_size':0.03, 'scan_normal_k':20,
              'Icp/PointToPlane':'true', 'Icp/PointToPlaneK':'20',
              'Icp/CorrespondenceRatio':'0.15', 'Icp/MaxCorrespondenceDistance':'0.25',
              'Icp/Iterations':'30', 'Icp/VoxelSize':'0.03',
              'Odom/Strategy':'0', 'Odom/ResetCountdown':'0',
              'OdomF2M/ScanMaxSize':'20000'}
    wheel_enabled = config.get('motion_prior', {}).get('enabled', False)
    tf_remaps = []
    if wheel_enabled:
        common.update(publish_tf=False, guess_frame_id='wheel_odom',
                      guess_min_translation=0.0, guess_min_rotation=0.0, guess_min_time=0.0,
                      deskewing=False, wait_imu_to_init=False)
        tf_remaps = [('/tf','/wc_mapping/icp_guess_tf'),
                     ('/tf_static','/wc_mapping/icp_guess_tf_static')]
    ack_required = LaunchConfiguration('graph_ack_required').perform(context)
    if ack_required not in ('true','false'):
        raise RuntimeError('graph_ack_required must be true or false')
    processes = [
        ExecuteProcess(cmd=[sys.executable,'-m','wc_fusion.ros_node','--session-config',str(path)] +
                       (['--graph-ack-required'] if ack_required == 'true' else []),output='screen'),
        Node(package='rtabmap_odom',executable='icp_odometry',name='wc_icp_odometry',
             parameters=[common],output='screen',
             remappings=[('scan_cloud','/wc_mapping/fusion/points'),('scan','/wc_mapping/unused_scan'),('odom','/wc_mapping/odom'),
                         ('odom_info','/wc_mapping/icp/odom_info'), *tf_remaps]),
    ]
    return processes + [RegisterEventHandler(OnProcessExit(target_action=process,
        on_exit=[EmitEvent(event=Shutdown(reason='A required fusion/ICP process exited'))])) for process in processes]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('session_config'),DeclareLaunchArgument('graph_ack_required',default_value='true'),OpaqueFunction(function=configure)])
