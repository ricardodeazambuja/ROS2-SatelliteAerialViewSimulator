import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, Shutdown, SetEnvironmentVariable, RegisterEventHandler, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)



def generate_launch_description():

   lat = LaunchConfiguration('lat')
   lat_arg = DeclareLaunchArgument(
      'lat',
      default_value='48.858327718853104'
   )

   lon = LaunchConfiguration('lon')
   lon_arg = DeclareLaunchArgument(
      'lon',
      default_value='2.294309636169546'
   )

   z_init = LaunchConfiguration('z_init')
   z_init_arg = DeclareLaunchArgument(
      'z_init',
      default_value='100.0'
   )

   rgb_topic = LaunchConfiguration('rgb_topic')
   rgb_topic_arg = DeclareLaunchArgument(
      'rgb_topic',
      default_value='/carla/flying_sensor/rgb_down/image'
   )

   depth_topic = LaunchConfiguration('depth_topic')
   depth_topic_arg = DeclareLaunchArgument(
      'depth_topic',
      default_value='/carla/flying_sensor/depth_down/image'
   )

   twist_topic = LaunchConfiguration('twist_topic')
   twist_topic_arg = DeclareLaunchArgument(
      'twist_topic',
      default_value='/quadctrl/flying_sensor/ctrl_twist_sp'
   )

   baseurl = LaunchConfiguration('baseurl')
   baseurl_arg = DeclareLaunchArgument(
      'baseurl',
      default_value='https://wxs.ign.fr/choisirgeoportail/geoportail/wmts?REQUEST=GetTile&SERVICE=WMTS&VERSION=1.0.0&STYLE=normal&TILEMATRIXSET=PM&FORMAT=image/jpeg&LAYER=ORTHOIMAGERY.ORTHOPHOTOS&TILEMATRIX={z}&TILEROW={y}&TILECOL={x}'
   )

   aerialimages = Node(
         package='ros2_satellite_aerial_view_simulator',
         executable='aerialimages_publisher',
         name='aerialimages_module',
         # emulate_tty=True,
         parameters=[
            {'lat':lat}, 
            {'lon':lon}, 
            {'z_init':z_init},
            {'rgb_topic':rgb_topic},
            {'depth_topic':depth_topic},
            {'twist_topic':twist_topic},
            {'baseurl':baseurl},
            ]
      )

   return LaunchDescription([
      SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
      lat_arg,
      lon_arg,
      z_init_arg,
      rgb_topic_arg,
      depth_topic_arg,
      twist_topic_arg,
      baseurl_arg,
      aerialimages
   ])