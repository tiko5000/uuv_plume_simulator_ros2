from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    # Define variables with the default values
    # uuv_name = "uuv_name"
    # latitude_ref = "0"
    # longitude_ref = "0"
    # odom_topic = "pose_gt"
    # gps_topic = "gps"
    # gamma = "gamma"
    # gain = "gain"
    # radius = "radius"
    # update_rate = "1"
    # use_geo_coordinates = "use_geo_coordinates"
    # reference_salinity_value = "35.0"
    # salinity_unit = "ppt"
    # sensor_frame_id = "wamv/wamv/base_link"
    # publish_salinity = "true"
    # use_odom = "false"
    # use_gps = "false"

    uuv_name = LaunchConfiguration('uuv_name')
    uuv_name_launch_arg = DeclareLaunchArgument('uuv_name', default_value='uuv_name')

    latitude_ref = LaunchConfiguration('latitude_ref')
    latitude_ref_launch_arg = DeclareLaunchArgument('latitude_ref', default_value='0')

    longitude_ref = LaunchConfiguration('longitude_ref')
    longitude_ref_launch_arg = DeclareLaunchArgument('longitude_ref', default_value='0')

    odom_topic = LaunchConfiguration('odom_topic')
    odom_topic_launch_arg = DeclareLaunchArgument('odom_topic', default_value='pose_gt')

    gps_topic = LaunchConfiguration('gps_topic')
    gps_topic_launch_arg = DeclareLaunchArgument('gps_topic', default_value='gps')

    gamma = LaunchConfiguration('gamma')
    gamma_launch_arg = DeclareLaunchArgument('gamma', default_value='gamma')

    gain = LaunchConfiguration('gain')
    gain_launch_arg = DeclareLaunchArgument('gain', default_value='gain')

    radius = LaunchConfiguration('radius')
    radius_launch_arg = DeclareLaunchArgument('radius', default_value='radius')

    update_rate = LaunchConfiguration('update_rate')
    update_rate_launch_arg = DeclareLaunchArgument('update_rate', default_value='1')

    use_geo_coordinates = LaunchConfiguration('use_geo_coordinates')
    use_geo_coordinates_launch_arg = DeclareLaunchArgument('use_geo_coordinates', default_value='use_geo_coordinates')

    reference_salinity_value = LaunchConfiguration('reference_salinity_value')
    reference_salinity_value_launch_arg = DeclareLaunchArgument('reference_salinity_value', default_value='35.0')

    salinity_unit = LaunchConfiguration('salinity_unit')
    salinity_unit_launch_arg = DeclareLaunchArgument('salinity_unit', default_value='ppt')

    publish_salinity = LaunchConfiguration('publish_salinity')
    publish_salinity_launch_arg = DeclareLaunchArgument('publish_salinity', default_value='true')

    use_odom = LaunchConfiguration('use_odom')
    use_odom_launch_arg = DeclareLaunchArgument('use_odom', default_value='false')

    use_gps = LaunchConfiguration('use_gps')
    use_gps_launch_arg = DeclareLaunchArgument('use_gps', default_value='false')

    return LaunchDescription([
        uuv_name_launch_arg,
        latitude_ref_launch_arg,
        longitude_ref_launch_arg,
        odom_topic_launch_arg,
        gps_topic_launch_arg,
        gamma_launch_arg,
        gain_launch_arg,
        radius_launch_arg,
        update_rate_launch_arg,
        use_geo_coordinates_launch_arg,
        reference_salinity_value_launch_arg,
        salinity_unit_launch_arg,
        publish_salinity_launch_arg,
        use_odom_launch_arg,
        use_gps_launch_arg,
        Node(
            package="uuv_cpc_sensor",
            executable="uuv_cpc_ros_sensor_node",
            name="uuv_cpc_ros_sensor_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"uuv_name": uuv_name,
                    "latitude_ref": latitude_ref,
                    "longitude_ref": longitude_ref,
                    "odom_topic": odom_topic,
                    "gps_topic": gps_topic,
                    "gamma": gamma,
                    "gain": gain,
                    "radius": radius,
                    "update_rate": update_rate,
                    "use_geo_coordinates": use_geo_coordinates,
                    "reference_salinity_value": reference_salinity_value,
                    "salinity_unit": salinity_unit,
                    "publish_salinity": publish_salinity,
                    "use_odom": use_odom,
                    "use_gps": use_gps}
            ]
        )
    ])
