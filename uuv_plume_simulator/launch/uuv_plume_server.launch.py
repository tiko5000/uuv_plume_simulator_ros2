from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    
    # Define variables with the default values
    # update_rate = 5.0
    # gamma = 0.001
    # gain = 1.0
    # radius = 3.0
    # saturation = 0.0
    # noise_amplitude = 0.0
    # noise_sigma = 1.0
    # latitude = 100.0
    # longitude = 100.0
    # salinity_unit = "ppm"
    # reference_salinity_value = 0.0
    # use_geo_coordinates = False
    # use_odom = False
    # use_gps = False
    # publish_salinity = True
    # pointcloud_frame_id = "map"
    # sensor_frame_id = "base_link"

    update_rate = LaunchConfiguration('update_rate')
    update_rate_launch_arg = DeclareLaunchArgument('update_rate', default_value='5.0')

    gamma = LaunchConfiguration('gamma')
    gamma_launch_arg = DeclareLaunchArgument('gamma', default_value='0.001')

    gain = LaunchConfiguration('gain')
    gain_launch_arg = DeclareLaunchArgument('gain', default_value='1.0')

    radius = LaunchConfiguration('radius')
    radius_launch_arg = DeclareLaunchArgument('radius', default_value='3.0')

    saturation = LaunchConfiguration('saturation')
    saturation_launch_arg = DeclareLaunchArgument('saturation', default_value='0.0')

    noise_amplitude = LaunchConfiguration('noise_amplitude')
    noise_amplitude_launch_arg = DeclareLaunchArgument('noise_amplitude', default_value='0.0')

    noise_sigma = LaunchConfiguration('noise_sigma')
    noise_sigma_launch_arg = DeclareLaunchArgument('noise_sigma', default_value='1.0')

    latitude = LaunchConfiguration('latitude')
    latitude_launch_arg = DeclareLaunchArgument('latitude', default_value='100.0')

    longitude = LaunchConfiguration('longitude')
    longitude_launch_arg = DeclareLaunchArgument('longitude', default_value='100.0')

    salinity_unit = LaunchConfiguration('salinity_unit')
    salinity_unit_launch_arg = DeclareLaunchArgument('salinity_unit', default_value='ppm')

    reference_salinity_value = LaunchConfiguration('reference_salinity_value')
    reference_salinity_value_launch_arg = DeclareLaunchArgument('reference_salinity_value', default_value='0.0')

    use_geo_coordinates = LaunchConfiguration('use_geo_coordinates')
    use_geo_coordinates_launch_arg = DeclareLaunchArgument('use_geo_coordinates', default_value='false')

    use_odom = LaunchConfiguration('use_odom')
    use_odom_launch_arg = DeclareLaunchArgument('use_odom', default_value='false')

    use_gps = LaunchConfiguration('use_gps')
    use_gps_launch_arg = DeclareLaunchArgument('use_gps', default_value='false')

    publish_salinity = LaunchConfiguration('publish_salinity')
    publish_salinity_launch_arg = DeclareLaunchArgument('publish_salinity', default_value='true')

    pointcloud_frame_id = LaunchConfiguration('pointcloud_frame_id')
    pointcloud_frame_id_launch_arg = DeclareLaunchArgument('pointcloud_frame_id', default_value='map')

    sensor_frame_id = LaunchConfiguration('sensor_frame_id')
    sensor_frame_id_launch_arg = DeclareLaunchArgument('sensor_frame_id', default_value='base_link')

    return LaunchDescription([
        update_rate_launch_arg,
        gamma_launch_arg,
        gain_launch_arg,
        radius_launch_arg,
        saturation_launch_arg,
        noise_amplitude_launch_arg,
        noise_sigma_launch_arg,
        latitude_launch_arg,
        longitude_launch_arg,
        salinity_unit_launch_arg,
        reference_salinity_value_launch_arg,
        use_geo_coordinates_launch_arg,
        use_odom_launch_arg,
        use_gps_launch_arg,
        publish_salinity_launch_arg,
        pointcloud_frame_id_launch_arg,
        sensor_frame_id_launch_arg,
        Node(
            package="uuv_plume_simulator",
            executable="plume_server",
            name="plume_server_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "update_rate": update_rate,
                    "gamma": gamma,
                    "gain": gain,
                    "radius": radius,
                    "saturation": saturation,
                    "noise_amplitude": noise_amplitude,
                    "noise_sigma": noise_sigma,
                    "latitude": latitude,
                    "longitude": longitude,
                    "salinity_unit": salinity_unit,
                    "reference_salinity_value": reference_salinity_value,
                    "use_geo_coordinates": use_geo_coordinates,
                    "use_odom": use_odom,
                    "use_gps": use_gps,
                    "publish_salinity": publish_salinity,
                    "pointcloud_frame_id": pointcloud_frame_id,
                    "sensor_frame_id": sensor_frame_id}
            ]
        )
    ])
