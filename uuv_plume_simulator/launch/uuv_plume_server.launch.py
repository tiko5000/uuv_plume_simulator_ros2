from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    # Define variables with the default values

    update_rate = 5.0
    gamma = 0.001
    gain = 1.0
    radius = 3.0
    saturation = 0.0
    noise_amplitude = 0.0
    noise_sigma = 1.0
    latitude = 100.0
    longitude = 100.0
    salinity_unit = "ppm"
    reference_salinity_value = 0.0
    use_geo_coordinates = False
    use_odom = False
    use_gps = False
    publish_salinity = True

    return LaunchDescription([
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
                    "publish_salinity": publish_salinity}
            ]
        )
    ])
