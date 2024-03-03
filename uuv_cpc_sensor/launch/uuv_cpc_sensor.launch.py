from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    # Define variables with the default values

    uuv_name = "uuv_name"
    latitude_ref = "0"
    longitude_ref = "0"
    odom_topic = "pose_gt"
    gps_topic = "gps"
    gamma = "gamma"
    gain = "gain"
    radius = "radius"
    update_rate = "1"
    use_geo_coordinates = "use_geo_coordinates"
    reference_salinity_value = "35.0"
    salinity_unit = "ppt"
    sensor_frame_id = "$(arg uuv_name)/base_link"
    publish_salinity = "true"
    use_odom = "false"
    use_gps = "false"

    return LaunchDescription([
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
                    "sensor_frame_id": sensor_frame_id,
                    "publish_salinity": publish_salinity,
                    "use_odom": use_odom,
                    "use_gps": use_gps}
            ]
        )
    ])
