#!/usr/bin/env python
# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""**Description**

Passive turbulent plume server node.

!!! note "See also"

    [Description of the turbulent plume model](../user_guide/introduction.md)

**Input ROS parameters**

* `update_rate` (*default:* `5`, *type:* `int` or `float`):  Update rate for the plume particle point cloud update

**Launch file snippet**

```xml
<group ns="plume">
    <node name="plume_simulation_server"
          pkg="uuv_plume_simulator3"
          type="plume_server"
          output="screen">
      <remap from="current_vel" to="/hydrodynamics/current_velocity"/>
      <rosparam subst_value="true">
            update_rate: 5
      </rosparam>
    </node>
</group>
```

**ROS services**

> **`create_spheroid_plume`**

*Service description file*

[`uuv_plume_msgs/CreateSpheroidPlume`](uuv_plume_msgs.md#createspheroidplume)

*Service call*

```bash
rosservice call /<plume_namespace>/create_spheroid_plume "source: {x: 0.0, y: 0.0, z: 0.0}
orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
n_points: 0
a: 0.0
c: 0.0
x_min: 0.0
x_max: 0.0
y_min: 0.0
y_max: 0.0
z_min: 0.0
z_max: 0.0"
```

Create a static plume in the shape of a spheroid.

* `source`: Plume source position wrt ENU frame
* `orientation`: Spheroid orientation in quaternion
* `n_points`: Number of plume particles
* `a` and `c`: Spheroid's semi-axis length
* `x_min`, `x_max`, `y_min`, `y_max`, `z_min`, `z_max`: Limits of the bounding box where the plume particles are allowed to exist

> **`create_passive_scalar_turbulent_plume`**

*Service description file*

[`uuv_plume_msgs/CreatePassiveScalarTurbulentPlume`](uuv_plume_msgs.md#createpassivescalarturbulentplume)

*Service call*

```bash
rosservice call /<plume_namespace>/create_passive_scalar_turbulent_plume "turbulent_diffusion_coefficients: {x: 0.0, y: 0.0, z: 0.0}
source: {x: 0.0, y: 0.0, z: 0.0}
buoyancy_flux: 0.0
stability_param: 0.0
n_points: 0
max_particles_per_iter: 0
x_min: 0.0
x_max: 0.0
y_min: 0.0
y_max: 0.0
z_min: 0.0
z_max: 0.0
max_life_time: 0.0" 
```

Create a passive turbulent plume.

* `turbulent_diffusion_coefficients`: Coefficients ruling the diffusion of the particles for each degree of freedom of the particle.
* `source`: Position of the plume source wrt ENU frame
* `buoyancy_flux` and `stability_param`: Parameters controlling the plume rise
* `n_points`: Maximum number of plume particles to be generated
* `max_particles_per_iter`: Maximum number of particles generated at the source position at each iteration
* `x_min`, `x_max`, `y_min`, `y_max`, `z_min`, `z_max`: Limits of the bounding box where the plume particles are allowed to exist
* `max_life_time`: Maximum life time of each particle in seconds

> **`set_plume_limits`**

*Service description file*

[`uuv_plume_msgs/SetPlumeLimits`](uuv_plume_msgs.md#setplumelimits)

*Service call*

```bash
rosservice call /<plume_namespace>/set_plume_limits "{x_min: 0.0, x_max: 0.0, y_min: 0.0, y_max: 0.0, z_min: 0.0, z_max: 0.0}"
```

Sets the bounds of the box where the plume particles are allowed to exist.

* `x_min`, `x_max`, `y_min`, `y_max`, `z_min`, `z_max`: Limits of the bounding box where the plume particles are allowed to exist

> **`set_plume_config`**

*Service description file*

[`uuv_plume_msgs/SetPlumeConfiguration`](uuv_plume_msgs.md#setplumeconfiguration)

*Service call*

```bash
rosservice call /<plume_namespace>/set_plume_config "n_points: 0
max_particles_per_iter: 0"
```

Configure the plume's particle generation parameters.

* `n_points`: Maximum number of plume particles to be generated
* `max_particles_per_iter`: Maximum number of particles generated at the source position at each iteration

> **`get_plume_config`**

*Service description file*

[`uuv_plume_msgs/GetPlumeConfiguration`](uuv_plume_msgs.md#getplumeconfiguration)

*Service call*

```bash
rosservice call /<plume_namespace>/get_plume_config
```

Return the plume's generation configuration parameters, including maximum number of particles,
maximum number of particles per iteration, source position and limits of the bounding box.

> **`delete_plume`**

*Service description file*

[`uuv_plume_msgs/DeletePlume`](uuv_plume_msgs.md#deleteplume)

*Service call*

```bash
rosservice call /<plume_namespace>/delete_plume
```

Delete the source and the particles.

> **`set_plume_source_position`**

*Service description file*

[`uuv_plume_msgs/SetPlumeSourcePosition`](uuv_plume_msgs.md#setplumesourceposition)

*Service call*

```bash
rosservice call /<plume_namespace>/set_plume_source_position "source:
  x: 0.0
  y: 0.0
  z: 0.0"
```

Set plume source position wrt ENU frame

* `source`: New plume source position wrt ENU frame

> **`get_plume_source_position`**

*Service description file*

[`uuv_plume_msgs/GetPlumeSourcePosition`](uuv_plume_msgs.md#getplumesourceposition)

*Service call*

```bash
rosservice call /<plume_namespace>/get_plume_source_position
```

Return the plume source position coordinates.

> **`get_num_particles`**

*Service description file*

[`uuv_plume_msgs/GetNumParticles`](uuv_plume_msgs.md#getnumparticles)

*Service call*

```bash
rosservice call /<plume_namespace>/get_num_particles
```

Return current number of plume particles.

> **`store_plume_state`**

*Service description file*

[`uuv_plume_msgs/StorePlumeState`](uuv_plume_msgs.md#storeplumestate)

*Service call*

```bash
rosservice call /<plume_namespace>/store_plume_state "output_dir: ''
filename: ''"
```

Store the position and time of creation for of each particle in a file as an YAML file.

* `output_dir`: Path to output directory
* `filename`: Output YAML file

> **`load_plume_particles`**

*Service description file*

[`uuv_plume_msgs/LoadPlumeParticles`](uuv_plume_msgs.md#loadplumeparticles)

*Service call*

```bash
rosservice call /<plume_namespace>/load_plume_particles "plume_file: ''" 
```

Load an YAML file with the plume particles' list positions and time of creation.

* `plume_file`: Plume YAML file

"""
from __future__ import print_function

import sys

from ros_gz_interfaces.msg import Float32Array
from sensor_msgs.msg import PointCloud, PointCloud2
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from threading import Lock
from .uuv_plume_model.plume import Plume
import numpy as np
from uuv_plume_msgs.srv import *
import os
import yaml

import rclpy
from rclpy.node import Node


class PlumeSimulatorServer(Node):
    """
    The plume server allows a plume entity to be created and configured in the
    on going simulation using ROS services. This class also contains timer
    callback functions to update the visual markers and plume point clouds
    that can be visualized in RViz.

    Examples
    --------

    To start this plume server, be sure that Gazebo has been initialized and
    then run::

        $ roslaunch uuv_plume_simulator3 start_plume_server.launch

    Check the files *scripts/set_demo_spheroid_plume* and
    *scripts/set_demo_turbulent_plume* to see examples on how to use the
    service calls to create plumes.

    .. note::

        Check the chemical particle concentration sensor plugin (*CPCROSPlugin*)
        to generate simulated sensor data for the particle concentration around
        the vehicle.
    """
    def __init__(self):
        super().__init__("PlumeSimulatorServer")
        # Plume model (to be defined by a service call)
        self._model = None

        # Current time stamp and time step
        # self._t = rclpy.time.Time().to_msg()
        self._t = self.get_clock().now()
        # self._t = self.get_clock().now() # https://github.com/mikeferguson/ros2_cookbook/blob/main/rclpy/time.md
        self._dt = 0.0

        # Define default values for the parameters
        self._update_rate = 5.0
        self._gamma = 0.001
        self._gain = 1.0
        self._radius = 3.0
        self._saturation = 0.0
        self._noise_amplitude = 0.0
        self._noise_sigma = 1.0
        self._latitude = 100.0
        self._longitude = 100.0
        self._salinity_unit = "ppm"
        self._reference_salinity_value = 0.0
        self._use_geo_coordinates = False
        self._use_odom = False
        self._use_gps = False
        self._publish_salinity = True

        # Declare parameters
        self._update_rate = float(self.declare_parameter('update_rate', self._update_rate).value)
        self._gamma = float(self.declare_parameter('gamma', self._gamma).value)
        self._gain = float(self.declare_parameter('gain', self._gain).value)
        self._radius = float(self.declare_parameter('radius', self._radius).value)
        self._saturation = float(self.declare_parameter('saturation', self._saturation).value)
        self._noise_amplitude = float(self.declare_parameter('noise_amplitude', self._noise_amplitude).value)
        self._noise_sigma = float(self.declare_parameter('noise_sigma', self._noise_sigma).value)
        self._latitude = float(self.declare_parameter('latitude', self._latitude).value)
        self._longitude = float(self.declare_parameter('longitude', self._longitude).value)
        self._reference_salinity_value = float(self.declare_parameter('reference_salinity_value', self._reference_salinity_value).value)
        self._salinity_unit = str(self.declare_parameter('salinity_unit', self._salinity_unit).value)
        self._use_geo_coordinates = bool(self.declare_parameter('use_geo_coordinates', self._use_geo_coordinates).value)
        self._use_odom = bool(self.declare_parameter('use_odom', self._use_odom).value)
        self._use_gps = bool(self.declare_parameter('use_gps', self._use_gps).value)
        self._publish_salinity = bool(self.declare_parameter('publish_salinity', self._publish_salinity).value)

        # Assert the delcared parameters
        assert isinstance(self._update_rate, float), 'port parameter must be a float'
        assert isinstance(self._gamma, float), 'port parameter must be a float'
        assert isinstance(self._gain, float), 'port parameter must be a float'
        assert isinstance(self._radius, float), 'port parameter must be a float'
        assert isinstance(self._saturation, float), 'port parameter must be a float'
        assert isinstance(self._noise_amplitude, float), 'port parameter must be a float'
        assert isinstance(self._noise_sigma, float), 'port parameter must be a float'
        assert isinstance(self._latitude, float), 'port parameter must be a float'
        assert isinstance(self._longitude, float), 'port parameter must be a float'
        assert isinstance(self._reference_salinity_value, float), 'port parameter must be a float'
        assert isinstance(self._salinity_unit, str), 'port parameter must be a float'
        assert isinstance(self._use_geo_coordinates, bool), 'port parameter must be a boolean'
        assert isinstance(self._use_odom, bool), 'port parameter must be a float'
        assert isinstance(self._use_gps, bool), 'port parameter must be a float'
        assert isinstance(self._publish_salinity, bool), 'port parameter must be a float'

        # Log the parameters
        self._update_rate = max(0.05, self._update_rate)
        self.get_logger().info(f'Update rate [Hz]: {format(self._update_rate, ".3f")}')
        self.get_logger().info(f'Gamma : {format(self._gamma, ".3f")}')
        self.get_logger().info(f'Gain : {format(self._gain, ".3f")}')
        self.get_logger().info(f'Radius of Plume : {format(self._radius, ".3f")}')
        self.get_logger().info(f'Saturation : {format(self._saturation, ".3f")}')
        self.get_logger().info(f'Noise Amplitude : {format(self._noise_amplitude, ".3f")}')
        self.get_logger().info(f'Noise Variance : {format(self._noise_sigma, ".3f")}')
        self.get_logger().info(f'Plume Latitude : {format(self._latitude, ".3f")}')
        self.get_logger().info(f'Plume Longitude : {format(self._longitude, ".3f")}')
        self.get_logger().info(f'Reference Salinity Value : {format(self._reference_salinity_value, ".3f")}')
        self.get_logger().info(f'salinity_unit: {self._salinity_unit}')
        self.get_logger().info(f'useGeoCoordinates [Boolean]: {self._use_geo_coordinates}')
        self.get_logger().info(f'useOdometry [Boolean]: {self._use_odom}')
        self.get_logger().info(f'useGPS [Boolean]: {self._use_gps}')
        self.get_logger().info(f'Publish Salinity [Boolean]: {self._publish_salinity}')

        # Definition of service callbacks
        # self._services = dict()

        # Service callback to create a static plume in the form of a
        # spheroid
        #self._services['create_spheroid_plume'] = (
        self.create_service(
            CreateSpheroidPlume,
            'create_spheroid_plume',
            self.create_spheroid_plume)

        # Service callback to create a dynamic passive scalar turbulent plume
        #self._services['create_passive_scalar_turbulent_plume'] = (
        self.create_service(
            CreatePassiveScalarTurbulentPlume,
            'create_passive_scalar_turbulent_plume',
            self.create_passive_scalar_turbulent_plume)

        # Service callback to configure the bounding box limiting the plume
        # source and particles
        #self._services['set_plume_limits'] = (
        self.create_service(
            SetPlumeLimits,
            'set_plume_limits',
            self.set_plume_limits)

        # Service callback to change the maximum number of particles to be
        # generated by a single plume and, in the case of the dynamic plume,
        # the maximum number of particles generated at each iteration
        # self._services['set_plume_config'] =
        self.create_service(
            SetPlumeConfiguration,
            'set_plume_config',
            self.set_plume_configuration)

        # Service callback to return general plume configuration parameters,
        # such as maximum number of particles, model name, etc.
        #self._services['get_plume_config'] = (
        self.create_service(
            GetPlumeConfiguration,
            'get_plume_config',
            self.get_plume_configuration)

        # Service callback to the delete the plume model, after this is called
        # the output plume point cloud will be empty
        #self._services['delete_plume'] =
        self.create_service(
            DeletePlume,
            'delete_plume',
            self.delete_plume)

        # Service callback to set the position of the plume source
        #self._services['set_plume_source_position'] =
        self.create_service(
            SetPlumeSourcePosition,
            'set_plume_source_position',
            self.set_plume_source_position)

        # Service callback to get the position of the plume source
        #self._services['get_plume_source_position'] =
        self.create_service(
            GetPlumeSourcePosition,
            'get_plume_source_position',
            self.get_plume_source_position)

        #self._services['get_num_particles'] =
        self.create_service(
            GetNumParticles,
            'get_num_particles',
            self.get_num_particles)

        #self._services['store_plume_state'] = s
        self.create_service(
            StorePlumeState,
            'store_plume_state',
            self.store_plume_state)

        #self._services['load_plume_particles'] =
        self.create_service(
            LoadPlumeParticles,
            'load_plume_particles',
            self.load_plume_particles)

        # Publisher for the time_creation values of the plume particles
        # This is only used for Bridging it into gazebo so the plume can be visualized in gazebo as PointCloud
        self._plume_time_creation_publisher = self.create_publisher(
            Float32Array,
            'time_creation',
            1)

        # Publisher for the plume visual markers
        self._plume_marker_publisher = self.create_publisher(
            MarkerArray,
            'markers',
            1)

        # Publisher for the plume particles point cloud
        self._plume_point_cloud_publisher = self.create_publisher(
            PointCloud,
            'particles',
            1)

        # Publisher for the plume particles point cloud
        self._plume_point_cloud2_publisher = self.create_publisher(
            PointCloud2,
            'particles2',
            1)

        # Subscriber to the current velocity
        #self._current_vel_subscriber =
        self.create_subscription(
            Point,
            'ocean_current',
            self.current_vel_callback,
            10)

        self._loading_plume = Lock()
        # Timer called to update both the particle point cloud and visual
        # markers
        self._update_plume_timer = self.create_timer(float(1 / self._update_rate), self.update_plume)

        # rospy.Timer(rospy.Duration(1 / self._update_rate), self.update_plume))

    def current_vel_callback(self, msg):
        """
        Subscriber callback function for the current velocity vector update.
        """
        if self._model is None:
            return

        self._model.update_current_vel([msg.x,
                                        msg.y,
                                        msg.z])

    def delete_plume(self, request):
        """
        Service function callback to delete the plume model. All markers and
        point cloud will be published with empty topics.
        """
        if self._model is not None:
            del self._model
        self._model = None
        self.get_logger().info('Plume deleted')
        return DeletePlumeResponse(True)

    def set_plume_source_position(self, request):
        """
        Service function callback that sets a new position for the plume
        source wrt the ENU frame
        """
        if self._model is None:
            self.get_logger().warn('No plume model has been created')
            return SetPlumeSourcePositionResponse(False)
        else:
            self.get_logger().info('Plume source position set to:')
            self.get_logger().info('\t(X, Y, Z) [m] wrt the ENU frame: (%.2f, %.2f, %.2f)' % (request.source.x, request.source.y, request.source.z))

            self._model.source_pos = \
                [request.source.x, request.source.y, request.source.z]
            return SetPlumeSourcePositionResponse(True)

    def get_num_particles(self, request):
        """
        Return the number of particles currently active.
        """
        if self._model is None:
            self.get_logger().warn('No plume model has been created')
            return GetNumParticlesResponse(0)
        else:
            self.get_logger().info('Number of plume particles requested: %d' % self._model.num_particles)
            return GetNumParticlesResponse(int(self._model.num_particles))

    def get_plume_source_position(self, request):
        """
        Service function callback that returns the position wrt to the ENU
        frame for the plume source.
        """
        if self._model is None:
            self.get_logger().warn('No plume model has been created')
            return GetPlumeSourcePositionResponse(Point(-1, -1, -1))
        else:
            self.get_logger().info('Current plume source position:')
            self.get_logger().info('\t(X, Y, Z) [m] wrt the ENU frame: (%.2f, %.2f, %.2f)' % (self._model.source_pos[0], self._model.source_pos[1], self._model.source_pos[2]))

            return GetPlumeSourcePositionResponse(
                Point(self._model.source_pos[0],
                      self._model.source_pos[1],
                      self._model.source_pos[2]))

    def get_plume_configuration(self, request):
        """
        Service function callback to return the configuration parameters for
        the current plume model being used.
        """
        if self._model is None:
            self.get_logger().warn('No plume model has been created')
            return GetPlumeConfigurationResponse(
                '',
                0,
                0,
                Point(0, 0, 0),
                0, 0, 0, 0, 0, 0)
        else:
            return GetPlumeConfigurationResponse(
                self._model.LABEL,
                self._model.n_points,
                (0 if self._model.LABEL == 'spheroid' else \
                    self._model.max_particles_per_iter),
                Point(self._model.source_pos[0],
                      self._model.source_pos[1],
                      self._model.source_pos[2]),
                self._model.x_lim[0],
                self._model.x_lim[1],
                self._model.y_lim[0],
                self._model.y_lim[1],
                self._model.z_lim[0],
                self._model.z_lim[1])

    def set_plume_configuration(self, request):
        """
        Service function callback to set general plume configuration parameters.
        """
        if self._model is None:
            self.get_logger().warn('No plume model has been created')
            return SetPlumeConfigurationResponse(False)
        try:
            if not self._model.set_n_points(request.n_points):
                return SetPlumeConfigurationRequest(False)

            self.get_logger().info('Change in the plume configuration:')
            self.get_logger().info('\t- # particles=%d' % request.n_points)

            if request.max_particles_per_iter > 0 and self._model.LABEL == 'passive_scalar_turbulence':
                if not self._model.set_max_particles_per_iter(request.max_particles_per_iter):
                    self.get_logger().error(f'Error ocurred while setting the maximum number of particles per iteration, value={int(request.max_particles_per_iter)}')
                    return SetPlumeConfigurationRequest(False)
                self.get_logger().info('\t- Max. number of particles per iteration=%d', request.max_particles_per_iter)
            return SetPlumeConfigurationResponse(True)
        except Exception as e:
            self.get_logger().error(f'Error setting the plume configuration, message={str(e)}')
            return SetPlumeConfigurationResponse(False)

    def set_plume_limits(self, request):
        """
        Service function callback to set the plume bounding box limits.
        """
        if self._model is None:
            self.get_logger().warn('No plume model has been created')
            return SetPlumeLimitsResponse(False)

        try:
            self._model.set_x_lim(request.x_min, request.x_max)
            self._model.set_y_lim(request.y_min, request.y_max)
            self._model.set_z_lim(request.z_min, request.z_max)
            return SetPlumeLimitsResponse(True)
        except Exception as e:
            self.get_logger().error(f'Error setting the plume limits, message={str(e)}')
            return SetPlumeLimitsResponse(False)

    def create_spheroid_plume(self, request):
        """
        Service function callback to create a static spheroid plume model.
        """
        if request.a <= 0 or request.c <= 0:
            return CreateSpheroidPlumeResponse(False)
        if request.n_points <= 0:
            return CreateSpheroidPlumeResponse(False)

        try:
            q = np.array([request.orientation.x,
                          request.orientation.y,
                          request.orientation.z,
                          request.orientation.w])
            self._model = Plume.create_plume_model(
                'spheroid',
                request.a,
                request.c,
                q,
                [request.source.x, request.source.y, request.source.z],
                request.n_points,
                self.get_clock().now())

            self._model.set_x_lim(request.x_min, request.x_max)
            self._model.set_y_lim(request.y_min, request.y_max)
            self._model.set_z_lim(request.z_min, request.z_max)
            self.get_logger().info('Spheroid plume created!')
            self.get_logger().info(request)
            return CreateSpheroidPlumeResponse(True)
        except Exception as e:
            self.get_logger().error(f'Error creating spheroid plume model, message={str(e)}')
            self._model = None
            return CreateSpheroidPlumeResponse(False)

    def create_passive_scalar_turbulent_plume(self, request, response):
        """
        Service function callback to create a passive scalar turbulent plume
        model.
        """
        try:
            self._model = Plume.create_plume_model(
                'passive_scalar_turbulence',
                [request.turbulent_diffusion_coefficients.x,
                 request.turbulent_diffusion_coefficients.y,
                 request.turbulent_diffusion_coefficients.z],
                request.buoyancy_flux,
                request.stability_param,
                [request.source.x, request.source.y, request.source.z],
                request.n_points,
                float(self.get_clock().now().nanoseconds)/1000000000,
                request.max_particles_per_iter,
                request.max_life_time)

            self._model.set_x_lim(request.x_min, request.x_max)
            self._model.set_y_lim(request.y_min, request.y_max)
            self._model.set_z_lim(request.z_min, request.z_max)
            self.get_logger().info('Passive turbulent plume created!')
            self.get_logger().info('Turbulent diffusion coefficients: (%.3f, %.3f, %.3f)' % (
                request.turbulent_diffusion_coefficients.x,
                request.turbulent_diffusion_coefficients.y,
                request.turbulent_diffusion_coefficients.z))
            self.get_logger().info('Buoyancy flux: %.4f' % request.buoyancy_flux)
            self.get_logger().info('Stability parameters: %.4f' % request.stability_param)
            self.get_logger().info('Source position wrt ENU frame: (%.2f %.2f, %.2f)' % (
                request.source.x, request.source.y, request.source.z))
            self.get_logger().info('Max. number of particles: %d' % request.n_points)
            self.get_logger().info('Max. number of particles per iteration: %d' % request.max_particles_per_iter)
            self.get_logger().info('Particle max. life time: %.2f' % request.max_life_time)

            response.success = True
            return response

            # return CreatePassiveScalarTurbulentPlumeResponse(True)
        except Exception as e:
            self.get_logger().error(f'Error creating passive turbulent plume model, message={str(e)}')
            self._model = None
            response.success = False
            return response
            # return CreatePassiveScalarTurbulentPlumeResponse(False)

    def store_plume_state(self, request):
        self.get_logger().info('Store plume state service called')
        if self._model is None:
            self.get_logger().error('No plume model available!')
            return StorePlumeStateResponse(False, '')
        output_dir = '/tmp'
        if os.path.isdir(request.output_dir):
            output_dir = request.output_dir
        filename = 'plume.yaml'
        if '.yaml' in request.filename or '.yml' in request.filename:
            filename = request.filename

        abs_filename = os.path.join(output_dir, filename)

        data = dict(x=self._model.x.flatten().tolist(),
                    y=self._model.y.flatten().tolist(),
                    z=self._model.z.flatten().tolist(),
                    time_creation=self._model.time_of_creation.flatten().tolist())

        with open(abs_filename, 'w') as yaml_file:
            yaml_file.write(yaml.safe_dump(data))

        self.get_logger().info('Plume particles stored in=%s', abs_filename)
        return StorePlumeStateResponse(True, abs_filename)

    def load_plume_particles(self, request):
        self.get_logger().info('Load plume particles service called')
        if self._model is None:
            self.get_logger().error('No plume model available!')
            return LoadPlumeParticlesResponse(False)

        if not os.path.isfile(request.plume_file):
            self.get_logger().error(f'Plume file provided is invalid, filename={request.plume_file}')
            return LoadPlumeParticlesResponse(False)
        self._loading_plume.acquire()
        try:
            with open(request.plume_file, 'r') as plume_file:
                plume_particles = yaml.load(plume_file)
            self.get_logger().info('# particles loaded=%d', len(plume_particles['x']))
            self._model.set_plume_particles(
                float(self.get_clock().now().to_msg().nanoseconds)/1000000000,
                plume_particles['x'],
                plume_particles['y'],
                plume_particles['z'],
                plume_particles['time_creation'])
        except Exception as e:
            self.get_logger().error(f'Error while loading plume file, message={str(e)}')
            return LoadPlumeParticlesResponse(False)

        self._loading_plume.release()
        return LoadPlumeParticlesResponse(True)

    def update_plume(self):
        """
        Callback function for the plume timer which contains the update for
        the plume particle point cloud and visual markers for RViz.
        """
        self._dt = self.get_clock().now() - self._t
        self._loading_plume.acquire()
        if self._dt <= rclpy.duration.Duration(seconds=0.0):
            return True

        if self._model is None:
            marker = MarkerArray()
            self._plume_marker_publisher.publish(marker)
            pc_msg = PointCloud()
            pc_msg.header.stamp = self.get_clock().now().to_msg()
            pc_msg.header.frame_id = 'world'
            self._plume_point_cloud_publisher.publish(pc_msg)

            pc2_msg = PointCloud2()
            pc2_msg.header.stamp = self.get_clock().now().to_msg()
            pc2_msg.header.frame_id = 'world'
            self._plume_point_cloud2_publisher.publish(pc2_msg)

            self._loading_plume.release()
            return True

        now = self.get_clock().now()
        msg = self.get_clock().now().to_msg()

        if not self._model.update(float(self.get_clock().now().nanoseconds)/1000000000):
            self.get_logger().error('Error while updating the plume particles positions')
            self._loading_plume.release()
            return True

        marker = self._model.get_markers(stamp=self.get_clock().now().to_msg())

        if marker is not None:
            self._plume_marker_publisher.publish(marker)

        pc_msg = self._model.get_point_cloud_as_msg(stamp=self.get_clock().now().to_msg())
        if pc_msg is not None:
            self._plume_point_cloud_publisher.publish(pc_msg)

        pc2_msg = self._model.get_point_cloud2_as_msg(stamp=self.get_clock().now().to_msg())
        if pc2_msg is not None:
            self._plume_point_cloud2_publisher.publish(pc2_msg)

        time_of_creation = self._model.time_of_creation
        if time_of_creation is not None:
            self._plume_time_creation_publisher.publish(Float32Array(data=time_of_creation.flatten().tolist()))

        self._loading_plume.release()
        return True


def main():
    rclpy.init(args=sys.argv)
    node = PlumeSimulatorServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    node = rclpy.create_node('plume_simulation_server')

    node.get_logger().info('Created Plume simulator server node')


if __name__ == '__main__':
    main()
