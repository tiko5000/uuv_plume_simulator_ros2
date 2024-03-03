#include "rclcpp/rclcpp.hpp"
#include <rcpputils/asserts.hpp>
#include "uuv_plume_msgs/msg/particle_concentration.hpp"
#include "uuv_plume_msgs/msg/salinity.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <tf2_msgs/msg/tf_message.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <random>
#include <chrono>
using namespace std::chrono_literals;


namespace uuv_plume_simulator {

#define CONCENTRATION_UNIT_PPT "ppt"
#define CONCENTRATION_UNIT_PPM "ppm"
#define CONCENTRATION_UNIT_PSU "psu"

class CPCSensor : public rclcpp::Node
{
public:
    CPCSensor() : Node("cpc_sensor")
    {
        // Subscribe to the particles topic
        particles_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/particles2", 10, std::bind(&CPCSensor::OnPlumeParticlesUpdate, this, std::placeholders::_1));

        // Publish to the concentration topic
        concentration_publisher_ = this->create_publisher<uuv_plume_msgs::msg::ParticleConcentration>("/concentration", 10);

        tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
    }

private:
    // Callback function for particles topic
    void OnPlumeParticlesUpdate(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        this->updateMeasurement = true;
        this->areParticlesInit = true;
        if (this->useTFUpdate)
        {
            // Read the current position of the sensor frame
            geometry_msgs::msg::TransformStamped childTransform;
            // std::string targetFrame = msg->header.frame_id;
            std::string targetFrame ="world";
            std::string sourceFrame = this->sensorFrameID;
            try
            {
              childTransform = this->tfBuffer->lookupTransform(
                targetFrame, sourceFrame, tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                targetFrame.c_str(), sourceFrame.c_str(), ex.what());
            return;
            }

            this->cartPos.x = childTransform.transform.translation.x;
            this->cartPos.y = childTransform.transform.translation.y;
            this->cartPos.z = childTransform.transform.translation.z;
        }

        this->concentrationMsg.header.frame_id = msg->header.frame_id;
        // this->concentrationMsg.header.stamp = this->now();
        this->concentrationMsg.header.stamp.sec = this->now().seconds();
        this->concentrationMsg.header.stamp.nanosec = this->now().nanoseconds();


        this->concentrationMsg.position.x = this->cartPos.x;
        this->concentrationMsg.position.y = this->cartPos.y;
        this->concentrationMsg.position.z = this->cartPos.z;

        if (this->useGeoCoordinates)
        {
            if (this->useGPS)
                this->concentrationMsg.geo_point = this->geoPos;
            else
            {
                double latitude, longitude, altitude;
                this->projection.Reverse(
                    this->cartPos.x, this->cartPos.y, this->cartPos.z,
                    latitude, longitude, altitude);
                this->concentrationMsg.geo_point.latitude = latitude;
                this->concentrationMsg.geo_point.longitude = longitude;
                this->concentrationMsg.geo_point.altitude = altitude;
            }
        }
        else
        {
            this->concentrationMsg.geo_point.latitude = 0;
            this->concentrationMsg.geo_point.longitude = 0;
            this->concentrationMsg.geo_point.altitude = 0;
        }

        double totalParticleConc = 0.0;
        double smoothingParam;
        double particleConc;
        double distToParticle;

        double currentTime = this->concentrationMsg.header.stamp.sec;

        double initSmoothingLength = std::pow(this->smoothingLength, 2.0 / 3); 

        RCLCPP_DEBUG(get_logger(), "Starting to Parse the PointCloud2 Msg ...");

        // Now create iterators for fields
        sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            // Compute the distance to the sensor
            distToParticle = std::sqrt(
                std::pow(*iter_x - this->cartPos.x, 2) +
                std::pow(*iter_y - this->cartPos.y, 2) +
                std::pow(*iter_z - this->cartPos.z, 2));

            // Todo currentTime - _msg->channels[0].values[i]
            smoothingParam = std::pow(initSmoothingLength +
                this->gamma * (currentTime), 1.5);

            // Compute particle concentration
            if (distToParticle >= 0 && distToParticle < smoothingParam)
            particleConc = 4.0 -
                6.0 * std::pow(distToParticle / smoothingParam, 2) +
                3.0 * std::pow(distToParticle / smoothingParam, 3);
            else if (distToParticle >= smoothingParam && distToParticle < 2 * smoothingParam)
            particleConc = std::pow(2 - distToParticle / smoothingParam, 3);
            else
            particleConc = 0.0;

            particleConc *= 1 / (4 * M_PI * std::pow(smoothingParam, 3));
            totalParticleConc += particleConc;       
        }                      

        // TODO: Applying saturation
        // this->concentrationMsg.concentration = std::min(this->gain * totalParticleConc, this->saturation);
        this->concentrationMsg.concentration = this->gain * totalParticleConc;

        if (this->publishSalinity)
        {
            this->salinityMsg.header.frame_id = msg->header.frame_id;
            // this->salinityMsg.header.stamp = this->now();
            this->salinityMsg.header.stamp.sec = this->now().seconds();
            this->salinityMsg.header.stamp.nanosec = this->now().nanoseconds();            
            this->salinityMsg.position = this->concentrationMsg.position;
            this->salinityMsg.geo_point = this->concentrationMsg.geo_point;

            // Calculating salinity
            this->salinityMsg.salinity = this->referenceSalinityValue * \
                (this->saturation - this->concentrationMsg.concentration) + \
                this->concentrationMsg.concentration * this->plumeSalinityValue;

            // Adding noise to the salinity value
            this->salinityMsg.salinity += this->noiseAmp \
                * this->noiseModel(this->rndGen);
        }
        // Adding noise to concentration value
        this->concentrationMsg.concentration += this->noiseAmp \
            * this->noiseModel(this->rndGen);
        this->updateMeasurement = false;
    }

    void OnOdometryUpdate(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Inside OnOdometryUpdate ...");        
        if(this->updateMeasurement)
            return;
        this->cartPos.x = msg->pose.pose.position.x;
        this->cartPos.y = msg->pose.pose.position.y;
        this->cartPos.z = msg->pose.pose.position.z;        
    }

    void OnGPSUpdate(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Inside OnGPSUpdate ...");        
        rcpputils::assert_true(this->useGeoCoordinates, "useGeoCoordinates must be set for OnGPSUpdate");
        if(this->updateMeasurement)
            return;
        this->geoPos.latitude = msg->latitude;
        this->geoPos.longitude = msg->longitude;
        this->geoPos.altitude = msg->altitude;
        this->projection.Forward(
            msg->latitude, msg->longitude, msg->altitude,
            this->cartPos.x, this->cartPos.y, this->cartPos.z);
    }

    void OnSensorUpdate()
    {
        RCLCPP_DEBUG(get_logger(), "Inside OnSensorUpdate ...");
        if (!this->areParticlesInit)
            return;
        this->concentration_publisher_->publish(this->concentrationMsg);
        if (this->publishSalinity)
            this->salinity_publisher_->publish(this->salinityMsg);        
    }

public:    void getParametersFromOtherNode()
    {
        // Create a parameter client to communicate with the other node
        // https://github.com/ros2/rclcpp/blob/fdaf96f2171e70f3e013610aa44df2d7e9c866a3/rclcpp/include/rclcpp/parameter_client.hpp
        auto parameter_client = std::make_shared<rclcpp::SyncParametersClient>(this,"plume_server_node");

        // Wait for the parameter service of the other node to be available
        while (!parameter_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                // Handle error or shutdown
                return;
            }
            RCLCPP_INFO(get_logger(), "Waiting for parameter service to become available...");
        }

        // Get each parameter
        this->updateRate = parameter_client->get_parameter<double>("update_rate");
        rcpputils::assert_true(this->updateRate > 0, "updateRate must be greater than 0");
        // Wall timer
        sensor_timer_ = this->create_wall_timer(1s/this->updateRate, std::bind(&CPCSensor::OnSensorUpdate, this));

        this->gamma = parameter_client->get_parameter<double>("gamma");
        rcpputils::assert_true(this->gamma >= 0, "gamma must be greater than or equal 0");

        this->gain = parameter_client->get_parameter<double>("gain");
        rcpputils::assert_true(this->gain >= 0, "gain must be greater than or equal 0");

        this->smoothingLength = parameter_client->get_parameter<double>("radius");
        rcpputils::assert_true(this->smoothingLength >= 0, "smoothingLength must be greater than or equal 0");

        if(parameter_client->has_parameter("saturation"))
            this->saturation = parameter_client->get_parameter<double>("saturation");
        rcpputils::assert_true(this->saturation >= 0, "saturation must be greater than or equal 0");

        if(parameter_client->has_parameter("noise_amplitude"))
            this->noiseAmp = parameter_client->get_parameter<double>("noise_amplitude");
        rcpputils::assert_true(this->noiseAmp >= 0, "noiseAmp must be greater than or equal 0");

        if(parameter_client->has_parameter("noise_sigma"))
            this->noiseSigma = parameter_client->get_parameter<double>("noise_sigma");
        rcpputils::assert_true(this->noiseSigma >= 0, "noiseSigma must be greater than or equal 0");

        if(parameter_client->has_parameter("use_geo_coordinates"))
            this->useGeoCoordinates = parameter_client->get_parameter<bool>("use_geo_coordinates");
        if (this->useGeoCoordinates)
        {
            double latitude = -1;
            double longitude = -1;
            GeographicLib::Geocentric earth(
            GeographicLib::Constants::WGS84_a(),
            GeographicLib::Constants::WGS84_f());
            rcpputils::assert_true(parameter_client->has_parameter("latitude"));
            rcpputils::assert_true(parameter_client->has_parameter("longitude"));
            latitude = parameter_client->get_parameter<double>("latitude");
            longitude = parameter_client->get_parameter<double>("longitude");

            this->projection = GeographicLib::LocalCartesian(
            latitude, longitude, 0, earth);
        }

        if(parameter_client->has_parameter("salinity_unit"))
            this->salinityUnit = parameter_client->get_parameter<std::string>("salinity_unit");
        rcpputils::assert_true(!this->salinityUnit.compare(CONCENTRATION_UNIT_PPT) ||
                               !this->salinityUnit.compare(CONCENTRATION_UNIT_PPM) ||
                               !this->salinityUnit.compare(CONCENTRATION_UNIT_PSU)  , "salinityUnit must be ppt or ppm or psu");
        
        if(parameter_client->has_parameter("reference_salinity_value"))
            this->referenceSalinityValue = parameter_client->get_parameter<double>("reference_salinity_value");
        else
        {
            // If no reference is given, the sea water reference is provided
            if (this->salinityUnit.compare(CONCENTRATION_UNIT_PPT) == 0)
                this->referenceSalinityValue = 35.0;
            else if (this->salinityUnit.compare(CONCENTRATION_UNIT_PPM) == 0)
                this->referenceSalinityValue = 35000.0;
            else
                this->referenceSalinityValue = 35.0;
        }
        
        rcpputils::assert_true(this->referenceSalinityValue >= 0, "referenceSalinityValue must be greater than or equal 0");
        if(parameter_client->has_parameter("use_odom"))
        this->useOdom = parameter_client->get_parameter<bool>("use_odom");
        if(parameter_client->has_parameter("use_gps"))
        this->useGPS = parameter_client->get_parameter<bool>("use_gps");

        if (this->useOdom)
        {
            // Subscribe to the odometry topic
            this->odom_sensor_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&CPCSensor::OnOdometryUpdate, this, std::placeholders::_1));
            RCLCPP_INFO(get_logger(),"Using the odometry <nav_msgs::Odometry> as input for sensor position");
        }
        else if (this->useGPS && this->useGeoCoordinates)
        {
            // Subscribe to the GPS topic
            this->gps_sensor_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps", 10, std::bind(&CPCSensor::OnGPSUpdate, this, std::placeholders::_1));
           RCLCPP_INFO(get_logger(),"Using the GPS <sensor_msgs::NatSatFix> as input for sensor position");
        }
        else
        {
            this->useTFUpdate = true;
            // TODO: Publish a sensor frame ID
            if(parameter_client->has_parameter("sensor_frame_id"))
            this->sensorFrameID = parameter_client->get_parameter<std::string>("sensor_frame_id");
            RCLCPP_INFO(get_logger(),"Using the a frame ID %s as input for sensor position", this->sensorFrameID.c_str());
        }

        if(parameter_client->has_parameter("publish_salinity"))
        this->publishSalinity = parameter_client->get_parameter<bool>("publish_salinity");
        if (this->publishSalinity)
        {
            this->salinity_publisher_ = this->create_publisher<uuv_plume_msgs::msg::Salinity>("/salinity", 10);
        }
        
        
        // Get parameters from the other node
        auto parameters = parameter_client->get_parameters({"update_rate","gamma","gain","radius","saturation","noise_amplitude","noise_sigma","latitude","longitude",
                        "reference_salinity_value","salinity_unit","use_geo_coordinates","use_odom","use_gps","publish_salinity"});
        // Log received parameters
        for (const auto& parameter : parameters) {
            RCLCPP_INFO(get_logger(), "Received parameter %s with value %s", 
                        parameter.get_name().c_str(), parameter.value_to_string().c_str());
        }
    }

private:
    // Subscriber for particles topic
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr particles_subscription_;

    // Publisher for concentration topic
    rclcpp::Publisher<uuv_plume_msgs::msg::ParticleConcentration>::SharedPtr concentration_publisher_;

    // Subscriber for Sensor odom topic
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sensor_subscription_;

    // Subscriber for Sensor gps topic
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sensor_subscription_;

    /// \brief Output topic for salinity
    rclcpp::Publisher<uuv_plume_msgs::msg::Salinity>::SharedPtr salinity_publisher_;

    /// \brief Update the output concentration and salinity topics
    // protected: void OnSensorUpdate();

    /// \brief Update callback from the plume particles
    // protected: void OnPlumeParticlesUpdate(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    /// \brief Update the odometry callback
    // protected:  void OnOdometryUpdate(const nav_msgs::msg::Odometry::SharedPtr msg);

    /// \brief Update the GPS update callback
    // protected: void OnGPSUpdate(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    /// \brief Flag to ensure the cloud and measurement update don't coincide
    protected: bool updatingCloud = false;

    /// \brief Gamma velocity parameter for the smoothing function
    protected: double gamma;

    /// \brief Sensor gain
    protected: double gain;

    /// \brief Radius of the kernel to identify particles that will be
    /// taken into account in the concentration computation
    protected: double smoothingLength;

    /// \brief Salinity unit to be used. Options are
    /// * `ppt` (parts per thousand)
    /// * `ppm` (parts per million)
    /// * `psu` (practical salinity unit)
    protected: std::string salinityUnit = CONCENTRATION_UNIT_PPT;

    /// \brief Sensor saturation
    protected: double saturation = 1.0;

    /// \brief Flag that will allow storing the geodetic coordinates with the
    /// measurement message
    protected: bool useGeoCoordinates = false; 

    /// \brief Flag to activate publishing the simulated salinity
    protected: bool publishSalinity = false;

    /// \brief Default salinity value for the fluid e.g. sea water
    protected: double referenceSalinityValue;

    /// \brief Default salinity value for the plume
    protected: double plumeSalinityValue;

    /// \brief Set to true to avoid particle update
    protected: bool updateMeasurement;

    /// \brief Output topic's update rate
    protected: double updateRate;

    /// \brief Name of the sensor frame
    protected: std::string sensorFrameID = "base_link";

    /// \brief Flag set to true after the first set of plume particles is
    /// received
    protected: bool areParticlesInit = false;

    /// \brief Flag set if the sensor position update must be read from the
    /// vehicle's odometry input topic
    protected: bool useOdom = false;

    /// \brief Flag set if the sensor position update must be read from the
    /// vehicle's GPS topic
    protected: bool useGPS = false;

    /// \brief Flag set if the TF update wrt the sensor frame ID
    protected: bool useTFUpdate = false;

    /// \brief Measured Cartesian position
    protected: geometry_msgs::msg::Vector3 cartPos;

    /// \brief Measured geodetic position
    protected: geographic_msgs::msg::GeoPoint geoPos;

    /// \brief TF buffer instance
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;

    /// \brief TF listener pointer
    std::shared_ptr<tf2_ros::TransformListener> tfListener{nullptr};

    /// \brief Local Cartesian projection
    protected: GeographicLib::LocalCartesian projection;

    /// \brief Plume concentration message
    protected: uuv_plume_msgs::msg::ParticleConcentration concentrationMsg;

    /// \brief Salinity message
    protected: uuv_plume_msgs::msg::Salinity salinityMsg;

    /// \brief Sensor update timer
    protected: rclcpp::TimerBase::SharedPtr sensor_timer_;

    /// \brief Pseudo random number generator
    protected: std::default_random_engine rndGen;

    /// \brief Normal distribution describing the noise model
    protected: std::normal_distribution<double> noiseModel;

    /// \brief Noise amplitude
    protected: double noiseAmp = 0.0;

    /// \brief Noise standard deviation
    protected: double noiseSigma = 0.0;

};

} // namespace uuv_plume_simulator

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<uuv_plume_simulator::CPCSensor>();
    node->getParametersFromOtherNode();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
