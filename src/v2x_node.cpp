#include "autoware_v2x/v2x_node.hpp"

#include "autoware_v2x/cam_application.hpp"
#include "autoware_v2x/cpm_application.hpp"
#include "autoware_v2x/link_layer.hpp"
#include "autoware_v2x/positioning.hpp"
#include "autoware_v2x/router_context.hpp"
#include "autoware_v2x/security.hpp"
#include "autoware_v2x/time_trigger.hpp"
#include "autoware_v2x/v2x_app.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include <vanetza/asn1/cam.hpp>
#include <vanetza/asn1/cpm.hpp>

#include "autoware_adapi_v1_msgs/msg/vehicle_dimensions.hpp"
#include "autoware_adapi_v1_msgs/srv/get_vehicle_dimensions.hpp"
#include "std_msgs/msg/string.hpp"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>

#include <gps.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <regex>

namespace gn = vanetza::geonet;

using namespace vanetza;
using namespace std::chrono;

namespace v2x
{
  V2XNode::V2XNode(const rclcpp::NodeOptions &node_options) : rclcpp::Node("autoware_v2x_node", node_options) {
    using std::placeholders::_1;

    get_vehicle_dimensions_ = this->create_client<autoware_adapi_v1_msgs::srv::GetVehicleDimensions>("/api/vehicle/dimensions");
    this->getVehicleDimensions();
    timer_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&V2XNode::getVehicleDimensions, this));

    objects_sub_ = this->create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>("/perception/object_recognition/objects", 10, std::bind(&V2XNode::objectsCallback, this, _1));
    tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 10, std::bind(&V2XNode::tfCallback, this, _1));

    velocity_report_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", 10, std::bind(&V2XNode::velocityReportCallback, this, _1));
    gear_report_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::GearReport>("/vehicle/status/gear_status", 10, std::bind(&V2XNode::gearReportCallback, this, _1));
    steering_report_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", 10, std::bind(&V2XNode::steeringReportCallback, this, _1));

    cpm_objects_pub_ = create_publisher<autoware_auto_perception_msgs::msg::PredictedObjects>("/v2x/cpm/objects", rclcpp::QoS{10});
    // cpm_sender_pub_ = create_publisher<autoware_auto_perception_msgs::msg::PredictedObjects>("/v2x/cpm/sender", rclcpp::QoS{10});

    cam_rec_pub_ = create_publisher<etsi_its_cam_ts_msgs::msg::CAM>("/v2x/cam_ts/received", rclcpp::QoS{10});
    cam_sent_pub_ = create_publisher<etsi_its_cam_ts_msgs::msg::CAM>("/v2x/cam_ts/sent", rclcpp::QoS{10});

    // Declare Parameters
    this->declare_parameter<std::string>("link_layer");
    this->declare_parameter<std::string>("target_device");
    this->declare_parameter<std::string>("gps_method", "ros");
    this->declare_parameter<std::string>("gps_provider", "/sensing/gnss/gps");
    this->declare_parameter<bool>("is_sender");
    this->declare_parameter<bool>("cam_enabled");
    this->declare_parameter<bool>("cpm_enabled");
    this->declare_parameter<std::string>("security", "none");
    this->declare_parameter<std::string>("certificate", "");
    this->declare_parameter<std::string>("certificate-key", "");
    this->declare_parameter<std::vector<std::string>>("certificate-chain", std::vector<std::string>());

    std::string gps_method, gps_provider;
    this->get_parameter("gps_method", gps_method);
    this->get_parameter("gps_provider", gps_provider);
    if (gps_method == "gpsd") {
      std::string gpsd_host, gpsd_port;
      std::istringstream iss(gps_provider);
      std::getline(iss, gpsd_host, ':');
      std::getline(iss, gpsd_port, ':');
      if (gpsd_host.empty() || gpsd_port.empty()) {
        throw std::runtime_error("Invalid gps_provider value for gpsd method");
      }
      boost::thread gps(boost::bind(&V2XNode::runGpsClient, this, gpsd_host, gpsd_port));
      RCLCPP_INFO(get_logger(), "GPS Client Launched on %s:%s", gpsd_host.c_str(), gpsd_port.c_str());
    } else {
      if (const std::regex ros_topic_regex("^(\\/|[a-zA-Z])[\\w\\/]*[^\\/]$"); !std::regex_match(gps_provider, ros_topic_regex)) {
        throw std::runtime_error("Invalid gps_provider value for ros topic method");
      }
      gps_sub_ = this->create_subscription<gps_msgs::msg::GPSFix>(gps_provider, 10, std::bind(&V2XNode::gpsFixCallback, this, _1));
    }

    // Launch V2XApp in a new thread
    app = new V2XApp(this);
    boost::thread v2xApp(boost::bind(&V2XApp::start, app));

    RCLCPP_INFO(get_logger(), "V2X Node Launched");

    // Make latency_log file from current timestamp
    time_t t = time(nullptr);
    const tm* lt = localtime(&t);
    std::stringstream s;
    s << "20" << lt->tm_year-100 <<"-" << lt->tm_mon+1 << "-" << lt->tm_mday << "_" << lt->tm_hour << ":" << lt->tm_min << ":" << lt->tm_sec;
    std::string timestamp = s.str();
    char cur_dir[1024];
    getcwd(cur_dir, 1024);
    std::string latency_log_filename = std::string(cur_dir) + "/latency_logs/latency_log_file_" + timestamp + ".csv";
    latency_log_file.open(latency_log_filename, std::ios::out);
  }

  void V2XNode::publishCpmSenderObject(double x_mgrs, double y_mgrs, double orientation) {
    autoware_auto_perception_msgs::msg::PredictedObjects cpm_sender_object_msg;
    std_msgs::msg::Header header;
    rclcpp::Time current_time = this->now();
    cpm_sender_object_msg.header.frame_id = "map";
    cpm_sender_object_msg.header.stamp = current_time;

    autoware_auto_perception_msgs::msg::PredictedObject object;
    autoware_auto_perception_msgs::msg::ObjectClassification classification;
    autoware_auto_perception_msgs::msg::Shape shape;
    autoware_auto_perception_msgs::msg::PredictedObjectKinematics kinematics;

    classification.label = autoware_auto_perception_msgs::msg::ObjectClassification::CAR;
    classification.probability = 0.99;

    shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
    shape.dimensions.x = 5.0;
    shape.dimensions.y = 2.0;
    shape.dimensions.z = 1.7;

    kinematics.initial_pose_with_covariance.pose.position.x = x_mgrs;
    kinematics.initial_pose_with_covariance.pose.position.y = y_mgrs;
    kinematics.initial_pose_with_covariance.pose.position.z = 0.1;

    tf2::Quaternion quat;
    quat.setRPY(0, 0, orientation);

    kinematics.initial_pose_with_covariance.pose.orientation.x = quat.x();
    kinematics.initial_pose_with_covariance.pose.orientation.y = quat.y();
    kinematics.initial_pose_with_covariance.pose.orientation.z = quat.z();
    kinematics.initial_pose_with_covariance.pose.orientation.w = quat.w();

    object.classification.emplace_back(classification);
    object.shape = shape;
    object.kinematics = kinematics;

    cpm_sender_object_msg.objects.push_back(object);

    // publisher_v2x_cpm_sender_->publish(cpm_sender_object_msg);

  }

  void V2XNode::publishObjects(std::vector<CpmApplication::Object> *objectsStack, int cpm_num) {
    autoware_auto_perception_msgs::msg::PredictedObjects output_dynamic_object_msg;
    std_msgs::msg::Header header;
    rclcpp::Time current_time = this->now();
    output_dynamic_object_msg.header.frame_id = "map";
    output_dynamic_object_msg.header.stamp = current_time;

    for (CpmApplication::Object obj : *objectsStack) {
      autoware_auto_perception_msgs::msg::PredictedObject object;
      autoware_auto_perception_msgs::msg::ObjectClassification classification;
      autoware_auto_perception_msgs::msg::Shape shape;
      autoware_auto_perception_msgs::msg::PredictedObjectKinematics kinematics;

      classification.label = autoware_auto_perception_msgs::msg::ObjectClassification::CAR;
      classification.probability = 0.99;

      shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
      shape.dimensions.x = obj.shape_x / 10.0;
      shape.dimensions.y = obj.shape_y / 10.0;
      shape.dimensions.z = obj.shape_z / 10.0;

      kinematics.initial_pose_with_covariance.pose.position.x = obj.position_x;
      kinematics.initial_pose_with_covariance.pose.position.y = obj.position_y;
      kinematics.initial_pose_with_covariance.pose.position.z = 0.1;

      kinematics.initial_pose_with_covariance.pose.orientation.x = obj.orientation_x;
      kinematics.initial_pose_with_covariance.pose.orientation.y = obj.orientation_y;
      kinematics.initial_pose_with_covariance.pose.orientation.z = obj.orientation_z;
      kinematics.initial_pose_with_covariance.pose.orientation.w = obj.orientation_w;

      object.classification.emplace_back(classification);
      object.shape = shape;
      object.kinematics = kinematics;

      std::mt19937 gen(std::random_device{}());
      std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
      std::generate(object.object_id.uuid.begin(), object.object_id.uuid.end(), bit_eng);

      output_dynamic_object_msg.objects.push_back(object);
    }

    std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds> (
      std::chrono::system_clock::now().time_since_epoch()
    );
    latency_log_file << "T_publish," << cpm_num << "," << ms.count() << std::endl;

    cpm_objects_pub_->publish(output_dynamic_object_msg);
  }

  void V2XNode::publishReceivedCam(etsi_its_cam_ts_msgs::msg::CAM &msg) {
    RCLCPP_INFO(get_logger(), "Publishing received CAM to ROS network");
    cam_rec_pub_->publish(msg);
  }

  void V2XNode::publishSentCam(etsi_its_cam_ts_msgs::msg::CAM &msg) {
    RCLCPP_INFO(get_logger(), "Publishing sent CAM to ROS network");
    cam_sent_pub_->publish(msg);
  }

  void V2XNode::gpsFixCallback(const gps_msgs::msg::GPSFix::ConstSharedPtr msg) {
    if (!positioning_received_) positioning_received_ = true;

    gps_data_.timestamp = std::chrono::system_clock::now();
    gps_data_.latitude = msg->latitude;
    gps_data_.longitude = msg->longitude;
    gps_data_.altitude = msg->altitude;
  }

  void V2XNode::objectsCallback(const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg) {
    rclcpp::Time msg_time = msg->header.stamp; // timestamp included in the Autoware Perception Msg.

    std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds> (
      std::chrono::system_clock::now().time_since_epoch()
    );
    latency_log_file << "T_rosmsg,," << ms.count() << std::endl;

    app->objectsCallback(msg);
  }

  void V2XNode::tfCallback(const tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {
    app->tfCallback(msg);
  }

  bool V2XNode::positioningReceived() {
    return positioning_received_;
  }

  void V2XNode::velocityReportCallback(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr msg) {
    app->velocityReportCallback(msg);
  }

  void V2XNode::gearReportCallback(const autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr msg) {
    app->gearReportCallback(msg);
  }

  void V2XNode::steeringReportCallback(const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr msg) {
    app->steeringReportCallback(msg);
  }

  void V2XNode::getVehicleDimensions() {
    if (!get_vehicle_dimensions_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "[V2XNode::getVehicleDimensions] Service /api/vehicle/dimensions is not yet available");
      return;
    }

    RCLCPP_INFO(get_logger(), "[V2XNode::getVehicleDimensions] Sending request to /api/vehicle/dimensions");
    auto request = std::make_shared<autoware_adapi_v1_msgs::srv::GetVehicleDimensions::Request>();
    auto future_result = get_vehicle_dimensions_->async_send_request(request, [this](rclcpp::Client<autoware_adapi_v1_msgs::srv::GetVehicleDimensions>::SharedFuture future) {
      try {
        auto response = future.get();
        RCLCPP_INFO(get_logger(), "[V2XNode::getVehicleDimensions] Received response from /api/vehicle/dimensions");
        try {
          auto dimensions = response->dimensions;
          float length = dimensions.front_overhang + dimensions.wheel_base + dimensions.rear_overhang;
          float width = dimensions.left_overhang + dimensions.wheel_tread + dimensions.right_overhang;
          RCLCPP_INFO(get_logger(), "[V2XNode::getVehicleDimensions] Received vehicle dimensions: height=%f, length=%f, width=%f", dimensions.height, length, width);
          if (app->cam) {
            app->cam->setVehicleDimensions(dimensions);
            timer_->cancel();
          }
        } catch (const std::exception &e) {
          RCLCPP_ERROR(get_logger(), "[V2XNode::getVehicleDimensions] Invalid response of /api/vehicle/dimensions: %s", e.what());
        }
      }
      catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "[V2XNode::getVehicleDimensions] Service call of /api/vehicle/dimensions failed: %s", e.what());
      }
    });
  }

  void V2XNode::getGpsData(double& latitude, double& longitude, double& altitude) {
    latitude = gps_data_.latitude;
    longitude = gps_data_.longitude;
    altitude = gps_data_.altitude;
  }

  void V2XNode::runGpsClient(const std::string host, const std::string port) {
    gps_data_t gps_data;

    if (0 != gps_open(host.c_str(), port.c_str(), &gps_data)) {
      throw std::runtime_error("Failed to open gpsd on " + host + ":" + port);
    }

    (void)gps_stream(&gps_data, WATCH_ENABLE | WATCH_JSON, NULL);

    while (gps_waiting(&gps_data, 5000000)) {
      if (-1 == gps_read(&gps_data, NULL, 0)) {
        throw std::runtime_error("Failed to read gpsd on " + host + ":" + port);
      }
      if (MODE_SET != (MODE_SET & gps_data.set)) {
          continue;
      }
      if (!positioning_received_) positioning_received_ = true;
      gps_data_.timestamp = std::chrono::system_clock::now();
      gps_data_.latitude = gps_data.fix.latitude;
      gps_data_.longitude = gps_data.fix.longitude;
      gps_data_.altitude = gps_data.fix.altitude;
    }

    (void)gps_stream(&gps_data, WATCH_DISABLE, NULL);
    (void)gps_close(&gps_data);
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(v2x::V2XNode)
