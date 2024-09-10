#include "autoware_v2x/v2x_app.hpp"
#include "autoware_v2x/v2x_node.hpp"
#include "autoware_v2x/time_trigger.hpp"
#include "autoware_v2x/router_context.hpp"
#include "autoware_v2x/positioning.hpp"
#include "autoware_v2x/security.hpp"
#include "autoware_v2x/link_layer.hpp"
#include "autoware_v2x/cpm_application.hpp"
#include "autoware_v2x/cam_application.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <vanetza/asn1/cpm.hpp>
#include <vanetza/asn1/cam.hpp>
#include <sstream>
#include <memory>
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/MGRS.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace gn = vanetza::geonet;
namespace po = boost::program_options;

using namespace vanetza;
using namespace std::chrono;

namespace v2x
{
  V2XApp::V2XApp(V2XNode *node) :
    node_(node),
    tf_received_(false),
    tf_interval_(0),
    vehicle_dimensions_set_(false),
    cp_started_(false),
    cam_started_(false)
  {
  }

  void V2XApp::objectsCallback(const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg) {
    // RCLCPP_INFO(node_->get_logger(), "V2XApp: msg received");
    if (!tf_received_) {
      RCLCPP_WARN(node_->get_logger(), "[V2XApp::objectsCallback] tf not received yet");
    }
    if (tf_received_ && cp_started_) {
      cp->updateObjectsList(msg);
    }
  }

  void V2XApp::velocityReportCallback(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr msg) {
    if (cam_started_)
      cam->updateVelocityReport(msg);
  }

  void V2XApp::gearReportCallback(const autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr msg) {
    if (cam_started_)
      cam->updateGearReport(msg);
  }

  void V2XApp::steeringReportCallback(const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr msg) {
    if (cam_started_)
      cam->updateSteeringReport(msg);
  }


  void V2XApp::tfCallback(const tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {

    tf_received_ = true;

    double x = msg->transforms[0].transform.translation.x;
    double y = msg->transforms[0].transform.translation.y;
    double z = msg->transforms[0].transform.translation.z;

    long timestamp_sec = msg->transforms[0].header.stamp.sec; // seconds
    long timestamp_nsec = msg->transforms[0].header.stamp.nanosec; // nanoseconds
    timestamp_sec -= 1072915200; // convert to etsi-epoch
    long timestamp_msec = timestamp_sec * 1000 + timestamp_nsec / 1000000;
    int gdt = timestamp_msec % 65536;

    double rot_x = msg->transforms[0].transform.rotation.x;
    double rot_y = msg->transforms[0].transform.rotation.y;
    double rot_z = msg->transforms[0].transform.rotation.z;
    double rot_w = msg->transforms[0].transform.rotation.w;

    // Convert the quarternion to euler (yaw, pitch, roll)
    tf2::Quaternion quat(rot_x, rot_y, rot_z, rot_w);
    tf2::Matrix3x3 matrix(quat);
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);

    int zone = 54;
    int grid_num_x = 4;
    int grid_num_y = 39;
    int grid_size = 100000;
    bool northp = true;
    double lat, lon;

    // Reverse projection from UTM to geographic.
    GeographicLib::UTMUPS::Reverse(
      zone,
      northp,
      grid_num_x * grid_size + x,
      grid_num_y * grid_size + y,
      lat,
      lon
    );

    if (cp && cp_started_) {
      cp->updateMGRS(&x, &y);
      cp->updateRP(&lat, &lon, &z);
      cp->updateHeading(&yaw);
      cp->updateGenerationTime(&gdt, &timestamp_msec);
    }

    if (cam && cp_started_) {
      cam->updateMGRS(&x, &y);
      cam->updateRP(&lat, &lon, &z);
      cam->updateHeading(&yaw);
    }
  }

  void V2XApp::start() {
    RCLCPP_INFO(node_->get_logger(), "V2X App Launched");

    boost::asio::io_service io_service;
    TimeTrigger trigger(io_service);

    std::string link_layer_name;
    node_->get_parameter("link_layer", link_layer_name);
    RCLCPP_INFO(node_->get_logger(), "Link Layer: %s", link_layer_name.c_str());

    std::string network_interface;
    node_->get_parameter("network_interface", network_interface);
    RCLCPP_INFO(node_->get_logger(), "Network Interface: %s", network_interface.c_str());

    EthernetDevice device(network_interface.c_str());
    vanetza::MacAddress mac_address = device.address();

    std::stringstream sout;
    sout << mac_address;
    RCLCPP_INFO(node_->get_logger(), "MAC Address: '%s'", sout.str().c_str());

    // Geonetworking Management Infirmation Base (MIB) defines the GN protocol constants.
    gn::MIB mib;
    mib.itsGnLocalGnAddr.mid(mac_address);
    mib.itsGnLocalGnAddr.is_manually_configured(true);
    mib.itsGnLocalAddrConfMethod = geonet::AddrConfMethod::Managed;
    mib.itsGnSecurity = false;
    mib.itsGnProtocolVersion = 1;

    std::string cube_ip;
    node_->get_parameter("cube_ip", cube_ip);
    RCLCPP_INFO(node_->get_logger(), "CubeEVK IP: %s", cube_ip.c_str());

    // Create raw socket on device and LinkLayer object
    auto link_layer =  create_link_layer(io_service, device, link_layer_name, cube_ip);
    auto positioning = create_position_provider(io_service, trigger.runtime());

    po::variables_map security_options;
    std::string entity;
    std::string certificate;
    std::string certificate_key;
    std::vector<std::string> certificate_chain;
    // Grab the values from the node parameters
    node_->get_parameter("security", entity);
    security_options.insert(std::make_pair("security", po::variable_value(entity, false)));
    if (node_->has_parameter("certificate")) {
      node_->get_parameter("certificate", certificate);
      security_options.insert(std::make_pair("certificate", po::variable_value(certificate, false)));
    }
    if (node_->has_parameter("certificate-key")) {
      node_->get_parameter("certificate-key", certificate_key);
      security_options.insert(std::make_pair("certificate-key", po::variable_value(certificate_key, false)));
    }
    if (node_->has_parameter("certificate-chain")) {
      node_->get_parameter("certificate-chain", certificate_chain);
      security_options.insert(std::make_pair("certificate-chain", po::variable_value(certificate_chain, false)));
    }
    auto security = create_security_entity(security_options, trigger.runtime(), *positioning);

    RCLCPP_INFO(node_->get_logger(), "Security layer: %s", entity == "certs" ? "Certificates" : "None");

    RouterContext context(mib, trigger, *positioning, security.get());

    context.set_link_layer(link_layer.get());

    bool is_sender;
    node_->get_parameter("is_sender", is_sender);
    cp = new CpmApplication(node_, trigger.runtime(), is_sender);
    cam = new CamApplication(node_, trigger.runtime(), is_sender);

    context.enable(cp);
    context.enable(cam);

    cp_started_ = true;
    cam_started_ = true;

    io_service.run();
  }
}
