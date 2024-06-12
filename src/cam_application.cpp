#include "autoware_v2x/cam_application.hpp"
#include "autoware_v2x/positioning.hpp"
#include "autoware_v2x/security.hpp"
#include "autoware_v2x/link_layer.hpp"
#include "autoware_v2x/v2x_node.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "rclcpp/rclcpp.hpp"
#include <vanetza/btp/ports.hpp>
#include <vanetza/asn1/cam.hpp>
#include <vanetza/asn1/packet_visitor.hpp>
#include <chrono>
#include <functional>
#include <iostream>
#include <sstream>
#include <exception>
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/MGRS.hpp>
#include <string>

#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>

#include <sqlite3.h>

#define _USE_MATH_DEFINES
#include <math.h>

using namespace vanetza;
using namespace std::chrono;

namespace v2x
{
  CamApplication::CamApplication(V2XNode * node, Runtime & rt, bool is_sender)
  : node_(node),
    runtime_(rt),
    vehicleDimensions_(),
    ego_(),
    velocityReport_(),
    gearReport_(),
    generationTime_(0),
    updating_velocity_report_(false),
    updating_gear_report_(false),
    sending_(false),
    is_sender_(is_sender),
    reflect_packet_(false),
    objectConfidenceThreshold_(0.0),
    include_all_persons_and_animals_(false),
    cam_num_(0),
    received_cam_num_(0),
    use_dynamic_generation_rules_(false)
  {
    RCLCPP_INFO(node_->get_logger(), "CamApplication started. is_sender: %d", is_sender_);
    set_interval(milliseconds(100));
    //createTables();
  }

  void CamApplication::set_interval(Clock::duration interval) {
    cam_interval_ = interval;
    runtime_.cancel(this);
    schedule_timer();
  }

  void CamApplication::schedule_timer() {
    runtime_.schedule(cam_interval_, std::bind(&CamApplication::on_timer, this, std::placeholders::_1));
  }

  void CamApplication::on_timer(vanetza::Clock::time_point) {
    schedule_timer();
    send();
  }
  
  CamApplication::PortType CamApplication::port() {
    return btp::ports::CAM;
  }
  
  //std::string CamApplication::uuidToHexString(const unique_identifier_msgs::msg::UUID &id) {
  //  std::stringstream ss;
  //  for (int i = 0; i < id.uuid.size(); i++) {
  //    ss << std::hex << std::setfill('0') << std::setw(2) << (int)id.uuid[i];
  //  }
  //  return ss.str();
  //}
  
  void CamApplication::indicate(const Application::DataIndication &, Application::UpPacketPtr)
  {
    // TODO: implement
  }

  void CamApplication::updateMGRS(double *x, double *y) {
    ego_.mgrs_x = *x;
    ego_.mgrs_y = *y;
  }
  
  void CamApplication::updateRP(double *lat, double *lon, double *altitude) {
    ego_.latitude = *lat;
    ego_.longitude = *lon;
    ego_.altitude = *altitude;
  }
  
  void CamApplication::updateGenerationTime(int *gdt, long *gdt_timestamp) {
    generationTime_ = *gdt;
    gdt_timestamp_ = *gdt_timestamp; // ETSI-epoch milliseconds timestamp
  }

  void CamApplication::updateHeading(double *yaw) {
    ego_.heading = *yaw;
  }

  //void CamApplication::setAllObjectsOfPersonsAnimalsToSend(const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg) {
  //  if (msg->objects.size() > 0) {
  //    for (autoware_auto_perception_msgs::msg::PredictedObject obj : msg->objects) {
  //      std::string object_uuid = uuidToHexString(obj.object_id);
  //      auto found_object = std::find_if(objectsList.begin(), objectsList.end(), [&](auto const &e) {
  //        return !strcmp(e.uuid.c_str(), object_uuid.c_str());
  //      });
  //      
  //      if (found_object == objectsList.end()) {
  //        
  //      } else {
  //        found_object->to_send = true;
  //      }
  //    }
  //  }
  //}
  
  void CamApplication::getVehicleDimensions(const autoware_adapi_v1_msgs::msg::VehicleDimensions msg) {
    vehicleDimensions_.wheel_radius = msg.wheel_radius;
    vehicleDimensions_.wheel_width = msg.wheel_width;
    vehicleDimensions_.wheel_base = msg.wheel_base;
    vehicleDimensions_.wheel_tread = msg.wheel_tread;
    vehicleDimensions_.front_overhang = msg.front_overhang;
    vehicleDimensions_.rear_overhang = msg.rear_overhang;
    vehicleDimensions_.left_overhang = msg.left_overhang;
    vehicleDimensions_.right_overhang = msg.right_overhang;
    vehicleDimensions_.height = msg.height;
  }
  
  void CamApplication::updateVelocityReport(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr msg) {
    if (updating_velocity_report_) {
      RCLCPP_WARN(node_->get_logger(), "[CamApplication::updatevelocityReport_] already updating velocity report");
      return;
    }
    
    updating_velocity_report_ = true;
    
    rclcpp::Time msg_stamp(msg->header.stamp.sec, msg->header.stamp.nanosec);
    float dt = (msg_stamp - velocityReport_.stamp).seconds();
    if (dt == 0) {
      RCLCPP_WARN(node_->get_logger(), "[CamApplication::updatevelocityReport_] deltaTime is 0");
      return;
    }
    float longitudinal_acceleration = (msg->longitudinal_velocity - velocityReport_.longitudinal_velocity) / dt;
    
    velocityReport_.stamp = msg->header.stamp;
    velocityReport_.heading_rate = msg->heading_rate;
    velocityReport_.lateral_velocity = msg->lateral_velocity;
    velocityReport_.longitudinal_velocity = msg->longitudinal_velocity;
    velocityReport_.longitudinal_acceleration = longitudinal_acceleration;
  }

  void CamApplication::updateGearReport(const autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr msg) {
    if (updating_gear_report_) {
      RCLCPP_WARN(node_->get_logger(), "[CamApplication::updatevelocityReport_] already updating gear report");
      return;
    }

    updating_gear_report_ = true;

    gearReport_ = msg->report;
  }
  
  void CamApplication::send() {
    if (!is_sender_) return;
    
    if (sending_) {
      RCLCPP_WARN(node_->get_logger(), "[CamApplication::send] already sending");
      return;
    }
    
    sending_ = true;

    //printObjectsList(cam_num_);
    
    RCLCPP_INFO(node_->get_logger(), "[CamApplication::send] cam_num: %d", cam_num_);
    // RCLCPP_INFO(node_->get_logger(), "[CamApplication::send] sending CAM");
    
    vanetza::asn1::Cam message;
    
    ItsPduHeader_t &header = message->header;
    header.protocolVersion = 2;
    header.messageID = ItsPduHeader__messageID_cam;
    header.stationID = 0;
    
    CoopAwareness_t &cam = message->cam;

    //asn_long2INTEGER(&cam.generationDeltaTime, (long) gdt_timestamp_);
    
    BasicContainer_t &basic_container = cam.camParameters.basicContainer;
    basic_container.stationType = StationType_passengerCar;
    basic_container.referencePosition.latitude = ego_.latitude * 1e7;
    basic_container.referencePosition.longitude = ego_.longitude * 1e7;
    basic_container.referencePosition.altitude.altitudeValue = ego_.altitude;
    
    // UNAVAILABLE VALUES FOR TESTING
    basic_container.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
    basic_container.referencePosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
    basic_container.referencePosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
    basic_container.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
    // ------------------------------

    BasicVehicleContainerHighFrequency_t &bvc = cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
    cam.camParameters.highFrequencyContainer.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
    
    int heading = std::lround(((-ego_.heading * 180.0 / M_PI) + 90.0) * 10.0);
    if (heading < 0) heading += 3600;
    bvc.heading.headingValue = heading;
    
    float heading_rate = velocityReport_.heading_rate;
    float lateral_velocity = velocityReport_.lateral_velocity;
    float longitudinal_velocity = velocityReport_.longitudinal_velocity;
    
    float speed = std::sqrt(std::pow(longitudinal_velocity, 2) + std::pow(lateral_velocity, 2));
    bvc.speed.speedValue = std::lround(speed * 100);

    if ((gearReport_ >= 2 && gearReport_ <= 19) || gearReport_ == 21 || gearReport_ == 22)
      bvc.driveDirection = DriveDirection_forward;
    else if (gearReport_ == 20 || gearReport_ == 21)
      bvc.driveDirection = DriveDirection_backward;
    else
      bvc.driveDirection = DriveDirection_unavailable;
    
    float vehicleLength = vehicleDimensions_.front_overhang + vehicleDimensions_.wheel_base + vehicleDimensions_.rear_overhang;
    if (vehicleLength <= 0 || vehicleLength >= 1023)
      bvc.vehicleLength.vehicleLengthValue = VehicleLengthValue_unavailable;
    else
      bvc.vehicleLength.vehicleLengthValue = vehicleLength;
    
    float vehicleWidth = vehicleDimensions_.left_overhang + vehicleDimensions_.wheel_tread + vehicleDimensions_.right_overhang;
    if (vehicleWidth <= 0 || vehicleWidth >= 62)
      bvc.vehicleWidth = VehicleWidth_unavailable;
    else
      bvc.vehicleWidth = vehicleWidth;

    //if (longitudinal_acceleration > 160 || longitudinal_acceleration < -160) {
    //  RCLCPP_WARN(node_->get_logger(), "[CamApplication::send] Longitudinal acceleration out of bounds: %d", longitudinal_acceleration);
    //  bvc.longitudinalAcceleration.longitudinalAccelerationValue = LongitudinalAccelerationValue_unavailable;
    //} else {
    //  bvc.longitudinalAcceleration.longitudinalAccelerationValue = longitudinal_acceleration;
    //}
    
    bvc.longitudinalAcceleration.longitudinalAccelerationValue = LongitudinalAccelerationValue_unavailable;
    
    if (longitudinal_velocity != 0)
      bvc.curvature.curvatureValue = std::lround(lateral_velocity / std::pow(longitudinal_velocity, 2) * 100);
    else
      bvc.curvature.curvatureValue = std::numeric_limits<long>::infinity();
    bvc.curvatureCalculationMode = CurvatureCalculationMode_yawRateNotUsed;

    bvc.yawRate.yawRateValue = std::lround(heading_rate * 100);

    // UNAVAILABLE VALUES FOR TESTING
    bvc.heading.headingConfidence = HeadingConfidence_unavailable;
    bvc.speed.speedConfidence = SpeedConfidence_unavailable;
    bvc.vehicleLength.vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication_unavailable;
    bvc.longitudinalAcceleration.longitudinalAccelerationConfidence = AccelerationConfidence_unavailable;
    bvc.curvature.curvatureConfidence = CurvatureConfidence_unavailable;
    bvc.yawRate.yawRateConfidence = YawRateConfidence_unavailable;
    // ------------------------------
    
    RCLCPP_INFO(node_->get_logger(), "[CamApplication::send] Sending CAM");
    //insertCamToCamTable(message, (char *) "cam_sent");
    std::unique_ptr<geonet::DownPacket> payload{new geonet::DownPacket()};
    payload->layer(OsiLayer::Application) = std::move(message);
    
    Application::DataRequest request;
    request.its_aid = aid::CP;
    request.transport_type = geonet::TransportType::SHB;
    request.communication_profile = geonet::CommunicationProfile::ITS_G5;
    
    Application::DataConfirm confirm = Application::request(request, std::move(payload), node_);
    
    if (!confirm.accepted()) {
      throw std::runtime_error("[CamApplication::send] CAM application data request failed");
    }

    sending_ = false;

    std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds> (
      std::chrono::system_clock::now().time_since_epoch()
    );
    node_->latency_log_file << "T_depart," << cam_num_ << "," << ms.count() << std::endl;

    ++cam_num_;
  }

}