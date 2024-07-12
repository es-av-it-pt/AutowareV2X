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
#include <random>
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
    positionConfidenceEllipse_(),
    velocityReport_(),
    vehicleStatus_(),
    generationDeltaTime_(0),
    updating_velocity_report_(false),
    updating_vehicle_status_(false),
    sending_(false),
    is_sender_(is_sender),
    reflect_packet_(false),
    objectConfidenceThreshold_(0.0),
    cam_num_(0),
    received_cam_num_(0),
    use_dynamic_generation_rules_(false)
  {
    RCLCPP_INFO(node_->get_logger(), "CamApplication started. is_sender: %d", is_sender_);
    set_interval(milliseconds(100));
    //createTables();

    // Generate ID for this station
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<unsigned long> dis(0, 4294967295);
    stationId_ = dis(gen);
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

    positionConfidenceEllipse_.x.insert(*lat);
    positionConfidenceEllipse_.y.insert(*lon);
  }

  void CamApplication::updateGenerationDeltaTime(int *gdt, long *gdt_timestamp) {
    generationDeltaTime_ = *gdt;
    gdt_timestamp_ = *gdt_timestamp; // ETSI-epoch milliseconds timestamp
  }

  void CamApplication::updateHeading(double *yaw) {
    ego_.heading = *yaw;
  }

  void CamApplication::setVehicleDimensions(const autoware_adapi_v1_msgs::msg::VehicleDimensions &msg) {
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
      return;
    }

    updating_velocity_report_ = true;

    rclcpp::Time msg_stamp(msg->header.stamp.sec, msg->header.stamp.nanosec);
    float dt = msg_stamp.seconds() - velocityReport_.stamp.seconds();
    if (dt == 0) {
      RCLCPP_WARN(node_->get_logger(), "[CamApplication::updateVelocityReport] deltaTime is 0");
      return;
    }
    float longitudinal_acceleration = (msg->longitudinal_velocity - velocityReport_.longitudinal_velocity) / dt;

    velocityReport_.stamp = msg->header.stamp;
    velocityReport_.heading_rate = msg->heading_rate;
    velocityReport_.lateral_velocity = msg->lateral_velocity;
    velocityReport_.longitudinal_velocity = msg->longitudinal_velocity;
    velocityReport_.longitudinal_acceleration = longitudinal_acceleration;

    updating_velocity_report_ = false;
  }

  void CamApplication::updateVehicleStatus(const autoware_adapi_v1_msgs::msg::VehicleStatus::ConstSharedPtr msg) {
    if (updating_vehicle_status_) {
      return;
    }

    updating_vehicle_status_ = true;

    vehicleStatus_.gear = msg->gear.status;
    vehicleStatus_.steering_tire_angle = msg->steering_tire_angle;

    updating_vehicle_status_ = false;
  }

  void CamApplication::send() {
    if (!is_sender_) return;

    if (sending_) {
      RCLCPP_WARN(node_->get_logger(), "[CamApplication::send] already sending");
      return;
    }

    sending_ = true;

    RCLCPP_INFO(node_->get_logger(), "[CamApplication::send] cam_num: %d", cam_num_);

    vanetza::asn1::Cam message;

    ItsPduHeader_t &header = message->header;
    header.protocolVersion = 2;
    header.messageID = ItsPduHeader__messageID_cam;
    header.stationID = stationId_;

    CoopAwareness_t &cam = message->cam;

    RCLCPP_INFO(node_->get_logger(), "[CpmApplication::send] %ld", gdt_timestamp_);
    cam.generationDeltaTime = generationDeltaTime_;

    BasicContainer_t &basic_container = cam.camParameters.basicContainer;
    basic_container.stationType = StationType_passengerCar;
    float latitude = ego_.latitude * 1e7;
    float longitude = ego_.longitude * 1e7;
    float altitude = ego_.altitude * 100;

    if (-900000000 <= latitude && latitude <= 900000000) basic_container.referencePosition.latitude = latitude;
    else basic_container.referencePosition.latitude = Latitude_unavailable;
    if (-1800000000 <= longitude && longitude <= 1800000000) basic_container.referencePosition.longitude = longitude;
    else basic_container.referencePosition.longitude = Longitude_unavailable;
    if (-100000 <= altitude && altitude <= 800000) basic_container.referencePosition.altitude.altitudeValue = altitude;
    else basic_container.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;

    // Articles consulted for the positionConficenceEllipse
    // https://users.cs.utah.edu/~tch/CS4640F2019/resources/A%20geometric%20interpretation%20of%20the%20covariance%20matrix.pdf
    // https://users.cs.utah.edu/~tch/CS6640F2020/resources/How%20to%20draw%20a%20covariance%20error%20ellipse.pdf
    if (positionConfidenceEllipse_.x.getSize() == positionConfidenceEllipse_.y.getSize()) {
      double xx_sum = 0;
      double yy_sum = 0;
      double xy_sum = 0;
      for (double x : positionConfidenceEllipse_.x)
        xx_sum += std::pow(x - positionConfidenceEllipse_.x.getMean(), 2);
      for (double y : positionConfidenceEllipse_.y)
        yy_sum += std::pow(y - positionConfidenceEllipse_.y.getMean(), 2);
      for (int i = 0; i < positionConfidenceEllipse_.x.getSize(); i++)
        xy_sum += (positionConfidenceEllipse_.x[i] - positionConfidenceEllipse_.x.getMean()) *
                  (positionConfidenceEllipse_.y[i] - positionConfidenceEllipse_.y.getMean());

      double sigma_xx = xx_sum / (positionConfidenceEllipse_.x.getSize() - 1);
      double sigma_yy = yy_sum / (positionConfidenceEllipse_.y.getSize() - 1);
      double sigma_xy = xy_sum / (positionConfidenceEllipse_.x.getSize() - 1);

      double lambda1 = (sigma_xx + sigma_yy) - std::sqrt(std::pow(sigma_xx + sigma_yy, 2) - 4 * (sigma_xx * sigma_yy - sigma_xy * sigma_xy)) / 2;
      double lambda2 = (sigma_xx + sigma_yy) + std::sqrt(std::pow(sigma_xx + sigma_yy, 2) - 4 * (sigma_xx * sigma_yy - sigma_xy * sigma_xy)) / 2;

      double lambda_max = std::max(lambda1, lambda2);
      double lambda_min = std::min(lambda1, lambda2);

      // For 95% confidence level, must use 2.4477
      double majorConfidence = std::lround(2.4477 * std::sqrt(lambda_max));
      double minorConfidence = std::lround(2.4477 * std::sqrt(lambda_min));
      double majorOrientation = - (sigma_xy != 0
                                   ? std::lround(std::atan(- (sigma_xx - lambda_max) / sigma_xy) * 180 / M_PI)
                                   : sigma_xx != 0 ? 0 : -90) * 10;
      if (majorOrientation < 0) majorOrientation += 3600;

      if (0 <= majorConfidence && majorConfidence <= 4094) basic_container.referencePosition.positionConfidenceEllipse.semiMajorConfidence = majorConfidence;
      else basic_container.referencePosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
      if (0 <= minorConfidence && minorConfidence <= 4094) basic_container.referencePosition.positionConfidenceEllipse.semiMinorConfidence = minorConfidence;
      else basic_container.referencePosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
      if (0 <= majorOrientation && majorOrientation <= 3600) basic_container.referencePosition.positionConfidenceEllipse.semiMajorOrientation = majorOrientation;
      else basic_container.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
    } else {
      basic_container.referencePosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
      basic_container.referencePosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
      basic_container.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
    }

    BasicVehicleContainerHighFrequency_t &bvc = cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
    cam.camParameters.highFrequencyContainer.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;

    int heading = std::lround(((-ego_.heading * 180.0 / M_PI) + 90.0) * 10.0);
    if (heading < 0) heading += 3600;
    if (0 <= heading && heading <= 3600) bvc.heading.headingValue = heading;
    else bvc.heading.headingValue = HeadingValue_unavailable;

    float heading_rate = velocityReport_.heading_rate;
    float lateral_velocity = velocityReport_.lateral_velocity;
    float longitudinal_velocity = velocityReport_.longitudinal_velocity;
    float longitudinal_acceleration = std::lround(velocityReport_.longitudinal_acceleration * 100);
    uint8_t gearStatus = vehicleStatus_.gear;
    float steering_tire_angle = vehicleStatus_.steering_tire_angle;

    long speed = std::lround(std::sqrt(std::pow(longitudinal_velocity, 2) + std::pow(lateral_velocity, 2)) * 100);
    if (0 <= speed && speed <= 16382) bvc.speed.speedValue = speed;
    else bvc.speed.speedValue = SpeedValue_unavailable;

    if (gearStatus == 2 || gearStatus == 5)
      bvc.driveDirection = DriveDirection_forward;
    else if (gearStatus == 3)
      bvc.driveDirection = DriveDirection_backward;
    else
      bvc.driveDirection = DriveDirection_unavailable;

    long vehicleLength = std::lround((vehicleDimensions_.front_overhang + vehicleDimensions_.wheel_base + vehicleDimensions_.rear_overhang) * 10);
    if (1 <= vehicleLength && vehicleLength <= 1022) bvc.vehicleLength.vehicleLengthValue = vehicleLength;
    else bvc.vehicleLength.vehicleLengthValue = VehicleLengthValue_unavailable;

    long vehicleWidth = std::lround((vehicleDimensions_.left_overhang + vehicleDimensions_.wheel_tread + vehicleDimensions_.right_overhang) * 10);
    if (1 <= vehicleWidth && vehicleWidth <= 61) bvc.vehicleWidth = vehicleWidth;
    else bvc.vehicleWidth = VehicleWidth_unavailable;

    if (-160 <= longitudinal_acceleration && longitudinal_acceleration <= 160) bvc.longitudinalAcceleration.longitudinalAccelerationValue = longitudinal_acceleration;
    else bvc.longitudinalAcceleration.longitudinalAccelerationValue = LongitudinalAccelerationValue_unavailable;

    long curvature = longitudinal_velocity != 0 ? std::abs(std::lround(lateral_velocity / std::pow(longitudinal_velocity, 2) * 100)) * (steering_tire_angle < 0 ? -1 : 1)
                                                : std::numeric_limits<long>::infinity();
    if (-1023 <= curvature && curvature <= 1022) bvc.curvature.curvatureValue = curvature;
    else bvc.curvature.curvatureValue = CurvatureValue_unavailable;
    bvc.curvatureCalculationMode = CurvatureCalculationMode_yawRateNotUsed;

    long heading_rate_deg = std::abs(std::lround(heading_rate * (180.0 / M_PI))) * (steering_tire_angle < 0 ? -1 : 1);
    if (-32766 <= heading_rate_deg && heading_rate_deg <= 32766) bvc.yawRate.yawRateValue = heading_rate_deg;
    else bvc.yawRate.yawRateValue = YawRateValue_unavailable;

    // UNAVAILABLE VALUES
    basic_container.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
    // ------------------------------
    bvc.heading.headingConfidence = HeadingConfidence_unavailable;
    bvc.speed.speedConfidence = SpeedConfidence_unavailable;
    bvc.vehicleLength.vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication_unavailable;
    bvc.longitudinalAcceleration.longitudinalAccelerationConfidence = AccelerationConfidence_unavailable;
    bvc.curvature.curvatureConfidence = CurvatureConfidence_unavailable;
    bvc.yawRate.yawRateConfidence = YawRateConfidence_unavailable;
    // ------------------------------

    RCLCPP_INFO(node_->get_logger(), "[CamApplication::send] Sending CAM from station with ID %ld", stationId_);
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
