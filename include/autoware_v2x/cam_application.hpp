#ifndef CAM_APPLICATION_HPP_EUIC2VFR
#define CAM_APPLICATION_HPP_EUIC2VFR

#include "autoware_v2x/application.hpp"
#include "autoware_v2x/positioning.hpp"
#include "rclcpp/rclcpp.hpp"

#include <vanetza/asn1/cam.hpp>

#include "autoware_adapi_v1_msgs/msg/vehicle_dimensions.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"

#include <boost/asio/io_service.hpp>
#include <boost/asio/steady_timer.hpp>

#include <etsi_its_cam_ts_coding/cam_ts_CAM.h>

namespace v2x
{
class V2XNode;
class CamApplication : public Application
{
public:
  CamApplication(V2XNode *node, vanetza::Runtime &rt, unsigned long stationId, bool is_sender);
  PortType port() override;
  void indicate(const DataIndication &, UpPacketPtr) override;
  void set_interval(vanetza::Clock::duration);
  void updateMGRS(double *, double *);
  void updateRP(const double *, const double *, const double *);
  void updateConfidencePositionEllipse(const double *, const double *, const double *);
  void updateHeading(const double *);
  void setVehicleDimensions(const autoware_adapi_v1_msgs::msg::VehicleDimensions &);
  void updateVelocityReport(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr);
  void updateGearReport(const autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr);
  void updateSteeringReport(const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr);
  void send();

private:
  void calc_interval();
  void schedule_timer();
  void on_timer(vanetza::Clock::time_point);

  V2XNode *node_;
  unsigned long stationId_;
  vanetza::Runtime &runtime_;
  vanetza::Clock::duration cam_interval_{};

  struct VehicleDimensions {
    float wheel_radius = 0.0;
    float wheel_width = 0.0;
    float wheel_base = 0.0;
    float wheel_tread = 0.0;
    float front_overhang = 0.0;
    float rear_overhang = 0.0;
    float left_overhang = 0.0;
    float right_overhang = 0.0;
    float height = 0.0;
  };
  VehicleDimensions vehicleDimensions_;

  struct Ego_station {
    double mgrs_x;
    double mgrs_y;
    double latitude;
    double longitude;
    double altitude;
    double heading;
  };
  Ego_station ego_;

  struct PositionConfidenceEllipse {
    double majorAxisLength{};
    double minorAxisLength{};
    double majorAxisOrientation{};
  };
  PositionConfidenceEllipse positionConfidenceEllipse_;

  struct VelocityReport {
    rclcpp::Time stamp;
    float heading_rate;
    float lateral_velocity;
    float longitudinal_velocity;
    float speed;
    float longitudinal_acceleration;
  };
  VelocityReport velocityReport_;

  struct VehicleStatus {
    uint8_t gear;
    float steering_tire_angle;
  };
  VehicleStatus vehicleStatus_;

  bool sending_;
  bool is_sender_;

  bool use_dynamic_generation_rules_;
};
}

#endif /* CAM_APPLICATION_HPP_EUIC2VFR */
