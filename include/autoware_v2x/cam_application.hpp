#ifndef CAM_APPLICATION_HPP_EUIC2VFR
#define CAM_APPLICATION_HPP_EUIC2VFR

#include "autoware_v2x/application.hpp"
#include "rclcpp/rclcpp.hpp"
#include <boost/asio/io_service.hpp>
#include <boost/asio/steady_timer.hpp>
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_adapi_v1_msgs/msg/vehicle_dimensions.hpp"
#include "autoware_v2x/positioning.hpp"
#include <vanetza/asn1/cam.hpp>

namespace v2x
{
class V2XNode;
class CamApplication : public Application
{
public:
  CamApplication(V2XNode *node, vanetza::Runtime &, bool is_sender);
  PortType port() override;
  void indicate(const DataIndication &, UpPacketPtr) override;
  void set_interval(vanetza::Clock::duration);
  void updateMGRS(double *, double *);
  void updateRP(double *, double *, double *);
  void updateGenerationDeltaTime(int *, long *);
  void updateHeading(double *);
  void setVehicleDimensions(const autoware_adapi_v1_msgs::msg::VehicleDimensions &);
  void updateVelocityReport(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr);
  void updateGearReport(const autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr);
  void updateSteeringReport(const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr);
  void send();

private:
  void schedule_timer();
  void on_timer(vanetza::Clock::time_point);

  V2XNode *node_;
  vanetza::Runtime &runtime_;
  vanetza::Clock::duration cam_interval_{};

  struct VehicleDimensions {
    float wheel_radius;
    float wheel_width;
    float wheel_base;
    float wheel_tread;
    float front_overhang;
    float rear_overhang;
    float left_overhang;
    float right_overhang;
    float height;
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

  class PositionsDeque {
  public:
    void insert(double value) {
      if (deque.size() >= maxSize) {
        total -= deque.front();
        deque.pop_front();
      }
      total += value;
      mean = total / deque.size();
      deque.push_back(value);
    }

    int getSize() {
      return deque.size();
    }

    [[nodiscard]] double getMean() const {
      return this->mean;
    }

    using iterator = std::deque<double>::const_iterator;

    [[nodiscard]] iterator begin() const {
      return deque.begin();
    }

    [[nodiscard]] iterator end() const {
      return deque.end();
    }

    double operator[](std::size_t index) const {
      if (index >= deque.size())
        throw std::out_of_range("[PositionDeque] Index out of range");
      return deque[index];
    }

  private:
    static const std::size_t maxSize = 5;
    std::deque<double> deque;

    double total = 0;
    double mean = 0;
  };

  struct PositionConfidenceEllipse {
    PositionsDeque x;
    PositionsDeque y;

    double semiMajorConfidence{};
    double semiMinorConfidence{};
    double semiMajorOrientation{};
  };
  PositionConfidenceEllipse positionConfidenceEllipse_;

  struct VelocityReport {
    rclcpp::Time stamp;
    float heading_rate;
    float lateral_velocity;
    float longitudinal_velocity;
    float longitudinal_acceleration;
  };
  VelocityReport velocityReport_;

  struct VehicleStatus {
    uint8_t gear;
    float steering_tire_angle;
  };
  VehicleStatus vehicleStatus_;

  int generationDeltaTime_;
  long gdt_timestamp_{};

  double objectConfidenceThreshold_;

  bool updating_velocity_report_;
  bool updating_vehicle_status_;
  bool sending_;
  bool is_sender_;
  bool reflect_packet_;

  unsigned long stationId_;
  int cam_num_;
  int received_cam_num_;

  bool use_dynamic_generation_rules_;
};
}

#endif /* CAM_APPLICATION_HPP_EUIC2VFR */
