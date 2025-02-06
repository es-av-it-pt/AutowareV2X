#ifndef V2X_APP_HPP_EUIC2VFR
#define V2X_APP_HPP_EUIC2VFR

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <boost/asio/io_service.hpp>
#include "autoware_v2x/cpm_application.hpp"
#include "autoware_v2x/cam_application.hpp"
#include "autoware_v2x/time_trigger.hpp"
#include "autoware_v2x/link_layer.hpp"
#include "autoware_v2x/ethernet_device.hpp"
#include "autoware_v2x/positioning.hpp"
#include "autoware_v2x/security.hpp"
#include "autoware_v2x/router_context.hpp"
// #include "autoware_v2x/v2x_node.hpp"

namespace v2x
{
  class V2XNode;
  class V2XApp
  {
  public:
    V2XApp(V2XNode *);
    void start();
    void objectsCallback(const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr);
    void velocityReportCallback(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr);
    void gearReportCallback(const autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr);
    void steeringReportCallback(const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr);
    void tfCallback(const tf2_msgs::msg::TFMessage::ConstSharedPtr);

    CpmApplication *cpm;
    CamApplication *cam;
    // V2XNode *v2x_node;

  private:
    friend class CpmApplication;
    friend class CamApplication;
    friend class Application;
    V2XNode* node_;
    bool tf_received_;
    int tf_interval_;
    bool vehicle_dimensions_set_;
    bool cpm_started_;
    bool cam_started_;

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

      double majorAxisLength{};
      double minorAxisLength{};
      double majorAxisConfidence{};
    };
    PositionConfidenceEllipse positionConfidenceEllipse_;
  };
}

#endif
