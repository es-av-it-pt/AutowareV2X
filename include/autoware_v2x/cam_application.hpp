#ifndef CAM_APPLICATION_HPP_EUIC2VFR
#define CAM_APPLICATION_HPP_EUIC2VFR

#include "autoware_v2x/application.hpp"
#include "rclcpp/rclcpp.hpp"
#include <boost/asio/io_service.hpp>
#include <boost/asio/steady_timer.hpp>
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_report.hpp"
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
  //std::string uuidToHexString(const unique_identifier_msgs::msg::UUID&);
  void indicate(const DataIndication &, UpPacketPtr) override;
  void set_interval(vanetza::Clock::duration);
  //void setAllObjectsOfPersonsAnimalsToSend(const autoware_auto_control_msgs::msg::LongitudinalCommand::ConstSharedPtr);
  void getVehicleDimensions(const autoware_adapi_v1_msgs::msg::VehicleDimensions);
  void updateVelocityReport(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr);
  void updateGearReport(const autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr);
  void updateMGRS(double *, double *);
  void updateRP(double *, double *, double *);
  void updateGenerationTime(int *, long *);
  void updateHeading(double *);
  //void printObjectsList(int);
  void send();

  /** Creates the database schema.
         *  Creates tables for cam_sent, cam_received
   */
  //void createTables();

  /** Inserts CAM into the database.
         *  Constructs and executes queries for inserting the specified CAM data.
         *  @param cam The CAM to be inserted 
         *  @param table_name The table to insert the CAM into (cam_sent or cam_received)
   */
  //void insertCamToCamTable(vanetza::asn1::Cam, char*);

  struct Object {
    std::string uuid;
    int vehicleID;
    vanetza::Clock::time_point timestamp;
    rclcpp::Time timestamp_ros;
    
    // BasicContainer
    long stationType;
    long latitude;                            // Obtained from tfMessage (same value as CPMs)
    long longitude;                           // Obtained from tfMessage (same value as CPMs)
    long semiMajorConfidence;                 // Confidence (?)
    long semiMinorConfidence;                 // Confidence (?)
    long semiMajorOrientation;                // Confidence (?)
    long altitude;                            // Obtained from tfMessage (same value as CPMs)
    long altitudeConfidence;                  // Confidence (?)

    // BasicVehicleContainerHighFrequency
    long heading;                             // Obtained from tfMessage (same value as CPMs)
    long headingConfidence;                   // Confidence (?)
    long speed;                               // Obtained from velocityReport after calculation
    long speedConfidence;                     // Confidence (?)
    long driveDirection;                      // Obtained from gearReport
    long vehicleLength;
    long vehicleLengthConfidence;             // Confidence (?)
    long vehicleWidth;
    long longitudinalAcceleration;            // Obtained from velocityReport after calculation over time
    long longitudinalAccelerationConfidence;  // Confidence (?)
    long curvature;                           // Obtained from velocityReport after calculation
    long curvatureConfidence;                 // Confidence (?)
    long curvatureCalculationMode;            // Manually set
    long yawRate;                             // Obtained from velocityReport
    long yawRateConfidence;                   // Confidence (?)

    vanetza::PositionFix position;
    int timeOfMeasurement;
    bool to_send;
    int to_send_trigger;
  };
  
private:
  void schedule_timer();
  void on_timer(vanetza::Clock::time_point);

  V2XNode *node_;
  vanetza::Runtime &runtime_;
  vanetza::Clock::duration cam_interval_;

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

  struct VelocityReport {
    rclcpp::Time stamp;
    float heading_rate;
    float lateral_velocity;
    float longitudinal_velocity;
    float longitudinal_acceleration;
  };
  VelocityReport velocityReport_;
  
  uint8_t gearReport_;
  
  int generationTime_;
  long gdt_timestamp_;

  double objectConfidenceThreshold_;

  bool updating_velocity_report_;
  bool updating_gear_report_;
  bool sending_;
  bool is_sender_;
  bool reflect_packet_;
  bool include_all_persons_and_animals_;

  int cam_num_;
  int received_cam_num_;

  bool use_dynamic_generation_rules_;
};
}

#endif /* CAM_APPLICATION_HPP_EUIC2VFR */
