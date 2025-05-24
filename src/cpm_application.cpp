#include "autoware_v2x/cpm_application.hpp"
#include "autoware_v2x/positioning.hpp"
#include "autoware_v2x/security.hpp"
#include "autoware_v2x/link_layer.hpp"
#include "autoware_v2x/v2x_node.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "rclcpp/rclcpp.hpp"
#include <vanetza/btp/ports.hpp>
#include <vanetza/asn1/cpm.hpp>
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

namespace v2x {
  CpmApplication::CpmApplication(V2XNode *node, Runtime &rt, unsigned long stationId, bool is_sender) :
    node_(node),
    runtime_(rt),
    ego_(),
    generationTime_(0),
    updating_objects_list_(false),
    sending_(false),
    is_sender_(is_sender),
    reflect_packet_(false),
    objectConfidenceThreshold_(0.0),
    include_all_persons_and_animals_(false),
    cpm_object_id_(0),
    use_dynamic_generation_rules_(false)
  {
    RCLCPP_INFO(node_->get_logger(), "CpmApplication started. is_sender: %s", is_sender_ ? "yes" : "no");
    stationId_ = stationId;
    set_interval(milliseconds(100));
    createTables();
  }

  void CpmApplication::set_interval(Clock::duration interval) {
    cpm_interval_ = interval;
    runtime_.cancel(this);
    schedule_timer();
  }

  void CpmApplication::schedule_timer() {
    runtime_.schedule(cpm_interval_, std::bind(&CpmApplication::on_timer, this, std::placeholders::_1), this);
  }

  void CpmApplication::on_timer(Clock::time_point) {
    schedule_timer();
    send();
  }

  CpmApplication::PortType CpmApplication::port() {
    return btp::ports::CPM;
  }

  std::string CpmApplication::uuidToHexString(const unique_identifier_msgs::msg::UUID &id) {
    std::stringstream ss;
    for (auto i = 0; i < 16; ++i) {
      ss << std::hex << std::setfill('0') << std::setw(2) << +id.uuid[i];
    }
    return ss.str();
  }

  void CpmApplication::indicate(const DataIndication &indication, UpPacketPtr packet) {

    asn1::PacketVisitor<asn1::Cpm> visitor;
    std::shared_ptr<const asn1::Cpm> cpm = boost::apply_visitor(visitor, *packet);

    if (cpm) {
      rclcpp::Time current_time = node_->now();
      // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::indicate] [measure] T_receive_r1 %ld", current_time.nanoseconds());

      asn1::Cpm message = *cpm;
      ItsPduHeader_t &header = message->header;

       std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds> (
        std::chrono::system_clock::now().time_since_epoch()
      );
      node_->latency_log_file << "T_received," << header.stationID << "," << ms.count() << std::endl;


      // Calculate GDT and get GDT from CPM and calculate the "Age of CPM"
      TimestampIts_t gt_cpm;
      asn_long2INTEGER(&gt_cpm, (long) message->cpm.generationDeltaTime);
      // const auto time_now = duration_cast<milliseconds> (runtime_.now().time_since_epoch());
      // uint16_t gdt = time_now.count();
      // int gdt_diff = (65536 + (gdt - gdt_cpm) % 65536) % 65536;
      // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::indicate] [measure] GDT_CPM: %ld", gdt_cpm);
      // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::indicate] [measure] GDT: %u", gdt);
      // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::indicate] [measure] T_R1R2: %d", gdt_diff);


      CpmManagementContainer_t &management = message->cpm.cpmParameters.managementContainer;
      double lat = management.referencePosition.latitude / 1.0e7;
      double lon = management.referencePosition.longitude / 1.0e7;

      int zone;
      int grid_num_x = 4;
      int grid_num_y = 39;
      int grid_size = 100000;
      bool northp;
      double x, y;

      GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, x, y);
      double x_mgrs = x - grid_num_x * grid_size;
      double y_mgrs = y - grid_num_y * grid_size;

      OriginatingVehicleContainer_t &ovc = message->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer;

      // Calculate ego-vehicle orientation (radians) from heading (degree).
      // orientation: True-East, counter-clockwise angle. (0.1 degree accuracy)
      int heading = ovc.heading.headingValue;
      double orientation = (90.0 - (double) heading / 10.0) * M_PI / 180.0;
      if (orientation < 0.0) orientation += (2.0 * M_PI);
      // double orientation = heading / 10.0;
      // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::indicate] heading: %d", heading);
      // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::indicate] orientation: %f", orientation);


      // Publish CPM Sender info to /v2x/cpm/sender through V2XNode function
      node_->publishCpmSenderObject(x_mgrs, y_mgrs, orientation);


      // Get PerceivedObjects
      receivedObjectsStack.clear();

      PerceivedObjectContainer_t *&poc = message->cpm.cpmParameters.perceivedObjectContainer;

      if (poc != NULL) {
        for (int i = 0; i < poc->list.count; ++i) {
          // RCLCPP_INFO(node_->get_logger(), "[INDICATE] Object: #%d", poc->list.array[i]->objectID);

          CpmApplication::Object object;
          double x1 = poc->list.array[i]->xDistance.value;
          double y1 = poc->list.array[i]->yDistance.value;
          x1 = x1 / 100.0;
          y1 = y1 / 100.0;
          object.position_x = x_mgrs + (cos(orientation) * x1 - sin(orientation) * y1);
          object.position_y = y_mgrs + (sin(orientation) * x1 + cos(orientation) * y1);
          object.shape_x = poc->list.array[i]->planarObjectDimension2->value;
          object.shape_y = poc->list.array[i]->planarObjectDimension1->value;
          object.shape_z = poc->list.array[i]->verticalObjectDimension->value;

          object.yawAngle = poc->list.array[i]->yawAngle->value;
          double yaw_radian = (M_PI * object.yawAngle / 10.0) / 180.0;

          tf2::Quaternion quat;
          quat.setRPY(0, 0, yaw_radian);
          object.orientation_x = quat.x();
          object.orientation_y = quat.y();
          object.orientation_z = quat.z();
          object.orientation_w = quat.w();
          // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::indicate] object.quat: %f, %f, %f, %f", object.orientation_x, object.orientation_y, object.orientation_z, object.orientation_w);

          receivedObjectsStack.push_back(object);
        }
        node_->publishObjects(&receivedObjectsStack, header.stationID);
      } else {
        // RCLCPP_INFO(node_->get_logger(), "[INDICATE] Empty POC");
      }

      insertCpmToCpmTable(message, (char*) "cpm_received");

      if (reflect_packet_) {
        Application::DownPacketPtr packet{new DownPacket()};
        std::unique_ptr<geonet::DownPacket> payload{new geonet::DownPacket()};

        payload->layer(OsiLayer::Application) = std::move(message);

        Application::DataRequest request;
        request.its_aid = aid::CP;
        request.transport_type = geonet::TransportType::SHB;
        request.communication_profile = geonet::CommunicationProfile::ITS_G5;

        Application::DataConfirm confirm = Application::request(request, std::move(payload), node_);

        if (!confirm.accepted()) {
          throw std::runtime_error("[CpmApplication::indicate] Packet reflection failed");
        }
      }
    } else {
      RCLCPP_INFO(node_->get_logger(), "[INDICATE] Received broken content");
    }
  }

  void CpmApplication::updateMGRS(double *x, double *y) {
    ego_.mgrs_x = *x;
    ego_.mgrs_y = *y;
    // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::updateMGRS] ego-vehicle.position: %.10f, %.10f", ego_.mgrs_x, ego_.mgrs_y);
  }

  void CpmApplication::updateRP(double *lat, double *lon, double *altitude) {
    ego_.latitude = *lat;
    ego_.longitude = *lon;
    ego_.altitude = *altitude;
  }

  void CpmApplication::updateConfidencePositionEllipse(const double *majorAxis, const double *minorAxis, const double *orientation) {
    positionConfidenceEllipse_.majorAxisLength = *majorAxis;
    positionConfidenceEllipse_.minorAxisLength = *minorAxis;
    positionConfidenceEllipse_.majorAxisOrientation = *orientation;
  }

  void CpmApplication::updateGenerationTime(int *gdt, int *gdt_timestamp) {
    generationTime_ = *gdt;
    gdt_timestamp_ = *gdt_timestamp; // ETSI-epoch milliseconds timestamp
  }

  void CpmApplication::updateHeading(double *yaw) {
    ego_.heading = *yaw;
  }

  void CpmApplication::updateVelocityReport(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr msg) {
    rclcpp::Time msg_stamp(msg->header.stamp.sec, msg->header.stamp.nanosec);
    double dt = msg_stamp.seconds() - velocityReport_.stamp.seconds();
    if (dt == 0) {
      RCLCPP_WARN(node_->get_logger(), "[CamApplication::updateVelocityReport] deltaTime is 0");
      return;
    }
    float longitudinal_acceleration = (msg->longitudinal_velocity - velocityReport_.longitudinal_velocity) / dt;
    float speed = std::sqrt(std::pow(msg->longitudinal_velocity, 2) + std::pow(msg->lateral_velocity, 2));

    velocityReport_.stamp = msg->header.stamp;
    velocityReport_.heading_rate = msg->heading_rate;
    velocityReport_.lateral_velocity = msg->lateral_velocity;
    velocityReport_.longitudinal_velocity = msg->longitudinal_velocity;
    velocityReport_.speed = speed;
    velocityReport_.longitudinal_acceleration = longitudinal_acceleration;
  }

  void CpmApplication::setAllObjectsOfPersonsAnimalsToSend(const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg) {
    if (msg->objects.size() > 0) {
      for (autoware_auto_perception_msgs::msg::PredictedObject obj : msg->objects) {
        std::string object_uuid = uuidToHexString(obj.object_id);
        auto found_object = std::find_if(objectsList.begin(), objectsList.end(), [&](auto const &e) {
          return !strcmp(e.uuid.c_str(), object_uuid.c_str());
        });

        if (found_object == objectsList.end()) {

        } else {
          found_object->to_send = true;
        }
      }

    }
  }

  void CpmApplication::updateObjectsList(const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg) {
    updating_objects_list_ = true;

    // Flag all objects as NOT_SEND
    if (objectsList.size() > 0) {
      for (auto& object : objectsList) {
        object.to_send = false;
        object.to_send_trigger = -1;
      }
    }

    if (msg->objects.size() > 0) {
      for (autoware_auto_perception_msgs::msg::PredictedObject obj : msg->objects) {

        // RCLCPP_INFO(node_->get_logger(), "%d", obj.classification.front().label);
        double existence_probability = obj.existence_probability;
        // RCLCPP_INFO(node_->get_logger(), "existence_probability: %f", existence_probability);

        std::string object_uuid = uuidToHexString(obj.object_id);
        // RCLCPP_INFO(node_->get_logger(), "received object_id: %s", object_uuid.c_str());

        // RCLCPP_INFO(node_->get_logger(), "ObjectsList count: %d", objectsList.size());

        if (existence_probability >= objectConfidenceThreshold_) {
          // ObjectConfidence > ObjectConfidenceThreshold

          // Object tracked in internal memory? (i.e. Is object included in ObjectsList?)
          auto found_object = std::find_if(objectsList.begin(), objectsList.end(), [&](auto const &e) {
            return !strcmp(e.uuid.c_str(), object_uuid.c_str());
          });

          if (found_object == objectsList.end()) {
            // Object is new to internal memory

            if (cpm_object_id_ > 255) {
              cpm_object_id_ = 0;
            }

            // Add new object to ObjectsList
            CpmApplication::Object object;
            object.objectID = cpm_object_id_;
            object.uuid = object_uuid;
            object.timestamp_ros = msg->header.stamp;
            object.position_x = obj.kinematics.initial_pose_with_covariance.pose.position.x;
            object.position_y = obj.kinematics.initial_pose_with_covariance.pose.position.y;
            object.position_z = obj.kinematics.initial_pose_with_covariance.pose.position.z;
            object.orientation_x = obj.kinematics.initial_pose_with_covariance.pose.orientation.x;
            object.orientation_y = obj.kinematics.initial_pose_with_covariance.pose.orientation.y;
            object.orientation_z = obj.kinematics.initial_pose_with_covariance.pose.orientation.z;
            object.orientation_w = obj.kinematics.initial_pose_with_covariance.pose.orientation.w;
            object.shape_x = std::lround(obj.shape.dimensions.x * 10.0);
            object.shape_y = std::lround(obj.shape.dimensions.y * 10.0);
            object.shape_z = std::lround(obj.shape.dimensions.z * 10.0);

            long long msg_timestamp_sec = msg->header.stamp.sec;
            long long msg_timestamp_nsec = msg->header.stamp.nanosec;
            msg_timestamp_sec -= 1072915200; // convert to etsi-epoch
            long long msg_timestamp_msec = msg_timestamp_sec * 1000 + msg_timestamp_nsec / 1000000;
            object.timeOfMeasurement = gdt_timestamp_ - msg_timestamp_msec;
            if (object.timeOfMeasurement < -1500 || object.timeOfMeasurement > 1500) {
              RCLCPP_INFO(node_->get_logger(), "[updateObjectsStack] timeOfMeasurement out of bounds: %d", object.timeOfMeasurement);
              continue;
            }

            object.to_send = true;
            object.to_send_trigger = 0;
            object.timestamp = runtime_.now();

            objectsList.push_back(object);
            ++cpm_object_id_;

          } else {

            // Object was already in internal memory

            // Object belongs to class person or animal
            if (obj.classification.front().label == autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN || obj.classification.front().label == autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN) {

              if (include_all_persons_and_animals_) {
                found_object->to_send = true;
                found_object->to_send_trigger = 5;
              }

              // Object has not been included in a CPM in the past 500 ms.
              if (runtime_.now().time_since_epoch().count() - found_object->timestamp.time_since_epoch().count() > 500000) {
                // Include all objects of class person or animal in the current CPM
                include_all_persons_and_animals_ = true;
                found_object->to_send = true;
                found_object->to_send_trigger = 5;
                setAllObjectsOfPersonsAnimalsToSend(msg);
                RCLCPP_INFO(node_->get_logger(), "Include all objects of person/animal class");
              }

            } else {
              // Object does not belong to class person or animal

              // Euclidean absolute distance has changed by more than 4m
              double dist = pow(obj.kinematics.initial_pose_with_covariance.pose.position.x - found_object->position_x, 2) + pow(obj.kinematics.initial_pose_with_covariance.pose.position.y - found_object->position_y, 2);
              dist = sqrt(dist);
              // RCLCPP_INFO(node_->get_logger(), "Distance changed: %f", dist);
              if (dist > 4) {
                found_object->to_send = true;
                found_object->to_send_trigger = 1;
              } else {

              }

              // Absolute speed changed by more than 0.5 m/s
              double speed = pow(obj.kinematics.initial_twist_with_covariance.twist.linear.x - found_object->twist_linear_x, 2) + pow(obj.kinematics.initial_twist_with_covariance.twist.linear.x- found_object->twist_linear_y, 2);
              speed = sqrt(speed);
              // RCLCPP_INFO(node_->get_logger(), "Speed changed: %f", dist);
              if (speed > 0.5) {
                found_object->to_send = true;
                found_object->to_send_trigger = 2;
              }

              // Orientation of speed vector changed by more than 4 degrees
              double twist_angular_x_diff = (obj.kinematics.initial_twist_with_covariance.twist.angular.x - found_object->twist_angular_x) * 180 / M_PI;
              double twist_angular_y_diff = (obj.kinematics.initial_twist_with_covariance.twist.angular.y - found_object->twist_angular_y) * 180 / M_PI;
              // RCLCPP_INFO(node_->get_logger(), "Orientation speed vector changed x: %f", twist_angular_x_diff);
              // RCLCPP_INFO(node_->get_logger(), "Orientation speed vector changed y: %f", twist_angular_y_diff);
              if( twist_angular_x_diff > 4 || twist_angular_y_diff > 4 ) {
                found_object->to_send = true;
                found_object->to_send_trigger = 3;
              }


              // It has been more than 1 s since last transmission of this object
              if (runtime_.now().time_since_epoch().count() - found_object->timestamp.time_since_epoch().count() > 1000000) {
                found_object->to_send = true;
                found_object->to_send_trigger = 4;
                // RCLCPP_INFO(node_->get_logger(), "Been more than 1s: %ld", runtime_.now().time_since_epoch().count() - found_object->timestamp.time_since_epoch().count());
              }

            }

            // Update found_object
            found_object->timestamp_ros = msg->header.stamp;
            found_object->position_x = obj.kinematics.initial_pose_with_covariance.pose.position.x;
            found_object->position_y = obj.kinematics.initial_pose_with_covariance.pose.position.y;
            found_object->position_z = obj.kinematics.initial_pose_with_covariance.pose.position.z;
            found_object->orientation_x = obj.kinematics.initial_pose_with_covariance.pose.orientation.x;
            found_object->orientation_y = obj.kinematics.initial_pose_with_covariance.pose.orientation.y;
            found_object->orientation_z = obj.kinematics.initial_pose_with_covariance.pose.orientation.z;
            found_object->orientation_w = obj.kinematics.initial_pose_with_covariance.pose.orientation.w;
            found_object->shape_x = std::lround(obj.shape.dimensions.x * 10.0);
            found_object->shape_y = std::lround(obj.shape.dimensions.y * 10.0);
            found_object->shape_z = std::lround(obj.shape.dimensions.z * 10.0);

            long long msg_timestamp_sec = msg->header.stamp.sec;
            long long msg_timestamp_nsec = msg->header.stamp.nanosec;
            msg_timestamp_sec -= 1072915200; // convert to etsi-epoch
            long long msg_timestamp_msec = msg_timestamp_sec * 1000 + msg_timestamp_nsec / 1000000;
            found_object->timeOfMeasurement = gdt_timestamp_ - msg_timestamp_msec;
            if (found_object->timeOfMeasurement < -1500 || found_object->timeOfMeasurement > 1500) {
              RCLCPP_INFO(node_->get_logger(), "[updateObjectsStack] timeOfMeasurement out of bounds: %d", found_object->timeOfMeasurement);
              continue;
            }

            found_object->timestamp = runtime_.now();

            // if use_dymanic_generation_rules_ == false, then always include object in CPM
            if (!use_dynamic_generation_rules_) {
              found_object->to_send = true;
              found_object->to_send_trigger = 0;
            }

          }
        }
      }
    } else {
      // No objects detected
    }

    // RCLCPP_INFO(node_->get_logger(), "ObjectsStack: %d objects", objectsStack.size());
    rclcpp::Time current_time = node_->now();
    // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::updateObjectsStack] [measure] T_objstack_updated %ld", current_time.nanoseconds());
    updating_objects_list_ = false;
  }

  void CpmApplication::printObjectsList(int cpm_num) {
    // RCLCPP_INFO(node_->get_logger(), "------------------------");
    if (objectsList.size() > 0) {
      for (auto& object : objectsList) {
        RCLCPP_INFO(node_->get_logger(), "[objectsList] %d,%d,%s,%d,%d", cpm_num, object.objectID, object.uuid.c_str(), object.to_send, object.to_send_trigger);
      }
    } else {
      RCLCPP_INFO(node_->get_logger(), "[objectsList] %d,,,,", cpm_num);
    }

    // RCLCPP_INFO(node_->get_logger(), "------------------------");
  }

  void CpmApplication::send() {
    if (!node_->positioningReceived()) return;
    if (!is_sender_) return;

    sending_ = true;

    // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::send] Sending CPM...");

    vanetza::asn1::r2::Cpm message;

    // ITS PDU Header
    Vanetza_ITS2_ItsPduHeader_t &header = message->header;
    header.protocolVersion = 2;
    header.messageId = 14;
    header.stationId = stationId_;

    Vanetza_ITS2_CpmPayload_t &cpm = message->payload;

    // ManagementContainer
    Vanetza_ITS2_CPM_PDU_Descriptions_ManagementContainer_t &management = cpm.managementContainer;
    asn_long2INTEGER(&management.referenceTime, (long) gdt_timestamp_);

    double latitude, longitude, altitude;
    node_->getGpsData(latitude, longitude, altitude);
    latitude *= 1e7;
    longitude *= 1e7;
    altitude *= 100;

    if (-900000000 <= latitude && latitude <= 900000000) management.referencePosition.latitude = latitude;
    else management.referencePosition.latitude = Vanetza_ITS2_Latitude_unavailable;
    if (-1800000000 <= longitude && longitude <= 1800000000) management.referencePosition.longitude = longitude;
    else management.referencePosition.longitude = Vanetza_ITS2_Longitude_unavailable;
    if (-100000 <= altitude && altitude <= 800000) management.referencePosition.altitude.altitudeValue = altitude;
    else management.referencePosition.altitude.altitudeValue = Vanetza_ITS2_AltitudeValue_unavailable;
    management.referencePosition.altitude.altitudeConfidence = Vanetza_ITS2_AltitudeConfidence_unavailable;

    if (0 <= positionConfidenceEllipse_.majorAxisLength && positionConfidenceEllipse_.majorAxisLength <= 4094) management.referencePosition.positionConfidenceEllipse.semiMajorConfidence = positionConfidenceEllipse_.majorAxisLength;
    else management.referencePosition.positionConfidenceEllipse.semiMajorConfidence = Vanetza_ITS2_SemiAxisLength_unavailable;
    if (0 <= positionConfidenceEllipse_.minorAxisLength && positionConfidenceEllipse_.minorAxisLength <= 4094) management.referencePosition.positionConfidenceEllipse.semiMinorConfidence = positionConfidenceEllipse_.minorAxisLength;
    else management.referencePosition.positionConfidenceEllipse.semiMinorConfidence = Vanetza_ITS2_SemiAxisLength_unavailable;
    if (0 <= positionConfidenceEllipse_.majorAxisOrientation && positionConfidenceEllipse_.majorAxisOrientation <= 3600) management.referencePosition.positionConfidenceEllipse.semiMajorOrientation = positionConfidenceEllipse_.majorAxisOrientation;
    else management.referencePosition.positionConfidenceEllipse.semiMajorOrientation = Vanetza_ITS2_Wgs84AngleValue_unavailable;

    Vanetza_ITS2_WrappedCpmContainers_t &cpmContainers = cpm.cpmContainers;

    // OriginatingVehicleContainer
    auto *originatingVehicleContainer = vanetza::asn1::allocate<Vanetza_ITS2_WrappedCpmContainer>();
    originatingVehicleContainer->containerId = 1;
    originatingVehicleContainer->containerData.present = Vanetza_ITS2_WrappedCpmContainer__containerData_PR_OriginatingVehicleContainer;
    ASN_SEQUENCE_ADD(&cpmContainers, originatingVehicleContainer);

    Vanetza_ITS2_OriginatingVehicleContainer_t &originatingVehicleContainerData = originatingVehicleContainer->containerData.choice.OriginatingVehicleContainer;

    int orientationAngle = std::lround((90.0 - (atan2(velocityReport_.lateral_velocity, velocityReport_.longitudinal_velocity) * (180.0 / M_PI))) * 10.0);
    if (orientationAngle < 0) orientationAngle += 3600;
    if (0 <= orientationAngle && orientationAngle <= 3600) originatingVehicleContainerData.orientationAngle.value = orientationAngle;
    else originatingVehicleContainerData.orientationAngle.value = Vanetza_ITS2_Wgs84AngleValue_unavailable;
    originatingVehicleContainerData.orientationAngle.confidence = Vanetza_ITS2_Wgs84AngleConfidence_unavailable;

//      // TODO: SensorInformationContainer
//      auto *sensorInformationContainer = vanetza::asn1::allocate<Vanetza_ITS2_WrappedCpmContainer>();
//      sensorInformationContainer->containerId = 3;
//      sensorInformationContainer->containerData.present = Vanetza_ITS2_WrappedCpmContainer__containerData_PR_SensorInformationContainer;
//      ASN_SEQUENCE_ADD(&cpmContainers, sensorInformationContainer);

//      // TODO: PerceptionRegionContainer
//      auto *perceptionRegionContainer = vanetza::asn1::allocate<Vanetza_ITS2_WrappedCpmContainer>();
//      perceptionRegionContainer->containerId = 4;
//      perceptionRegionContainer->containerData.present = Vanetza_ITS2_WrappedCpmContainer__containerData_PR_PerceptionRegionContainer;
//      ASN_SEQUENCE_ADD(&cpmContainers, perceptionRegionContainer);

    // PerceivedObjectContainer
    auto *perceivedObjectContainer = vanetza::asn1::allocate<Vanetza_ITS2_WrappedCpmContainer>();
    perceivedObjectContainer->containerId = 5;
    perceivedObjectContainer->containerData.present = Vanetza_ITS2_WrappedCpmContainer__containerData_PR_PerceivedObjectContainer;
    ASN_SEQUENCE_ADD(&cpmContainers, perceivedObjectContainer);

    Vanetza_ITS2_PerceivedObjectContainer_t &perceivedObjectContainerData = perceivedObjectContainer->containerData.choice.PerceivedObjectContainer;
    Vanetza_ITS2_CardinalNumber1B_t *numberOfPerceivedObjects = &perceivedObjectContainerData.numberOfPerceivedObjects;
    *numberOfPerceivedObjects = 0;

    if (!objectsList.empty()) {
      Vanetza_ITS2_PerceivedObjects_t &objects = perceivedObjectContainerData.perceivedObjects;

      for (auto& object : objectsList) {
        if (!object.to_send) continue;

        auto *pObj = vanetza::asn1::allocate<Vanetza_ITS2_PerceivedObject>();

        pObj->objectId = vanetza::asn1::allocate<Vanetza_ITS2_Identifier2B_t>();
        *pObj->objectId = object.objectID;
        pObj->measurementDeltaTime = object.timeOfMeasurement;

        pObj->position.xCoordinate.value = object.position_x;
        pObj->position.xCoordinate.confidence = Vanetza_ITS2_CoordinateConfidence_unavailable;
        pObj->position.yCoordinate.value = object.position_y;
        pObj->position.yCoordinate.confidence = Vanetza_ITS2_CoordinateConfidence_unavailable;

        pObj->objectDimensionX = vanetza::asn1::allocate<Vanetza_ITS2_ObjectDimension_t>();
        pObj->objectDimensionY = vanetza::asn1::allocate<Vanetza_ITS2_ObjectDimension_t>();
        pObj->objectDimensionZ = vanetza::asn1::allocate<Vanetza_ITS2_ObjectDimension_t>();

        (*(pObj->objectDimensionX)).value = object.shape_x;
        (*(pObj->objectDimensionX)).confidence = Vanetza_ITS2_ObjectDimensionConfidence_unavailable;
        (*(pObj->objectDimensionY)).value = object.shape_y;
        (*(pObj->objectDimensionY)).confidence = Vanetza_ITS2_ObjectDimensionConfidence_unavailable;
        (*(pObj->objectDimensionZ)).value = object.shape_z;
        (*(pObj->objectDimensionZ)).confidence = Vanetza_ITS2_ObjectDimensionConfidence_unavailable;

        ASN_SEQUENCE_ADD(&objects, pObj);

        // object.to_send = false;
        // object.to_send_trigger = -1;
         RCLCPP_INFO(node_->get_logger(), "Sending object: %s", object.uuid.c_str());

        ++*numberOfPerceivedObjects;
      }
    } else {
      RCLCPP_INFO(node_->get_logger(), "No objects to send.");
    }

    RCLCPP_INFO(node_->get_logger(), "[CpmApplication::send] Sending CPM from station #%ld with %ld objects", stationId_, *numberOfPerceivedObjects);

    // insertCpmToCpmTable(message, (char*) "cpm_sent");

    std::unique_ptr<geonet::DownPacket> payload{new geonet::DownPacket()};

    payload->layer(OsiLayer::Application) = std::move(message);

    Application::DataRequest request;
    request.its_aid = aid::CP;
    request.transport_type = geonet::TransportType::SHB;
    request.communication_profile = geonet::CommunicationProfile::ITS_G5;

    Application::DataConfirm confirm = Application::request(request, std::move(payload), node_);

    if (!confirm.accepted()) {
      throw std::runtime_error("[CpmApplication::send] CPM application data request failed");
    }

    RCLCPP_INFO(node_->get_logger(), "[CpmApplication::send] Successfully sent");

    sending_ = false;
  }

  void CpmApplication::createTables() {
    sqlite3 *db = NULL;
    char* err = NULL;

    int ret = sqlite3_open("autoware_v2x.db", &db);
    if (ret != SQLITE_OK) {
      RCLCPP_INFO(node_->get_logger(), "DB File Open Error");
      return;
    }

    char* sql_command;

    sql_command = (char*) "create table if not exists cpm_sent(id INTEGER PRIMARY KEY, timestamp BIGINT, perceivedObjectCount INTEGER);";

    ret = sqlite3_exec(db, sql_command, NULL, NULL, &err);
    if (ret != SQLITE_OK) {
      RCLCPP_INFO(node_->get_logger(), "DB Execution Error (create table cpm_sent)");
      sqlite3_close(db);
      sqlite3_free(err);
      return;
    }

    sql_command = (char*) "create table if not exists cpm_received(id INTEGER PRIMARY KEY, timestamp BIGINT, perceivedObjectCount INTEGER);";

    ret = sqlite3_exec(db, sql_command, NULL, NULL, &err);
    if (ret != SQLITE_OK) {
      RCLCPP_INFO(node_->get_logger(), "DB Execution Error (create table cpm_received)");
      sqlite3_close(db);
      sqlite3_free(err);
      return;
    }

    sqlite3_close(db);
    RCLCPP_INFO(node_->get_logger(), "CpmApplication::createTables Finished");
  }

  void CpmApplication::insertCpmToCpmTable(vanetza::asn1::Cpm cpm, char* table_name) {
    sqlite3 *db = NULL;
    char* err = NULL;

    int ret = sqlite3_open("autoware_v2x.db", &db);
    if (ret != SQLITE_OK) {
      RCLCPP_INFO(node_->get_logger(), "DB File Open Error");
      return;
    }

    int64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    int perceivedObjectCount = 0;
    if (cpm->cpm.cpmParameters.numberOfPerceivedObjects) {
      perceivedObjectCount = cpm->cpm.cpmParameters.numberOfPerceivedObjects;
    }

    std::stringstream sql_command;

    sql_command << "insert into " << table_name << " (timestamp, perceivedObjectCount) values (" << timestamp << ", " << perceivedObjectCount << ");";

    ret = sqlite3_exec(db, sql_command.str().c_str(), NULL, NULL, &err);
    if (ret != SQLITE_OK) {
      RCLCPP_INFO(node_->get_logger(), "DB Execution Error (insertCpmToCpmTable)");
      RCLCPP_INFO(node_->get_logger(), sql_command.str().c_str());
      RCLCPP_INFO(node_->get_logger(), err);
      sqlite3_close(db);
      sqlite3_free(err);
      return;
    }

    sqlite3_close(db);
    // RCLCPP_INFO(node_->get_logger(), "CpmApplication::insertCpmToCpmTable Finished");
  }
}
