#ifndef CONVERT_CAUSE_CODE_HPP
#define CONVERT_CAUSE_CODE_HPP

#include <etsi_its_cam_ts_msgs/msg/cause_code_choice.hpp>
#include <vanetza/asn1/its/r2/CauseCodeChoice.h>

void convert_cause_code_to_ros(const Vanetza_ITS2_CauseCodeChoice_t &asn1, etsi_its_cam_ts_msgs::msg::CauseCodeChoice &ros)
{
  switch (asn1.present) {
    case Vanetza_ITS2_CauseCodeChoice_PR_reserved0:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_RESERVED0;
      ros.reserved0.value = asn1.choice.reserved0;
      break;
    case Vanetza_ITS2_CauseCodeChoice_PR_trafficCondition1:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_TRAFFIC_CONDITION1;
      ros.traffic_condition1.value = asn1.choice.trafficCondition1;
      break;
    case Vanetza_ITS2_CauseCodeChoice_PR_accident2:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_ACCIDENT2;
      ros.accident2.value = asn1.choice.accident2;
      break;
    case Vanetza_ITS2_CauseCodeChoice_PR_roadworks3:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_ROADWORKS3;
      ros.roadworks3.value = asn1.choice.roadworks3;
      break;
    case Vanetza_ITS2_CauseCodeChoice_PR_impassability5:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_IMPASSABILITY5;
      ros.impassability5.value = asn1.choice.impassability5;
      break;
    case Vanetza_ITS2_CauseCodeChoice_PR_adverseWeatherCondition_Adhesion6:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_ADVERSE_WEATHER_CONDITION_ADHESION6;
      ros.adverse_weather_condition_adhesion6.value = asn1.choice.adverseWeatherCondition_Adhesion6;
      break;
    case Vanetza_ITS2_CauseCodeChoice_PR_aquaplaning7:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_AQUAPLANING7;
      ros.aquaplaning7.value = asn1.choice.aquaplaning7;
      break;
    case Vanetza_ITS2_CauseCodeChoice_PR_hazardousLocation_SurfaceCondition9:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_HAZARDOUS_LOCATION_SURFACE_CONDITION9;
      ros.hazardous_location_surface_condition9.value = asn1.choice.hazardousLocation_SurfaceCondition9;
      break;
    case Vanetza_ITS2_CauseCodeChoice_PR_hazardousLocation_ObstacleOnTheRoad10:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_HAZARDOUS_LOCATION_OBSTACLE_ON_THE_ROAD10;
      ros.hazardous_location_obstacle_on_the_road10.value = asn1.choice.hazardousLocation_ObstacleOnTheRoad10;
      break;
    case Vanetza_ITS2_CauseCodeChoice_PR_hazardousLocation_AnimalOnTheRoad11:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_HAZARDOUS_LOCATION_ANIMAL_ON_THE_ROAD11;
      ros.hazardous_location_animal_on_the_road11.value = asn1.choice.hazardousLocation_AnimalOnTheRoad11;
      break;
    case Vanetza_ITS2_CauseCodeChoice_PR_humanPresenceOnTheRoad12:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_HUMAN_PRESENCE_ON_THE_ROAD12;
      ros.human_presence_on_the_road12.value = asn1.choice.humanPresenceOnTheRoad12;
      break;
    case Vanetza_ITS2_CauseCodeChoice_PR_wrongWayDriving14:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_WRONG_WAY_DRIVING14;
      ros.wrong_way_driving14.value = asn1.choice.wrongWayDriving14;
      break;
    case Vanetza_ITS2_CauseCodeChoice_PR_rescueAndRecoveryWorkInProgress15:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_RESCUE_AND_RECOVERY_WORK_IN_PROGRESS15;
      ros.rescue_and_recovery_work_in_progress15.value = asn1.choice.rescueAndRecoveryWorkInProgress15;
      break;
    case Vanetza_ITS2_CauseCodeChoice_PR_adverseWeatherCondition_ExtremeWeatherCondition17:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_ADVERSE_WEATHER_CONDITION_EXTREME_WEATHER_CONDITION17;
      ros.adverse_weather_condition_extreme_weather_condition17.value = asn1.choice.adverseWeatherCondition_ExtremeWeatherCondition17;
      break;
    case Vanetza_ITS2_CauseCodeChoice_PR_adverseWeatherCondition_Visibility18:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_ADVERSE_WEATHER_CONDITION_VISIBILITY18;
      ros.adverse_weather_condition_visibility18.value = asn1.choice.adverseWeatherCondition_Visibility18;
      break;
    case Vanetza_ITS2_CauseCodeChoice_PR_adverseWeatherCondition_Precipitation19:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_ADVERSE_WEATHER_CONDITION_PRECIPITATION19;
      ros.adverse_weather_condition_precipitation19.value = asn1.choice.adverseWeatherCondition_Precipitation19;
      break;
    case Vanetza_ITS2_CauseCodeChoice_PR_violence20:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_VIOLENCE20;
      ros.violence20.value = asn1.choice.violence20;
      break;
    case Vanetza_ITS2_CauseCodeChoice_PR_slowVehicle26:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_SLOW_VEHICLE26;
      ros.slow_vehicle26.value = asn1.choice.slowVehicle26;
      break;
    case Vanetza_ITS2_CauseCodeChoice_PR_dangerousEndOfQueue27:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_DANGEROUS_END_OF_QUEUE27;
      ros.dangerous_end_of_queue27.value = asn1.choice.dangerousEndOfQueue27;
      break;
    case Vanetza_ITS2_CauseCodeChoice_PR_publicTransportVehicleApproaching28:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_PUBLIC_TRANSPORT_VEHICLE_APPROACHING28;
      ros.public_transport_vehicle_approaching28.value = asn1.choice.publicTransportVehicleApproaching28;
      break;
    case Vanetza_ITS2_CauseCodeChoice_PR_vehicleBreakdown91:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_VEHICLE_BREAKDOWN91;
      ros.vehicle_breakdown91.value = asn1.choice.vehicleBreakdown91;
      break;
    case Vanetza_ITS2_CauseCodeChoice_PR_postCrash92:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_POST_CRASH92;
      ros.post_crash92.value = asn1.choice.postCrash92;
      break;
    case Vanetza_ITS2_CauseCodeChoice_PR_humanProblem93:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_HUMAN_PROBLEM93;
      ros.human_problem93.value = asn1.choice.humanProblem93;
      break;
    case Vanetza_ITS2_CauseCodeChoice_PR_stationaryVehicle94:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_STATIONARY_VEHICLE94;
      ros.stationary_vehicle94.value = asn1.choice.stationaryVehicle94;
      break;
    case Vanetza_ITS2_CauseCodeChoice_PR_emergencyVehicleApproaching95:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_EMERGENCY_VEHICLE_APPROACHING95;
      ros.emergency_vehicle_approaching95.value = asn1.choice.emergencyVehicleApproaching95;
      break;
    case Vanetza_ITS2_CauseCodeChoice_PR_hazardousLocation_DangerousCurve96:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_HAZARDOUS_LOCATION_DANGEROUS_CURVE96;
      ros.hazardous_location_dangerous_curve96.value = asn1.choice.hazardousLocation_DangerousCurve96;
      break;
    case Vanetza_ITS2_CauseCodeChoice_PR_collisionRisk97:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_COLLISION_RISK97;
      ros.collision_risk97.value = asn1.choice.collisionRisk97;
      break;
    case Vanetza_ITS2_CauseCodeChoice_PR_signalViolation98:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_SIGNAL_VIOLATION98;
      ros.signal_violation98.value = asn1.choice.signalViolation98;
      break;
    case Vanetza_ITS2_CauseCodeChoice_PR_dangerousSituation99:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_DANGEROUS_SITUATION99;
      ros.dangerous_situation99.value = asn1.choice.dangerousSituation99;
      break;
    case Vanetza_ITS2_CauseCodeChoice_PR_railwayLevelCrossing100:
      ros.choice = etsi_its_cam_ts_msgs::msg::CauseCodeChoice::CHOICE_RAILWAY_LEVEL_CROSSING100;
      ros.railway_level_crossing100.value = asn1.choice.railwayLevelCrossing100;
      break;
    default:
      break;
  }
}

#endif /* CONVERT_CAUSE_CODE_HPP */
