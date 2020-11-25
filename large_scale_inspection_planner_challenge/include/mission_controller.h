/**
 * AERIALCORE Project:
 *
 * Mission controller.
 * 
 */

#ifndef MISSION_CONTROLLER_H
#define MISSION_CONTROLLER_H

#include <ros/ros.h>
#include <sstream>
#include <string>
#include <vector>
#include <thread>

#include <std_srvs/Trigger.h>
#include <geographic_msgs/GeoPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <aerialcore_msgs/StartSupervising.h>
#include <aerialcore_msgs/StopSupervising.h>
#include <aerialcore_msgs/GraphNode.h>
#include <aerialcore_msgs/DoSpecificSupervision.h>
#include <aerialcore_msgs/PostString.h>

#include <centralized_planner.h>
#include <mission_lib.h>


namespace aerialcore {

class MissionController {

public:
  MissionController();
  ~MissionController();

private:
  ros::NodeHandle n_;
  ros::NodeHandle pnh_;

  bool continuous_or_specific_supervision_ = true; // True if continuous supervision (default), false if specific.

  bool startSupervisingServiceCallback(aerialcore_msgs::StartSupervising::Request& _req, aerialcore_msgs::StartSupervising::Response& _res);
  bool stopSupervisingServiceCallback(aerialcore_msgs::StopSupervising::Request& _req, aerialcore_msgs::StopSupervising::Response& _res);
  bool doSpecificSupervisionServiceCallback(aerialcore_msgs::DoSpecificSupervision::Request& _req, aerialcore_msgs::DoSpecificSupervision::Response& _res);
  bool doContinuousSupervisionServiceCallback(std_srvs::Trigger::Request& _req, std_srvs::Trigger::Response& _res);

  int findUavIndexById(int _UAV_id);

  ros::ServiceServer start_supervising_srv_;
  ros::ServiceServer stop_supervising_srv_;
  ros::ServiceServer do_specific_supervision_srv_;
  ros::ServiceServer do_continuous_supervision_srv_;
  ros::ServiceClient post_yaml_client_;

  // Power lines graph:
  std::vector<aerialcore_msgs::GraphNode> current_graph_;
  std::vector<aerialcore_msgs::GraphNode> complete_graph_;
  std::vector<aerialcore_msgs::GraphNode> specific_subgraph_;

  CentralizedPlanner centralized_planner_;  // Planner that assigns nodes to inspect of the electric grid graph for each UAV.

  std::vector<aerialcore_msgs::FlightPlan> flight_plan_;

  bool commanding_UAV_with_mission_lib_or_DJI_SDK_ = true;  // Use mission_lib for commanding UAVs (true) or output a yaml string for using the DJI SDK (false).

  geographic_msgs::GeoPoint map_origin_geo_;

  void translateFlightPlanIntoUAVMission(const std::vector<aerialcore_msgs::FlightPlan>& _flight_plan);
  std::string translateFlightPlanIntoDJIyaml(const std::vector<aerialcore_msgs::FlightPlan>& _flight_plan);

  // UAVs:
  struct UAV {
    grvc::mission_ns::Mission * mission = nullptr;  // For simulation!!! In the real flights of november there will be no VTOL nor fixed wing, only DJI multicopters.

    float speed_xy;         // Maximum horizontal velocity (m/s) of this specific UAV (in AUTO mode, if higher speeds are commanded in a mission they will be capped to this velocity).
    float speed_z_down;     // Maximum vertical descent velocity (m/s) of this specific UAV (in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL)).
    float speed_z_up;       // Maximum vertical ascent velocity (m/s) of this specific UAV (in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL)).

    float minimum_battery = 0.2;    // Battery in parts per unit (not percentage or %) considered fully discharged. LiPo batteries should never discharge to less than 20% or else the life span (number of charge/discharge cycles) will be dramatically reduced.
    int time_until_fully_charged;   // Used for battery charge time estimation, time expected to charge completely the batteries in the charging pad from discharge state.

    int time_max_flying;  // Used for battery drop estimation, this is the estimated maximum flying time (in seconds) of this specific UAV before the drone runs out of battery.

    int id;

    bool recharging = false;
    bool enabled_to_supervise = false;
  };
  std::vector<UAV> UAVs_;

  std::thread spin_thread_;

}; // MissionController class

} // namespace aerialcore

#endif // MISSION_CONTROLLER_H