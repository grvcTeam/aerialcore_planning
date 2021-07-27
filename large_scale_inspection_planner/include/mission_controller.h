/**
 * AERIALCORE Project:
 *
 * Mission controller.
 * 
 */

#ifndef MISSION_CONTROLLER_H
#define MISSION_CONTROLLER_H

#include <ros/ros.h>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <vector>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>

#include <std_srvs/Trigger.h>
#include <geographic_msgs/GeoPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <aerialcore_msgs/StartSupervising.h>
#include <aerialcore_msgs/StopSupervising.h>
#include <aerialcore_msgs/GraphNode.h>
#include <aerialcore_msgs/DoSpecificSupervision.h>
#include <aerialcore_msgs/PostString.h>

#include <centralized_planner.h>
#include <parameter_estimator.h>
#include <plan_monitor.h>
#include <mission_lib.h>
#include <geographic_to_cartesian.h>
#include <path_planner.h>

namespace aerialcore {

class MissionController {

public:
  MissionController();
  ~MissionController();

private:
  ros::NodeHandle n_;
  ros::NodeHandle pnh_;

  bool complete_or_specific_graph_supervision_ = true;  // True if complete graph supervision (default), false if specific.
  bool continuous_or_fast_supervision_ = true;          // True if continuous supervision (slow, default, one UAV at the time), false if fast with all the UAVs flying together.

  bool startSupervisingServiceCallback(aerialcore_msgs::StartSupervising::Request& _req, aerialcore_msgs::StartSupervising::Response& _res);
  bool stopSupervisingServiceCallback(aerialcore_msgs::StopSupervising::Request& _req, aerialcore_msgs::StopSupervising::Response& _res);
  bool doCompleteSupervisionServiceCallback(std_srvs::Trigger::Request& _req, std_srvs::Trigger::Response& _res);
  bool doSpecificSupervisionServiceCallback(aerialcore_msgs::DoSpecificSupervision::Request& _req, aerialcore_msgs::DoSpecificSupervision::Response& _res);
  bool doContinuousSupervisionServiceCallback(std_srvs::Trigger::Request& _req, std_srvs::Trigger::Response& _res);
  bool doFastSupervisionServiceCallback(std_srvs::Trigger::Request& _req, std_srvs::Trigger::Response& _res);
  bool startSpecificSupervisionPlanServiceCallback(aerialcore_msgs::PostString::Request& _req, aerialcore_msgs::PostString::Response& _res);

  void parameterEstimatorThread(void);
  void planThread(void);

  int findUavIndexById(int _UAV_id);
  static bool compareFlightPlanById(const aerialcore_msgs::FlightPlan &a, const aerialcore_msgs::FlightPlan &b);

  void printCurrentGraph();

  void removeGraphNodesAndEdgesAboveNoFlyZones(std::vector<aerialcore_msgs::GraphNode>& _graph_to_edit);

  void translateFlightPlanIntoUAVMission(std::vector<aerialcore_msgs::FlightPlan>& _flight_plans);
  std::string translateFlightPlanIntoDJIyaml(const std::vector<aerialcore_msgs::FlightPlan>& _flight_plans);
  std::string translateFlightPlanIntoStdYaml(const std::vector<aerialcore_msgs::FlightPlan>& _flight_plans);

  ros::ServiceServer start_supervising_srv_;
  ros::ServiceServer stop_supervising_srv_;
  ros::ServiceServer do_complete_supervision_srv_;
  ros::ServiceServer do_specific_supervision_srv_;
  ros::ServiceServer do_continuous_supervision_srv_;
  ros::ServiceServer do_fast_supervision_srv_;
  ros::ServiceServer start_specific_supervision_plan_srv_;
  ros::ServiceClient post_yaml_client_;

  // Power lines graphs:
  std::vector<aerialcore_msgs::GraphNode> complete_graph_;
  std::vector<aerialcore_msgs::GraphNode> complete_graph_cleaned_;
  std::vector<aerialcore_msgs::GraphNode> specific_subgraph_;
  std::vector<aerialcore_msgs::GraphNode> specific_subgraph_cleaned_;
  std::vector<aerialcore_msgs::GraphNode> current_graph_;

  std::mutex current_graph_mutex_;
  std::mutex update_matrices_mutex_;

  CentralizedPlanner centralized_planner_;  // Planner that assigns nodes to inspect of the electric grid graph for each UAV.
  ParameterEstimator parameter_estimator_;  // Module to estimate the time_cost_matrices and battery_drop_matrices between edges.
  PlanMonitor        plan_monitor_;         // Module to check if the drones are deviating enough to run the replanning.
  grvc::PathPlanner  path_planner_;

  std::vector<aerialcore_msgs::FlightPlan> flight_plans_;

  // Argument UAV_command_mode can be:
  //  - mission_lib: the plan from the planner will be translated to a plan with mission_lib.
  //  - DJI_SDK: the plan from the planner will be translated to a yaml plan and sent to the DJI SDK.
  //  - std_yaml: the plan from the planner will be translated to a standard yaml plan and sent to a given ros service (CTU integration).
  std::string UAV_command_mode_ = "mission_lib";

  std::atomic<bool> stop_current_supervising_ = {false};

  geographic_msgs::GeoPoint map_origin_geo_;
  std::vector<geometry_msgs::Polygon> no_fly_zones_;
  geometry_msgs::Polygon geofence_;

  // UAVs:
  struct UAV {
    grvc::Mission * mission = nullptr;  // For simulation!!! In the real flights of November there will be no VTOL nor fixed wing, only DJI multicopters.

    std::string airframe_type;  // Not the one from the roslaunch (PX4), but the one in the yaml.

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

  std::thread parameter_estimator_thread_;
  std::thread plan_thread_;

  float parameter_estimator_time_;
  float plan_monitor_time_;

}; // MissionController class

} // namespace aerialcore

#endif // MISSION_CONTROLLER_H