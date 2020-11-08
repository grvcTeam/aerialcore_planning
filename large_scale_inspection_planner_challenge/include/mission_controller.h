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
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <aerialcore_msgs/StartSupervising.h>
#include <aerialcore_msgs/StopSupervising.h>
#include <aerialcore_msgs/GraphNode.h>
#include <aerialcore_msgs/DoSpecificSupervision.h>

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

  void timerCallback(const ros::TimerEvent&);

  bool startSupervisingServiceCallback(aerialcore_msgs::StartSupervising::Request& _req, aerialcore_msgs::StartSupervising::Response& _res);
  bool stopSupervisingServiceCallback(aerialcore_msgs::StopSupervising::Request& _req, aerialcore_msgs::StopSupervising::Response& _res);
  bool doSpecificSupervisionServiceCallback(aerialcore_msgs::DoSpecificSupervision::Request& _req, aerialcore_msgs::DoSpecificSupervision::Response& _res);
  bool doContinuousSupervisionServiceCallback(std_srvs::Trigger::Request& _req, std_srvs::Trigger::Response& _res);

  ros::ServiceServer start_supervising_srv_;
  ros::ServiceServer stop_supervising_srv_;
  ros::ServiceServer do_specific_supervision_srv_;
  ros::ServiceServer do_continuous_supervision_srv_;

  // Power lines graph:
  std::vector<aerialcore_msgs::GraphNode> current_graph_;
  std::vector<aerialcore_msgs::GraphNode> complete_graph_;
  std::vector<aerialcore_msgs::GraphNode> specific_subgraph_;

  CentralizedPlanner centralized_planner_;  // Planner that assigns nodes to inspect of the electric grid graph for each UAV.

  void translateFlightPlanIntoUAVMission(const std::vector<aerialcore_msgs::FlightPlan>& _flight_plan);

  // Landing stations:
  struct LandingStation {
    geometry_msgs::Point point; // Position of the landing station.
    int id;                     // Identifier of the landing station.
    int index;                  // Index where you can find this node in the current graph.
    bool recharge = false;      // True if it's a recharge landing station.
    bool busy = false;          // True if a UAV is landed here.
  };
  std::vector<LandingStation> landing_stations_;

  // UAVs:
  enum struct AutopilotType {PX4, APM, DJI, UNKNOWN};
  enum struct AirframeType {FIXED_WING, MULTICOPTER, VTOL, OTHER, UNKNOWN};
  struct UAV {
    grvc::mission_ns::Mission mission;  // For simulation!!! In the real flights of november there will be no VTOL nor fixed wing, only DJI multicopters.

    geometry_msgs::PoseStamped pose_stamped;  // Updated continuously. Can be found at "mission".
    float battery_percentage;                 // Updated continuously. Can be found at "mission".

    int time_max_flying;  // Used for battery drop estimation, this is the estimated maximum flying time (in seconds) of this specific UAV before the drone runs out of battery.

    int speed_xy;         // MPC_XY_VEL_MAX    [min, default, max] : [ 0.0 , 12 , 20.0 ]  Maximum horizontal velocity (m/s) of this specific UAV (in AUTO mode, if higher speeds are commanded in a mission they will be capped to this velocity).
    int speed_z_down;     // MPC_Z_VEL_MAX_DN  [min, default, max] : [ 0.5 ,  1 ,  4.0 ]  Maximum vertical descent velocity (m/s) of this specific UAV (in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL)).
    int speed_z_up;       // MPC_Z_VEL_MAX_UP  [min, default, max] : [ 0.5 ,  3 ,  8.0 ]  Maximum vertical ascent velocity (m/s) of this specific UAV (in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL)).

    int id;               // Can be found at "mission".

    AutopilotType autopilot_type = AutopilotType::UNKNOWN;  // Can be found at "mission".
    AirframeType airframe_type = AirframeType::UNKNOWN;     // Can be found at "mission".

    bool recharging = false;
    bool enabled_to_supervise = false;
  };
  std::vector<UAV> UAVs_; // Can't make a map instead of vector because mission_lib. See the following line:
  // std::map<int, grvc::mission_ns::Mission> mission_by_uav_;  // Doesn't work because the inner working of map needs Mission to have a copy constructor, and that's more work than the alternatives. Left here for reference.

  std::thread spin_thread_;

  // Timer
  ros::Timer timer_;

}; // MissionController class

} // namespace aerialcore

#endif // MISSION_CONTROLLER_H