/**
 * AERIALCORE Project:
 *
 * Centralized planner.
 * 
 */

#ifndef CENTRALIZED_PLANNER_H
#define CENTRALIZED_PLANNER_H

#include <string>
#include <vector>
#include <map>
#include <iostream>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <geographic_msgs/GeoPoint.h>
#include <aerialcore_msgs/FlightPlan.h>

#include <path_planner.h>

namespace aerialcore {

/// CentralizedPlanner class that works as interface
class CentralizedPlanner {

public:
    CentralizedPlanner();
    ~CentralizedPlanner();

// TODO:
    // bool setGraph(std::vector<GraphNode>);
    // bool setUAVs(std::vector< tuple ?? >);
// BETTER WITHOUT SETTERS, LIKE MULTIDRONE? JUST ONE GETPLAN THAT RECEIVES ALL:
// std::map< int, std::vector<multidrone_msgs::DroneAction> > current_getplan_plan = high_level_planner_.getPlan(it->second, initial_state_of_drones, origin_coordinates_geo_, incoming_KML, trigger_time_of_replanning);
// EXAMPLE OF TUPLES:
//   // Start conditions of the drones for the High-Level Planner .getPlan()
//   std::map< int,std::tuple<geometry_msgs::Point32,double,bool> > initial_state_of_drones; // map of tuples. The keys are the drone ids, the values a tuple with three elements each oen. The first in the tuple is the pose of the drone, the second its battery and the third a boolean that is false if the drone is in the ground and true if flying.
//   for (int current_drone_id : drones_id_) {
//     if ( drone_status_list[ current_drone_id ].drone_pose_defined == true && drone_status_list[ current_drone_id ].drone_alarmed == false ) {
//       std::tuple<geometry_msgs::Point32,double,bool> new_tuple;
//       std::get<0>(new_tuple).x = drone_status_list[ current_drone_id ].pose.position.x;
//       std::get<0>(new_tuple).y = drone_status_list[ current_drone_id ].pose.position.y;
//       std::get<0>(new_tuple).z = drone_status_list[ current_drone_id ].pose.position.z;
//       std::get<1>(new_tuple) = drone_status_list[ current_drone_id ].battery_remaining;
//       if (drone_status_list[ current_drone_id ].action_status.status != multidrone_msgs::ActionStatus::AS_IDLE && drone_status_list[ current_drone_id ].action_status.status != multidrone_msgs::ActionStatus::AS_WAIT_FOR_SAFE_TO_GO && drone_status_list[ current_drone_id ].action_status.status != multidrone_msgs::ActionStatus::AS_WAIT_FOR_GET_READY) {
//         std::get<2>(new_tuple) = true;    // If the drone is not idle, or waiting for safe to go or get ready means that the drone is flying, so this is true.
//       } else {
//         std::get<2>(new_tuple) = false;
//       }
//       initial_state_of_drones[ current_drone_id ] = new_tuple;
//     }
//   }

    std::vector<aerialcore_msgs::FlightPlan> getPlan();

private:

    multidrone::PathPlanner path_planner_;


    ////////////// < Parameters of the planning problem > //////////////

    // LiPo battery constraint:
    float minimum_battery_;         // Minimum battery charge (%) that the battery can have during the plan. LiPo batteries should never discharge to less than 20% or else the life span (number of charge/discharge cycles) will be dramatically reduced.

    // Drones separation management during navigation actions of multiple drones:
    float minimum_separation_xy_;   // Minimum drone separation (meters) between drones during navigation actions.
    float minimum_separation_z_;    // Minimum drone separation (meters) between drones during navigation actions.

    // Battery drop estimation:
    int time_max_hovering_;             // Estimated maximum hovering time (in seconds) before the drone runs out of battery. Must be greater than time_max_flying_full_speed_.
    int time_max_flying_full_speed_;    // Estimated maximum flying time at full speed (in seconds) before the drone runs out of battery. Must be lower than time_max_hovering_.

    // Drone maximum speed values from PX4, information extracted from here (30-01-2019): https://dev.px4.io/en/advanced/parameter_reference.html
    // Parameters not changed in the px4cmd file right now, so default values assigned:
    int full_speed_xy_;     // Maximum horizontal velocity (m/s) of the drone (in AUTO mode, if higher speeds are commanded in a mission they will be capped to this velocity).
    int full_speed_z_down_; // Maximum vertical descent velocity (m/s) of the drone (in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL)).
    int full_speed_z_up_;   // Maximum vertical ascent velocity (m/s) of the drone (in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL)).
    // TODO?: the drone could have its default parameters changed in "/grvc-ual/robots_description/models/typhoon_h480/px4cmd" or elsewhere if different drone, but in this case that parameters aren't changed. Take this into account. Maybe parse the px4cmd file?

    ////////////// </ Parameters of the planning problem > //////////////


};  // end CentralizedPlanner class

}   // end namespace aerialcore

#endif  // CENTRALIZED_PLANNER_H