/**
 * AERIALCORE Project:
 *
 * Centralized planner.
 * 
 */

#ifndef CENTRALIZED_PLANNER_H
#define CENTRALIZED_PLANNER_H

#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <tuple>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <geographic_msgs/GeoPoint.h>
#include <aerialcore_msgs/FlightPlan.h>
#include <aerialcore_msgs/GraphNode.h>

#include <path_planner.h>

namespace aerialcore {

/// CentralizedPlanner class that works as interface
class CentralizedPlanner {

public:
    CentralizedPlanner();
    ~CentralizedPlanner();

    std::vector<aerialcore_msgs::FlightPlan> const getPlan() { return flight_plan_; };      // Returns plan already calculated.
    std::vector<aerialcore_msgs::FlightPlan> getPlan(const std::vector<aerialcore_msgs::GraphNode>& _graph, const std::map< int, std::tuple<geometry_msgs::PoseStamped,float, float, int, int, int, int, int, bool, bool> >& _drone_info);    // Returns new plan.

private:

    multidrone::PathPlanner path_planner_;

    std::vector<aerialcore_msgs::GraphNode> graph_;

    // UAVs:
    struct UAV {
        geometry_msgs::PoseStamped initial_pose;    // Current pose stsamped of the UAV. Updated continuously.
        float initial_battery;                      // Current battery in parts per unit (not percentage or %). Updated continuously.

        float minimum_battery = 0.2;    // Battery in parts per unit (not percentage or %) considered fully discharged. LiPo batteries should never discharge to less than 20% or else the life span (number of charge/discharge cycles) will be dramatically reduced.
        int time_until_fully_charged;   // Used for battery charge time estimation, time expected to charge completely the batteries in the charging pad from discharge state.

        int time_max_flying;            // Used for battery drop estimation, this is the estimated maximum flying time (in seconds) of this specific UAV before the drone runs out of battery.

        int speed_xy;         // Maximum horizontal velocity (m/s) of this specific UAV (in AUTO mode, if higher speeds are commanded in a mission they will be capped to this velocity).
        int speed_z_down;     // Maximum vertical descent velocity (m/s) of this specific UAV (in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL)).
        int speed_z_up;       // Maximum vertical ascent velocity (m/s) of this specific UAV (in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL)).

        bool landed_or_flying_initially = true;     // True if landed initially, false if flying.
        bool recharging_initially = false;
    };
    std::map<int, UAV> UAVs_;

    std::vector<aerialcore_msgs::FlightPlan> flight_plan_;  // Output.

};  // end CentralizedPlanner class

}   // end namespace aerialcore

#endif  // CENTRALIZED_PLANNER_H