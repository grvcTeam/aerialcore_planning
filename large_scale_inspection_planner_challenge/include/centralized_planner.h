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

    // bool setGraph(std::vector<>);

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