/**
 * AERIALCORE Project:
 *
 * Plan Monitor.
 * 
 */

#ifndef PLAN_MONITOR_H
#define PLAN_MONITOR_H

#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <tuple>
#include <utility>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <geographic_msgs/GeoPoint.h>
#include <aerialcore_msgs/FlightPlan.h>
#include <aerialcore_msgs/GraphNode.h>

namespace aerialcore {

/// PlanMonitor class that works as interface
class PlanMonitor {

public:
    PlanMonitor();
    ~PlanMonitor();

    // Method called periodically in an external thread, located in the Mission Controller, that will call the planner (in the same thread) if it returns true:
    bool enoughDeviationToReplan(const std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector<aerialcore_msgs::FlightPlan>& _flight_plans, const std::vector< std::tuple<float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::vector< std::vector< std::vector<float> > >& _time_cost_matrices, const std::vector< std::vector< std::vector<float> > >& _battery_drop_matrices);

private:

    // bool checkIfPlansHaveChanged (const std::vector<aerialcore_msgs::FlightPlan>& _flight_plans) const;
    void constructUAVs(const std::vector< std::tuple<float, float, int, int, int, int, int, int, bool, bool> >& _drone_info);
    int  findUavIndexById(int _UAV_id);

    // UAVs:
    struct UAV {
        float initial_battery;  // Current battery in parts per unit (not percentage or %). Updated continuously.

        float speed_xy;         // Maximum horizontal velocity (m/s) of this specific UAV (in AUTO mode, if higher speeds are commanded in a mission they will be capped to this velocity).
        float speed_z_down;     // Maximum vertical descent velocity (m/s) of this specific UAV (in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL)).
        float speed_z_up;       // Maximum vertical ascent velocity (m/s) of this specific UAV (in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL)).

        float minimum_battery = 0.2;    // Battery in parts per unit (not percentage or %) considered fully discharged. LiPo batteries should never discharge to less than 20% or else the life span (number of charge/discharge cycles) will be dramatically reduced.
        int time_until_fully_charged;   // Used for battery charge time estimation, time expected to charge completely the batteries in the charging pad from discharge state.

        int time_max_flying;            // Used for battery drop estimation, this is the estimated maximum flying time (in seconds) of this specific UAV before the drone runs out of battery.


        // Specific of the Plan Monitor:
        int last_segment = 0;      // First node of the segment that this UAV was doing for this plan the last time the Plan Monitor was called.


        int id;

        bool flying_or_landed_initially = false;     // True if flying initially, false if landed.
        bool recharging_initially = false;
    };
    std::vector<UAV> UAVs_;

    float deviation_limit_;         // Parameter read from the configuration file. Parts per unit of the duration of the plans, if this is surpassed (greater or lower) the replanning will be triggered.

};  // end PlanMonitor class

}   // end namespace aerialcore

#endif  // PLAN_MONITOR_H