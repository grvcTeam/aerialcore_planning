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
    bool enoughDeviationToReplan(const std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector<aerialcore_msgs::FlightPlan>& _flight_plans, const std::vector< std::tuple<float, float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices, const std::map<int, std::map<int, std::map<int, float> > >& _battery_drop_matrices);

    // Getter for the last graph node that each UAV was doing in its current flight plan (this is, for the edge that the UAV was covering the last time enoughDeviationToReplan, the first node of that edge):
    std::map<int, int> getLastGraphNodes();

private:

    void constructUAVs(const std::vector< std::tuple<float, float, float, int, int, int, int, int, int, bool, bool> >& _drone_info);
    int  findUavIndexById(int _UAV_id);

    std::vector<aerialcore_msgs::FlightPlan> previous_flight_plans_;

    // UAVs:
    struct UAV {
        float battery;          // Current battery in parts per unit (not percentage or %). Updated continuously.
        float initial_battery;  // Battery level when the current plan was first computed. Will be used to calculate the real battery drop.

        float speed_xy;         // Maximum horizontal velocity (m/s) of this specific UAV (in AUTO mode, if higher speeds are commanded in a mission they will be capped to this velocity).
        float speed_z_down;     // Maximum vertical descent velocity (m/s) of this specific UAV (in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL)).
        float speed_z_up;       // Maximum vertical ascent velocity (m/s) of this specific UAV (in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL)).

        float minimum_battery = 0.2;    // Battery in parts per unit (not percentage or %) considered fully discharged. LiPo batteries should never discharge to less than 20% or else the life span (number of charge/discharge cycles) will be dramatically reduced.
        int time_until_fully_charged;   // Used for battery charge time estimation, time expected to charge completely the batteries in the charging pad from discharge state.

        int time_max_flying;            // Used for battery drop estimation, this is the estimated maximum flying time (in seconds) of this specific UAV before the drone runs out of battery.


        // Specific of the Plan Monitor:
        int last_pose_index = 0;    // First pose of the pair of poses of the segment that this UAV was doing for this plan the last time the Plan Monitor was called. // TODO: 0 when mission finished.
        int last_flight_plan_graph_node = -1;


        int id;

        bool flying_or_landed_initially = false;     // True if flying initially, false if landed.
        bool recharging_initially = false;
    };
    std::vector<UAV> UAVs_;

    float deviation_limit_;             // Deviation limit (in parts per unit of the duration and battery drop of the plans), if this is surpassed (greater or lower) the Plan Monitor will tell the planner to trigger the replanning.
    float minimum_battery_difference_;  // Parts per unit of difference that has to be deviated the battery (apart from the deviation limit).
    float minimum_time_difference_;     // Seconds difference that has to be deviated the duration of the plan (apart from the deviation limit).

};  // end PlanMonitor class

}   // end namespace aerialcore

#endif  // PLAN_MONITOR_H