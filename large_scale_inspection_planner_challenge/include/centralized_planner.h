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
#include <tuple>
#include <utility>

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

    std::vector<aerialcore_msgs::FlightPlan> getPlan(const std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector< std::tuple<float, float, int, int, int, int, int, int, bool, bool> >& _drone_info);    // Returns new plan.
    // _drone_info it's a vector of tuples, each tuple with 10 elements. The first in the tuple is the initial battery, and so on with all the elements in the "UAV" structure defined here below.

    void printPlan();

private:

    multidrone::PathPlanner path_planner_;

    std::vector<aerialcore_msgs::GraphNode> graph_;

    // UAVs:
    struct UAV {
        float initial_battery;                      // Current battery in parts per unit (not percentage or %). Updated continuously.

        float speed_xy;         // Maximum horizontal velocity (m/s) of this specific UAV (in AUTO mode, if higher speeds are commanded in a mission they will be capped to this velocity).
        float speed_z_down;     // Maximum vertical descent velocity (m/s) of this specific UAV (in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL)).
        float speed_z_up;       // Maximum vertical ascent velocity (m/s) of this specific UAV (in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL)).

        float minimum_battery = 0.2;    // Battery in parts per unit (not percentage or %) considered fully discharged. LiPo batteries should never discharge to less than 20% or else the life span (number of charge/discharge cycles) will be dramatically reduced.
        int time_until_fully_charged;   // Used for battery charge time estimation, time expected to charge completely the batteries in the charging pad from discharge state.

        int time_max_flying;            // Used for battery drop estimation, this is the estimated maximum flying time (in seconds) of this specific UAV before the drone runs out of battery.

        int id;

        bool flying_or_landed_initially = false;     // True if flying initially, false if landed.
        bool recharging_initially = false;
    };
    std::vector<UAV> UAVs_;

    std::vector< std::pair<int,int> > edges_;  // Edges or connections of the graph, being the pair of indexes of the graph nodes connected by wires. Always the first int index lower than the second of the pair.

    std::vector<aerialcore_msgs::FlightPlan> flight_plan_;  // Output.

    void nearestGraphNode(bool _pylon_or_land_station, int _from_this_index_graph, int& _index_graph_node_to_return, float& _distance_to_return);  // True searches for pylons, false for land stations.
    void mostRewardedPylon(int _initial_pylon, int& _index_graph_node_to_return, float& _distance_to_return);

    const float batteryDrop(int _flying_time, int _time_max_flying) { return (float)_flying_time/(float)_time_max_flying; }

};  // end CentralizedPlanner class

}   // end namespace aerialcore

#endif  // CENTRALIZED_PLANNER_H