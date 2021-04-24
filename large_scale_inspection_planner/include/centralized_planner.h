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

    std::vector<aerialcore_msgs::FlightPlan> getPlanGreedy() const { return flight_plans_; }     // Returns plan already calculated.
    std::vector<aerialcore_msgs::FlightPlan> getPlanMILP() const { return flight_plans_; }       // Returns plan already calculated.

    std::vector<aerialcore_msgs::FlightPlan> getPlanGreedy(std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector< std::tuple<float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::vector< geometry_msgs::Polygon >& _no_fly_zones, const geometry_msgs::Polygon& _geofence); // Returns new plan.
    std::vector<aerialcore_msgs::FlightPlan> getPlanMILP(std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector< std::tuple<float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::vector< geometry_msgs::Polygon >& _no_fly_zones, const geometry_msgs::Polygon& _geofence, const std::vector< std::vector< std::vector<float> > >& _time_cost_matrices, const std::vector< std::vector< std::vector<float> > >& _battery_drop_matrices);   // Returns new plan.
    // _drone_info it's a vector of tuples, each tuple with 10 elements. The first in the tuple is the initial battery, and so on with all the elements in the "UAV" structure defined here below.

    void printPlan();

private:

    grvc::PathPlanner path_planner_;

    int findUavIndexById(int _UAV_id);

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

    enum struct EdgeType {INSPECTION, NAVIGATION, TAKEOFF_AND_NAVIGATION, NAVIGATION_AND_LANDING};
    struct Edge {
        int i;      // Refered to the time cost and battery drop matrices.
        int j;      // Refered to the time cost and battery drop matrices.
        int k = -1; // Only different that -1 if TAKEOFF_AND_NAVIGATION (and NAVIGATION_AND_LANDING if UAV only can land in its), meaning that only that particular drone k can do that edge.
        EdgeType edge_type = EdgeType::NAVIGATION;
    };
    std::vector<Edge> edges_MILP_;   // Vector of edges possible in which for each pair of nodes (i,j) there is information of whether it is an inspection edge or not. If it is, it still can be used for cross-heading.

    std::map<int, int> from_graph_index_to_matrix_index_;
    std::map<int, int> from_matrix_index_to_graph_index_;
    std::map<int, std::map<int, std::map<int, int>>> from_k_i_j_to_x_index_;
    std::map<int, std::map<int, std::map<int, int>>> from_k_i_j_to_y_and_f_index_;

    std::vector<aerialcore_msgs::FlightPlan> flight_plans_;  // Output.

    void nearestGraphNodeLandStation(int _from_this_index_graph, int& _index_graph_node_to_return, float& _distance_to_return);
    void nearestGraphNodePylon(int _from_this_index_graph, int& _index_graph_node_to_return, float& _distance_to_return);
    void mostRewardedPylon(int _initial_pylon, int& _index_graph_node_to_return, float& _distance_to_return, int& _index_edge_to_erase);

    float batteryDrop(int _flying_time, int _time_max_flying) const { return (float)_flying_time/(float)_time_max_flying; }

};  // end CentralizedPlanner class

}   // end namespace aerialcore

#endif  // CENTRALIZED_PLANNER_H