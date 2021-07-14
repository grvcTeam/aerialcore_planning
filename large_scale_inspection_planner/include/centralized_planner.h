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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <geographic_msgs/GeoPoint.h>
#include <aerialcore_msgs/FlightPlan.h>
#include <aerialcore_msgs/GraphNode.h>
#include <aerialcore_msgs/Edge.h>

#include <path_planner.h>

namespace aerialcore {

/// CentralizedPlanner class that works as interface
class CentralizedPlanner {

public:
    CentralizedPlanner();
    ~CentralizedPlanner();

    std::vector<aerialcore_msgs::FlightPlan> getPlanGreedy() const { return flight_plans_; }     // Returns plan already calculated.
    std::vector<aerialcore_msgs::FlightPlan> getPlanMILP() const { return flight_plans_; }       // Returns plan already calculated.
    std::vector<aerialcore_msgs::FlightPlan> getPlanVNS() const { return flight_plans_; }  // Returns plan already calculated.
    std::vector<aerialcore_msgs::FlightPlan> getPlanMEM() const { return flight_plans_; }  // Returns plan already calculated.

    std::vector<aerialcore_msgs::FlightPlan> getPlanGreedy(std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector< std::tuple<float, float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::vector< geometry_msgs::Polygon >& _no_fly_zones, const geometry_msgs::Polygon& _geofence, const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices, const std::map<int, std::map<int, std::map<int, float> > >& _battery_drop_matrices, std::map<int, int> _last_flight_plan_graph_node); // Returns new plan.
    std::vector<aerialcore_msgs::FlightPlan> getPlanMILP(std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector< std::tuple<float, float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::vector< geometry_msgs::Polygon >& _no_fly_zones, const geometry_msgs::Polygon& _geofence, const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices, const std::map<int, std::map<int, std::map<int, float> > >& _battery_drop_matrices, std::map<int, int> _last_flight_plan_graph_node);   // Returns new plan.
    std::vector<aerialcore_msgs::FlightPlan> getPlanVNS(std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector< std::tuple<float, float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::vector< geometry_msgs::Polygon >& _no_fly_zones, const geometry_msgs::Polygon& _geofence, const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices, const std::map<int, std::map<int, std::map<int, float> > >& _battery_drop_matrices, std::map<int, int> _last_flight_plan_graph_node);    // Returns new plan.
    std::vector<aerialcore_msgs::FlightPlan> getPlanMEM(std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector< std::tuple<float, float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::vector< geometry_msgs::Polygon >& _no_fly_zones, const geometry_msgs::Polygon& _geofence, const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices, const std::map<int, std::map<int, std::map<int, float> > >& _battery_drop_matrices, std::map<int, int> _last_flight_plan_graph_node);    // Returns new plan.
    // _drone_info it's a vector of tuples, each tuple with 10 elements. The first in the tuple is the initial battery, and so on with all the elements in the "UAV" structure defined here below.

    // TODO:
    // 1) Agarwal: Merge-Embed-Merge algorithm (DONE)
    // 2) Online replanning
    // 3) Wind battery and adaptation
    // 4) VNS CTU
    // 5) Dubins for navigations with VTOL or fixed wing, may be needed navigations between non-parallel inspection edges.
    // 6) Compare with similar problems (maybe ask for the code or just do it myself)
    // 7) Problem with paths in the cost matrix (wind doesn't affect the same with paths)

    void printPlan();

    void resetInspectedEdges();

    void fillFlightPlansFields(std::vector<aerialcore_msgs::FlightPlan>& _flight_plans);

private:

    grvc::PathPlanner path_planner_;

    int findUavIndexById(int _UAV_id);

    std::vector<aerialcore_msgs::GraphNode> graph_;

    // UAVs:
    struct UAV {
        float initial_battery;  // Current battery in parts per unit (not percentage or %). Updated continuously.

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

    std::vector< std::pair<int,int> > complete_connection_edges_;   // Edges or connections of the graph (nothing inspected yet), being the pair of indexes of the graph nodes connected by wires. Always the first int index lower than the second of the pair.
    std::vector< std::pair<int,int> > connection_edges_;            // Edges or connections of the graph (some edges inspected, smaller than the complete), being the pair of indexes of the graph nodes connected by wires. Always the first int index lower than the second of the pair.

    std::vector<aerialcore_msgs::Edge> edges_;       // Vector of edges possible in which for each pair of nodes (i,j) there is information of whether it is an inspection edge or not. If it is, it still can be used for cross-heading.

    std::map<int, std::map<int, std::map<int, float> > > time_cost_matrices_;
    std::map<int, std::map<int, std::map<int, float> > > battery_drop_matrices_;

    std::map<int, std::map<int, std::map<int, int>>> from_k_i_j_to_x_index_;
    std::map<int, std::map<int, std::map<int, int>>> from_k_i_j_to_y_and_f_index_;
    int x_index_size_ = 0;
    int y_and_f_index_size_ = 0;

    bool reset_connection_edges_ = true;

    std::vector<aerialcore_msgs::FlightPlan> flight_plans_;  // Output.
    
    geographic_msgs::GeoPoint map_origin_geo_;

    void constructUAVs(const std::vector< std::tuple<float, float, float, int, int, int, int, int, int, bool, bool> >& _drone_info);
    void constructConnectionEdges(const std::vector<aerialcore_msgs::GraphNode>& _graph, std::map<int, int> _last_flight_plan_graph_node);
    void constructEdges(std::vector<aerialcore_msgs::GraphNode>& _graph);

    // Greedy:
    float nearestGraphNodeLandStation(int _from_this_index_graph, int& _index_graph_node_to_return, int _uav_id);
    float nearestGraphNodePylon(int _from_this_index_graph, int& _index_graph_node_to_return, int _uav_id);
    float mostRewardedPylon(int _initial_pylon, int& _index_graph_node_to_return, int& _index_edge_to_erase, int _uav_id);
    float nearestGraphNodeUAVInitialPosition(int _from_this_index_graph, int& _index_graph_node_to_return, int& _uav_id);
    float nearestGraphNodeUAVInitialPositionByTypeString(int _from_this_index_graph, int& _index_graph_node_to_return, int& _uav_id, std::string _type_string);

    float batteryDrop(int _flying_time, int _time_max_flying) const { return (float)_flying_time/(float)_time_max_flying; }

    // Heuristic:
    float solutionTimeCost(std::vector<aerialcore_msgs::FlightPlan> _flight_plans);
    bool solutionBatteryDropValidOrNot(std::vector<aerialcore_msgs::FlightPlan> _flight_plans);
    bool solutionValidOrNot(std::vector<aerialcore_msgs::FlightPlan> _flight_plans);

    struct Tour {
        std::vector<int> nodes;
        float cost;
        float demand;
        int tour_id;
        int uav_id;
    };
    struct Saving {
        std::vector<int> nodes;
        float cost;
        float demand;

        float saving = -1;   // Will keep this value if empty saving.
        int tour_id_1;
        int tour_id_2;

        int uav_id;
    };

    int findTourIndexById(int _tour_id, const std::vector<Tour>& _R_tours_array);

    void calculateSaving(Saving& _saving, const std::vector<Tour>& _R_tours_array);
    Saving bestSavingPermutation(const Saving& _saving_input, int _i, int _j, const std::vector<Tour>& _R_tours_array);

    std::vector<aerialcore_msgs::FlightPlan> buildFlightPlansFromTours(const std::vector<Tour>& _R_tours_array);

    std::string uavTypeStringByIndex(int _uav_index);
    std::string uavTypeStringById(int _uav_id);

};  // end CentralizedPlanner class

}   // end namespace aerialcore

#endif  // CENTRALIZED_PLANNER_H