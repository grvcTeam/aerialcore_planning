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
#include <unordered_map>
#include <tuple>
#include <utility>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
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

    // Regular inspection getters:

    std::vector<aerialcore_msgs::FlightPlan> getPlanGreedy() const { return flight_plans_; }    // Returns plan already calculated.
    std::vector<aerialcore_msgs::FlightPlan> getPlanMILP() const { return flight_plans_; }      // Returns plan already calculated.
    std::vector<aerialcore_msgs::FlightPlan> getPlanVNS() const { return flight_plans_; }       // Returns plan already calculated.
    std::vector<aerialcore_msgs::FlightPlan> getPlanMEM() const { return flight_plans_; }       // Returns plan already calculated.

    std::vector<aerialcore_msgs::FlightPlan> getPlanGreedy(std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector< std::tuple<float, float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::vector< geometry_msgs::Polygon >& _no_fly_zones, const geometry_msgs::Polygon& _geofence, const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices, const std::map<int, std::map<int, std::map<int, float> > >& _battery_drop_matrices, std::map<int, int> _last_flight_plan_graph_node); // Returns new plan.
    std::vector<aerialcore_msgs::FlightPlan> getPlanMILP(std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector< std::tuple<float, float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::vector< geometry_msgs::Polygon >& _no_fly_zones, const geometry_msgs::Polygon& _geofence, const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices, const std::map<int, std::map<int, std::map<int, float> > >& _battery_drop_matrices, std::map<int, int> _last_flight_plan_graph_node);   // Returns new plan.
    std::vector<aerialcore_msgs::FlightPlan> getPlanVNS(std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector< std::tuple<float, float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::vector< geometry_msgs::Polygon >& _no_fly_zones, const geometry_msgs::Polygon& _geofence, const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices, const std::map<int, std::map<int, std::map<int, float> > >& _battery_drop_matrices, std::map<int, int> _last_flight_plan_graph_node);    // Returns new plan.
    std::vector<aerialcore_msgs::FlightPlan> getPlanMEM(std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector< std::tuple<float, float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::vector< geometry_msgs::Polygon >& _no_fly_zones, const geometry_msgs::Polygon& _geofence, const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices, const std::map<int, std::map<int, std::map<int, float> > >& _battery_drop_matrices, std::map<int, int> _last_flight_plan_graph_node);    // Returns new plan.
    // _drone_info it's a vector of tuples, each tuple with 10 elements. The first in the tuple is the initial battery, and so on with all the elements in the "UAV" structure defined here below.

    // Fast electric fault search getters:

    std::vector<aerialcore_msgs::FlightPlan> getPlanMinimaxGreedy() const { return flight_plans_; }    // Returns plan already calculated.
    std::vector<aerialcore_msgs::FlightPlan> getPlanMinimaxMEM() const { return flight_plans_; }    // Returns plan already calculated.
    std::vector<aerialcore_msgs::FlightPlan> getPlanMinimaxVNS() const { return flight_plans_; }    // Returns plan already calculated.

    std::vector<aerialcore_msgs::FlightPlan> getPlanMinimaxGreedy(std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector< std::tuple<float, float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::vector< geometry_msgs::Polygon >& _no_fly_zones, const geometry_msgs::Polygon& _geofence, const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices, const std::map<int, std::map<int, std::map<int, float> > >& _battery_drop_matrices, std::map<int, int> _last_flight_plan_graph_node); // Returns new plan.
    std::vector<aerialcore_msgs::FlightPlan> getPlanMinimaxMEM(std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector< std::tuple<float, float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::vector< geometry_msgs::Polygon >& _no_fly_zones, const geometry_msgs::Polygon& _geofence, const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices, const std::map<int, std::map<int, std::map<int, float> > >& _battery_drop_matrices, std::map<int, int> _last_flight_plan_graph_node); // Returns new plan.
    std::vector<aerialcore_msgs::FlightPlan> getPlanMinimaxVNS(std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector< std::tuple<float, float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::vector< geometry_msgs::Polygon >& _no_fly_zones, const geometry_msgs::Polygon& _geofence, const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices, const std::map<int, std::map<int, std::map<int, float> > >& _battery_drop_matrices, std::map<int, int> _last_flight_plan_graph_node); // Returns new plan.

    // TODO:
    // 1) Agarwal: Merge-Embed-Merge algorithm. (DONE)
    // 2) Online replanning. (DONE)
    // 3) Wind affects battery consumption:
    //  3.1) Multicopter. (DONE)
    //  3.2) VTOL-fixed wing.
    // 4) Battery faker:
    //  4.1) Multicopter. (DONE)
    //  4.2) VTOL and fixed wing.
    // 5) Wind input for the planner.
    //  5.1) Wind known or wind sensor. (DONE, from Internet API weather)
    //  5.2) Wind from inversed consumption.
    //  5.3) Wind from UAVs angles (depends on airframe type).
    // 6) VNS (DONE).
    // 7) Minimax.
    //  7.1) MEM.
    //  7.2) VNS (DONE).
    // 8) Dubins for navigations with VTOL or fixed wing, may be needed navigations between non-parallel inspection edges.
    // 9) Compare with similar problems (maybe ask for the code or just do it myself).
    // 10) Problem with paths in the cost matrix (wind doesn't affect the same with paths).

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

    struct Edge {
        int i;      // Refered to the time cost and battery drop matrices.
        int j;      // Refered to the time cost and battery drop matrices.
        int k;      // UAV id. Only different that -1 if TAKEOFF_AND_NAVIGATION (and NAVIGATION_AND_LANDING if UAV only can land in its), meaning that only that particular drone k can do that edge.

        bool operator<(const Edge& other) const {     // This struct will be used as keys of maps, so it's necessary to define the operator "<" for the map sorting.
            return edge2String(*this) < edge2String(other);
        }

        inline std::string edge2String(const Edge& edge) const {
            std::string output = std::to_string(edge.i).c_str();
            output.append(std::to_string(edge.j).c_str());
            output.append(std::to_string(edge.k).c_str());
            return output;
        }
    };
    enum struct EdgeType {TYPE_INSPECTION, TYPE_NAVIGATION, TYPE_TAKEOFF_AND_NAVIGATION, TYPE_NAVIGATION_AND_LANDING};
    std::map<Edge, EdgeType> edges_;       // Vector of edges possible in which for each pair of nodes (i,j) there is information of whether it is an inspection edge or not. If it is, it still can be used for cross-heading.

    std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > > time_cost_matrices_;
    std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > > battery_drop_matrices_;

    std::map<int, std::map<int, std::map<int, int>>> from_k_i_j_to_x_index_;
    std::map<int, std::map<int, std::map<int, int>>> from_k_i_j_to_y_and_f_index_;
    int x_index_size_ = 0;
    int y_and_f_index_size_ = 0;

    bool reset_connection_edges_ = true;

    bool regular_or_fast_inspection_ = true;    // True if regular inspection, false if fast inspection searching for electric fault (minimax).
    float proportional_battery_ = -1;       // If fast inspection minimax (regular_or_fast_inspection_==false), this is the minimum battery that each UAV can have inspecting with the greedy minimax solver.

    std::vector<aerialcore_msgs::FlightPlan> flight_plans_;  // Output.

    geographic_msgs::GeoPoint map_origin_geo_;

    void constructUAVs(const std::vector< std::tuple<float, float, float, int, int, int, int, int, int, bool, bool> >& _drone_info);
    void constructConnectionEdges(const std::vector<aerialcore_msgs::GraphNode>& _graph, std::map<int, int> _last_flight_plan_graph_node);
    void constructEdges(std::vector<aerialcore_msgs::GraphNode>& _graph);
    void constructUnordenedMaps(const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices, const std::map<int, std::map<int, std::map<int, float> > >& _battery_drop_matrices);

    // Greedy:
    float nearestGraphNodeLandStation(int _from_this_index_graph, int& _index_graph_node_to_return, int _uav_id);
    float nearestGraphNodePylon(int _from_this_index_graph, int& _index_graph_node_to_return, int _uav_id);
    float mostRewardedPylon(int _initial_pylon, int& _index_graph_node_to_return, int& _index_edge_to_erase, int _uav_id);
    float nearestGraphNodeUAVInitialPosition(int _from_this_index_graph, int& _index_graph_node_to_return, int& _uav_id);

    float batteryDrop(int _flying_time, int _time_max_flying) const { return (float)_flying_time/(float)_time_max_flying; }

    // VNS heuristic:
    float solutionTimeCostTotal(const std::vector<aerialcore_msgs::FlightPlan>& _flight_plans);
    float solutionTimeCostMaximum(const std::vector<aerialcore_msgs::FlightPlan>& _flight_plans);
    bool solutionValidOrNot(const std::vector<aerialcore_msgs::FlightPlan>& _flight_plans);
    void shake(std::vector<aerialcore_msgs::FlightPlan>& _flight_plans);
    void localSearch(std::vector<aerialcore_msgs::FlightPlan>& _flight_plans);
    std::vector< std::vector< std::pair<int,int> > > buildPlannedInspectionEdgesFromNodes(const std::vector<aerialcore_msgs::FlightPlan>& _flight_plans);   // Edges output same size as _flight_plans.
    std::vector<aerialcore_msgs::FlightPlan> buildPlanNodesFromInspectionEdges(const std::vector< std::vector< std::pair<int,int> > >& _planned_inspection_edges, const std::vector<aerialcore_msgs::FlightPlan>& _flight_plans);

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

    // VNS parameters:
    float maximum_time_without_improvement_ = 1;    // Stop condition for the VNS algorithm. Seconds that the VNS will keep trying to improve a solution before giving up.
    float local_search_iteration_multiplier_ = 1;   // This number multiplied for the graph nodes matrix size will be the number of times that the local search of the VNS will be repeated in order to achieve the local minuma.

    // Numerical experiments:
    double time_computing_plan_ = 0;                    // seconds
    double accumulated_time_checking_solutions_ = 0;    // seconds

};  // end CentralizedPlanner class

}   // end namespace aerialcore

#endif  // CENTRALIZED_PLANNER_H