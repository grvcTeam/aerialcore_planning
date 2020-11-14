/**
 * AERIALCORE Project:
 *
 * Centralized planner.
 * 
 */

#include <centralized_planner.h>

#include <math.h>

namespace aerialcore {


// Brief Constructor
CentralizedPlanner::CentralizedPlanner() : path_planner_() {}


// Brief Destructor
CentralizedPlanner::~CentralizedPlanner() {}


std::vector<aerialcore_msgs::FlightPlan> CentralizedPlanner::getPlan(const std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector< std::tuple<float, float, int, int, int, int, int, int, bool, bool> >& _drone_info) {

    graph_.clear();
    edges_.clear();
    UAVs_.clear();
    flight_plan_.clear();

    graph_ = _graph;

    // _drone_info it's a vector of tuples, each tuple with 10 elements. The first in the tuple is the initial battery, and so on with all the elements in the "UAV" structure defined here below.
    for (const std::tuple<float, float, int, int, int, int, int, int, bool, bool>& current_drone : _drone_info) {
        UAV actual_UAV;
        actual_UAV.initial_battery = std::get<0>(current_drone);
        actual_UAV.minimum_battery = std::get<1>(current_drone);
        actual_UAV.time_until_fully_charged = std::get<2>(current_drone);
        actual_UAV.time_max_flying = std::get<3>(current_drone);
        actual_UAV.speed_xy = std::get<4>(current_drone);
        actual_UAV.speed_z_down = std::get<5>(current_drone);
        actual_UAV.speed_z_up = std::get<6>(current_drone);
        actual_UAV.id = std::get<7>(current_drone);
        actual_UAV.flying_or_landed_initially = std::get<8>(current_drone);
        actual_UAV.recharging_initially = std::get<9>(current_drone);
        UAVs_.push_back(actual_UAV);
    }

    // Construct edges_:
    for (int i=0; i<graph_.size(); i++) {
        if (graph_[i].type == aerialcore_msgs::GraphNode::TYPE_PYLON) {
            for (int j=0; j<graph_[i].connections_indexes.size(); j++) {
                if (i < graph_[i].connections_indexes[j]) {
                    std::pair <int,int> current_edge (i, graph_[i].connections_indexes[j]);
                    edges_.push_back(current_edge);
                }
            }
        }
    }

    for (const UAV& current_uav : UAVs_) {

        // GREEDY PLANNER:
        // First heuristic greedy approach. This algorithm is centralized and without replanning.
        // Each UAV selects its segments to inspect (graph edges) sequentially. 
        // The UAV selects the closest pylon to its depot, and then, those segments with greater profit (larger wires to inspect).
        // Repeat until the battery left is just enough to return to a depot.
        // Depots may have a charging pad to automatically recharge batteries.
        // Assigned segments are removed from the graph for next UAVs.

        aerialcore_msgs::FlightPlan current_flight_plan;
        current_flight_plan.uav_id = current_uav.id;

        // Find the initial position of the drone in the graph and insert it as the first node of the flight plan:
        int uav_initial_position_graph_index = -1;
        for (int i=0; i<graph_.size(); i++) {
            if (graph_[i].id == current_uav.id) {
                uav_initial_position_graph_index = i;
                break;
            }
        }
        if (uav_initial_position_graph_index == -1) {
            ROS_ERROR("Centralized Planner: UAV id provided in _drone_info not found in _graph.");
            exit(EXIT_FAILURE);
        }
        current_flight_plan.nodes.push_back(uav_initial_position_graph_index);

        float battery = current_uav.initial_battery;

        int index_graph_of_next_pylon;
        float distance_of_next_pylon;

        int index_graph_land_station_from_next_pylon;
        int index_graph_land_station_from_pylon_last;
        float distance_land_station_from_next_pylon;

        // Calculate closest pylon from the UAV initial pose:
        nearestGraphNode(true, uav_initial_position_graph_index, index_graph_of_next_pylon, distance_of_next_pylon);

        // Calculate closest land station from that closest pylon:
        nearestGraphNode(false, index_graph_of_next_pylon, index_graph_land_station_from_next_pylon, distance_land_station_from_next_pylon);

        while (edges_.size()>0 && (battery - batteryDrop( (distance_of_next_pylon+distance_land_station_from_next_pylon)/current_uav.speed_xy , current_uav.time_max_flying) > current_uav.minimum_battery)) {
            // Insert pylon because it can be reached with enough battery to go later to a land station:
            current_flight_plan.nodes.push_back(index_graph_of_next_pylon);

            // Update the battery left:            
            battery -= batteryDrop( distance_of_next_pylon/current_uav.speed_xy , current_uav.time_max_flying);

            // Calculate pylon with most benefit from current pylon:
            mostRewardedPylon(current_flight_plan.nodes.back(), index_graph_of_next_pylon, distance_of_next_pylon);

// TODO in process: RIGHT NOW IT'S NOT NAVIGATING WITHOUT WIRES.
// TODO: RIGHT NOW IT'S TAKEOFF ALTITUDE TOO LOW.
            if (index_graph_of_next_pylon == -1) break; // No next pylon with unserved edges.
            // if (index_graph_of_next_pylon == -1) {      // No next pylon with unserved edges from this pylon.
            //     // Calculate closest pylon from the UAV initial pose:
            //     nearestGraphNode(true, index_graph_of_next_pylon, index_graph_of_next_pylon, distance_of_next_pylon);

            //     if (index_graph_of_next_pylon == -1) {  // No next pylon in general with unserved edges.
            //         break;
            //     }

            //     // Calculate pylon with most benefit from current pylon:
            //     mostRewardedPylon(current_flight_plan.nodes.back(), index_graph_of_next_pylon, distance_of_next_pylon);
            // }

            index_graph_land_station_from_pylon_last = index_graph_land_station_from_next_pylon;

            // Calculate closest land station from that most rewarded pylon:
            nearestGraphNode(false, index_graph_of_next_pylon, index_graph_land_station_from_next_pylon, distance_land_station_from_next_pylon);
        }
        // Insert closest land station when inserting another pylon would result in low battery:
        current_flight_plan.nodes.push_back(index_graph_land_station_from_pylon_last);

        if (current_flight_plan.nodes.size()>3) {   // Consider the flight plan only if it visits two or more pylons.
            flight_plan_.push_back(current_flight_plan);
        }
    }

    return flight_plan_;

} // end getPlan method


// Brief method that returns the index and distance of the nearest graph node given an inital point. _pylon_or_land_station true returns the nearest pylon, _pylon_or_land_station false returns the nearest land station.
void CentralizedPlanner::nearestGraphNode(bool _pylon_or_land_station, int _from_this_index_graph, int& _index_graph_node_to_return, float& _distance_to_return) {

    _index_graph_node_to_return = -1;
    _distance_to_return = std::numeric_limits<float>::max();

    geometry_msgs::PointStamped from_here;
    from_here.point.x = graph_[_from_this_index_graph].x;
    from_here.point.y = graph_[_from_this_index_graph].y;

    for (int i=0; i<graph_.size(); i++) {
        if ( !_pylon_or_land_station&&(graph_[i].type==aerialcore_msgs::GraphNode::TYPE_RECHARGE_LAND_STATION || graph_[i].type==aerialcore_msgs::GraphNode::TYPE_REGULAR_LAND_STATION) || _pylon_or_land_station&&graph_[i].type==aerialcore_msgs::GraphNode::TYPE_PYLON) {

            geometry_msgs::PointStamped to_here;
            to_here.point.x = graph_[i].x;
            to_here.point.y = graph_[i].y;

            auto path = path_planner_.getPath(from_here, to_here);
            if (path.size()==0) continue;       // No path found.
            float distance_xy = path_planner_.getFlatDistance();
            if (distance_xy==0) continue;       // It's the same graph node, ignore and continue.

            if ( _distance_to_return > distance_xy ) {
                _index_graph_node_to_return = i;
                _distance_to_return = distance_xy;
            }
        }
    }

} // end nearestGraphNode method


// Brief method that returns the index and distance of the furthest connected pylon from an initial pylon.
void CentralizedPlanner::mostRewardedPylon(int _initial_pylon_index, int& _index_graph_node_to_return, float& _distance_to_return) {

    _index_graph_node_to_return = -1;
    _distance_to_return = 0;
    int index_edge_to_erase;

    if (!graph_[_initial_pylon_index].type==aerialcore_msgs::GraphNode::TYPE_PYLON) {
        return;
    }

    for (const int& current_connection_index : graph_[_initial_pylon_index].connections_indexes) {

        // Calculate the current edge between both pylons:
        std::pair <int,int> current_edge;
        current_edge.first =  _initial_pylon_index < current_connection_index ? _initial_pylon_index : current_connection_index;
        current_edge.second = _initial_pylon_index < current_connection_index ? current_connection_index : _initial_pylon_index;

        // If current edge not found means it has already been served, so continue.
        bool edge_found = false;
        int i = 0;
        for (i=0; i<edges_.size(); i++) {
            if (edges_[i] == current_edge) {
                edge_found = true;
                break;
            }
        }
        if (!edge_found) {
            continue;
        }

        float distance_xy = sqrt( pow(graph_[_initial_pylon_index].x-graph_[current_connection_index].x, 2) + pow(graph_[_initial_pylon_index].y-graph_[current_connection_index].y, 2) );

        if ( _distance_to_return < distance_xy ) {
            _index_graph_node_to_return = current_connection_index;
            _distance_to_return = distance_xy;
            index_edge_to_erase = i;
        }
    }
    if (_index_graph_node_to_return != -1) {
        edges_.erase( edges_.begin() + index_edge_to_erase );
    }

} // end mostRewardedPylon method


// Brief method to print the plan in the terminal.
void CentralizedPlanner::printPlan() {
    std::cout << std::endl;
    std::cout << "Printing flight plan from the planner:" << std::endl;
    std::cout << std::endl;
    for (int i=0; i<flight_plan_.size(); i++) {
        std::cout << "flight_plan[ " << i << " ].id = " << flight_plan_[i].uav_id << std::endl;
        for (int j=0; j<flight_plan_[i].nodes.size(); j++) {
            std::cout << "flight_plan[ " << i << " ].nodes[ " << j << " ] = " << flight_plan_[i].nodes[j] << std::endl;
        }
    }
    std::cout << std::endl;
} // end printPlan method


} // end namespace aerialcore
