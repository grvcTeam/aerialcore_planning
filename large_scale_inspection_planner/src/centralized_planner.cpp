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


std::vector<aerialcore_msgs::FlightPlan> CentralizedPlanner::getPlan(std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector< std::tuple<float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::vector< geometry_msgs::Polygon >& _no_fly_zones, const geometry_msgs::Polygon& _geofence) {

    graph_.clear();
    edges_.clear();
    UAVs_.clear();
    flight_plan_.clear();

    graph_ = _graph;

    if (_no_fly_zones.size()>0) {
        path_planner_ = multidrone::PathPlanner(_no_fly_zones, _geofence);
    }

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

        int index_edge_to_erase = -1;

        int index_graph_land_station_from_next_pylon;
        int index_graph_land_station_from_pylon_last;
        float distance_land_station_from_next_pylon;

        // Calculate closest pylon from the UAV initial pose:
        nearestGraphNodePylon(uav_initial_position_graph_index, index_graph_of_next_pylon, distance_of_next_pylon);

        // Calculate closest land station from that closest pylon:
        nearestGraphNodeLandStation(index_graph_of_next_pylon, index_graph_land_station_from_next_pylon, distance_land_station_from_next_pylon);

        while (edges_.size()>0 && (battery - batteryDrop( (distance_of_next_pylon+distance_land_station_from_next_pylon)/current_uav.speed_xy , current_uav.time_max_flying) > current_uav.minimum_battery)) {
            // Insert pylon because it can be reached with enough battery to go later to a land station:
            current_flight_plan.nodes.push_back(index_graph_of_next_pylon);

            if (index_edge_to_erase != -1) {
                edges_.erase( edges_.begin() + index_edge_to_erase );
            }

            // Update the battery left:            
            battery -= batteryDrop( distance_of_next_pylon/current_uav.speed_xy , current_uav.time_max_flying);

            // Calculate pylon with most benefit from current pylon:
            mostRewardedPylon(current_flight_plan.nodes.back(), index_graph_of_next_pylon, distance_of_next_pylon, index_edge_to_erase);

            index_graph_land_station_from_pylon_last = index_graph_land_station_from_next_pylon;

            if (index_graph_of_next_pylon == -1) { // No next pylon connected with unserved edges, search pylons not connected to this one.
                int index_pylon_connected_with_unserved_edge;
                float distance_pylon_connected_with_unserved_edge;
                nearestGraphNodePylon(current_flight_plan.nodes.back(), index_graph_of_next_pylon, distance_of_next_pylon);
                if (index_graph_of_next_pylon == -1) break; // No next pylon at all with unserved edges.
                else {  // Pylon has edges unserved connected.
                    mostRewardedPylon(index_graph_of_next_pylon, index_pylon_connected_with_unserved_edge, distance_pylon_connected_with_unserved_edge, index_edge_to_erase);
                    nearestGraphNodeLandStation(index_pylon_connected_with_unserved_edge, index_graph_land_station_from_next_pylon, distance_land_station_from_next_pylon);
                    // TODO: check if points are inside obstacles before calculating paths.
                    if ( (index_pylon_connected_with_unserved_edge!=-1) && (index_graph_land_station_from_next_pylon!=-1) && (battery - batteryDrop( (distance_of_next_pylon+distance_pylon_connected_with_unserved_edge+distance_land_station_from_next_pylon)/current_uav.speed_xy , current_uav.time_max_flying) > current_uav.minimum_battery) ) {
                        current_flight_plan.nodes.push_back(index_graph_of_next_pylon);
                        battery -= batteryDrop( distance_of_next_pylon/current_uav.speed_xy , current_uav.time_max_flying);
                        index_graph_of_next_pylon = index_pylon_connected_with_unserved_edge;
                        distance_of_next_pylon = distance_pylon_connected_with_unserved_edge;
                    } else {    // Only insert the next pylon if it has battery to at least fulfill one edge.
                        break;
                    }
                }
            } else {
                // Calculate closest land station from that most rewarded pylon:
                nearestGraphNodeLandStation(index_graph_of_next_pylon, index_graph_land_station_from_next_pylon, distance_land_station_from_next_pylon);
            }
        }
        // Insert closest land station when inserting another pylon would result in low battery:
        current_flight_plan.nodes.push_back(index_graph_land_station_from_pylon_last);

        if (current_flight_plan.nodes.size()>3) {   // Consider the flight plan only if it visits two or more pylons.
            flight_plan_.push_back(current_flight_plan);
        }
    }

    // TODO: actually add TYPE_PASS_WP_AVOIDING_NO_FLY_ZONE to the graph.

    return flight_plan_;

} // end getPlan method


// Brief method that returns the index and distance of the nearest land station graph node given an inital index.
void CentralizedPlanner::nearestGraphNodeLandStation(int _from_this_index_graph, int& _index_graph_node_to_return, float& _distance_to_return) {

    _index_graph_node_to_return = -1;
    _distance_to_return = std::numeric_limits<float>::max();

    geometry_msgs::PointStamped from_here;
    from_here.point.x = graph_[_from_this_index_graph].x;
    from_here.point.y = graph_[_from_this_index_graph].y;

    for (int i=0; i<graph_.size(); i++) {
        if (graph_[i].type==aerialcore_msgs::GraphNode::TYPE_RECHARGE_LAND_STATION || graph_[i].type==aerialcore_msgs::GraphNode::TYPE_REGULAR_LAND_STATION) {

            geometry_msgs::PointStamped to_here;
            to_here.point.x = graph_[i].x;
            to_here.point.y = graph_[i].y;

            auto path = path_planner_.getPath(from_here, to_here);
            if (path.size() == 0) continue;         // No path found.
            float distance_xy = path_planner_.getFlatDistance();
            if (distance_xy <= 0.001) continue;     // It's the same graph node, ignore and continue.

            if ( _distance_to_return > distance_xy ) {
                _index_graph_node_to_return = i;
                _distance_to_return = distance_xy;
            }
        }
    }

} // end nearestGraphNodeLandStation method


// Brief method that returns the index and distance of the nearest pylon graph node given an inital index. The new pylon will have edges unserved and will not be directly connected.
void CentralizedPlanner::nearestGraphNodePylon(int _from_this_index_graph, int& _index_graph_node_to_return, float& _distance_to_return) {

    _index_graph_node_to_return = -1;
    _distance_to_return = std::numeric_limits<float>::max();

    geometry_msgs::PointStamped from_here;
    from_here.point.x = graph_[_from_this_index_graph].x;
    from_here.point.y = graph_[_from_this_index_graph].y;

    for (int i=0; i<graph_.size(); i++) {
        if (graph_[i].type==aerialcore_msgs::GraphNode::TYPE_PYLON) {

            // If i (current node iterated) is connected to the original pylon index ignore it.
            bool continue_flag = false;
            for (const int& current_connection_index : graph_[_from_this_index_graph].connections_indexes) {
                if (current_connection_index==i) {
                    continue_flag = true;
                    break;
                }
            }
            if (continue_flag) {
                continue;
            }

            geometry_msgs::PointStamped to_here;
            to_here.point.x = graph_[i].x;
            to_here.point.y = graph_[i].y;

            auto path = path_planner_.getPath(from_here, to_here);
            if (path.size() == 0) continue;         // No path found.
            float distance_xy = path_planner_.getFlatDistance();
            if (distance_xy <= 0.001) continue;     // It's the same graph node, ignore and continue.

            // Only consider i (current node iterated) if it has edges unserved:
            bool has_edges_unserved = false;
            for (const int& current_connection_index : graph_[i].connections_indexes) {
                std::pair <int,int> current_edge;
                current_edge.first =  i < current_connection_index ? i : current_connection_index;
                current_edge.second = i < current_connection_index ? current_connection_index : i;
                for (int j=0; j<edges_.size(); j++) {
                    if (edges_[j] == current_edge) {
                        has_edges_unserved = true;
                        break;
                    }
                }
            }
            if (!has_edges_unserved) {
                continue;
            }

            if ( _distance_to_return > distance_xy ) {
                _index_graph_node_to_return = i;
                _distance_to_return = distance_xy;
            }
        }
    }

} // end nearestGraphNodePylon method


// Brief method that returns the index and distance of the furthest connected pylon from an initial pylon.
void CentralizedPlanner::mostRewardedPylon(int _initial_pylon_index, int& _index_graph_node_to_return, float& _distance_to_return, int& _index_edge_to_erase) {

    _index_graph_node_to_return = -1;
    _distance_to_return = 0;
    _index_edge_to_erase = -1;

    geometry_msgs::PointStamped from_here;
    from_here.point.x = graph_[_initial_pylon_index].x;
    from_here.point.y = graph_[_initial_pylon_index].y;

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

        geometry_msgs::PointStamped to_here;
        to_here.point.x = graph_[current_connection_index].x;
        to_here.point.y = graph_[current_connection_index].y;

        auto path = path_planner_.getPath(from_here, to_here);
        if (path.size() == 0) continue;         // No path found.
        float distance_xy = path_planner_.getFlatDistance();
        if (distance_xy <= 0.001) continue;     // It's the same graph node, ignore and continue.

        if ( _distance_to_return < distance_xy ) {
            _index_graph_node_to_return = current_connection_index;
            _distance_to_return = distance_xy;
            _index_edge_to_erase = i;
        }
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
