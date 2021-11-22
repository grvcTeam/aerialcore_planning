/**
 * AERIALCORE Project:
 *
 * Centralized planner.
 * 
 */

#include <centralized_planner.h>

#include <ros/ros.h>
#include <math.h>
#include <stdlib.h>
#include <algorithm>

#include "ortools/linear_solver/linear_solver.h"
#include "ortools/base/logging.h"

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

#define DEBUG       // UNCOMMENT FOR PRINTING VISUALIZATION OF RESULTS (DEBUG MODE)
#define NUMERICAL_EXPERIMENTS

namespace aerialcore {


// Brief Constructor
CentralizedPlanner::CentralizedPlanner() {
    ros::NodeHandle nh;

    // map_origin_geo is the geographic coordinate origin of the cartesian coordinates. Loaded to the param server in the YAML config file.
    std::vector<double> map_origin_geo_vector;
    nh.getParam("map_origin_geo", map_origin_geo_vector);
    map_origin_geo_.latitude  = map_origin_geo_vector[0];
    map_origin_geo_.longitude = map_origin_geo_vector[1];
    map_origin_geo_.altitude  = map_origin_geo_vector[2];

    nh.getParam("maximum_time_without_improvement", maximum_time_without_improvement_);
    nh.getParam("local_search_iteration_multiplier", local_search_iteration_multiplier_);

    srand(std::time(nullptr));
}


// Brief Destructor
CentralizedPlanner::~CentralizedPlanner() {}


std::vector<aerialcore_msgs::FlightPlan> CentralizedPlanner::getPlanGreedy(std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector< std::tuple<float, float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::vector< geometry_msgs::Polygon >& _no_fly_zones, const geometry_msgs::Polygon& _geofence, const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices, const std::map<int, std::map<int, std::map<int, float> > >& _battery_drop_matrices, std::map<int, int> _last_flight_plan_graph_node) {

    graph_.clear();
    edges_.clear();
    time_cost_matrices_.clear();
    battery_drop_matrices_.clear();

    graph_ = _graph;

    constructUnordenedMaps(_time_cost_matrices, _battery_drop_matrices);

    if (_geofence.points.size()>0 && _no_fly_zones.size()>0) {
        std::vector< std::vector<geographic_msgs::GeoPoint> > obstacle_polygon_vector_geo;
        std::vector<geographic_msgs::GeoPoint> geofence_polygon_geo;

        for (geometry_msgs::Polygon no_fly_zone: _no_fly_zones) {
            std::vector<geographic_msgs::GeoPoint> no_fly_zone_geo;
            for (geometry_msgs::Point32 nfz_point: no_fly_zone.points) {
                geographic_msgs::GeoPoint nfz_point_geo = cartesian_to_geographic(nfz_point, map_origin_geo_);
                no_fly_zone_geo.push_back(nfz_point_geo);
            }
            obstacle_polygon_vector_geo.push_back(no_fly_zone_geo);
        }

        for (geometry_msgs::Point32 geofence_point: _geofence.points) {
            geographic_msgs::GeoPoint nfz_point_geo = cartesian_to_geographic(geofence_point, map_origin_geo_);
            geofence_polygon_geo.push_back(nfz_point_geo);
        }

        path_planner_ = grvc::PathPlanner(obstacle_polygon_vector_geo, geofence_polygon_geo);
    }

    constructUAVs(_drone_info);

    constructConnectionEdges(_graph, _last_flight_plan_graph_node);
    flight_plans_.clear();  // Need to be later than constructConnectionEdges()

    if (!regular_or_fast_inspection_) {
        // Minimax greedy assigns to each UAV a proportional part (in battery terms) of the graph to inspect (the last UAV can be assigned more until graph completely inspected):
        float total_battery_to_inspect_times_UAVs = 0;
        for (const auto& wired_edge : connection_edges_) {
            for (const UAV& current_uav : UAVs_) {
                total_battery_to_inspect_times_UAVs += battery_drop_matrices_.at(current_uav.id).at( wired_edge.first ).at( wired_edge.second );
            }
        }
        proportional_battery_ = 1 - total_battery_to_inspect_times_UAVs/UAVs_.size();
    } else {
        proportional_battery_ = -1;
    }

    // constructEdges(_graph);

    for (int i=0; i<UAVs_.size(); i++) {

        // GREEDY PLANNER:
        // First heuristic greedy approach. This algorithm is centralized and without replanning.
        // Each UAV selects its segments to inspect (graph edges) sequentially. 
        // The UAV selects the closest pylon to its depot, and then, those segments with greater profit (larger wires to inspect).
        // Repeat until the battery left is just enough to return to a depot.
        // Depots may have a charging pad to automatically recharge batteries.
        // Assigned segments are removed from the graph for next UAVs.

        aerialcore_msgs::FlightPlan current_flight_plan;
        current_flight_plan.uav_id = UAVs_[i].id;

        // Find the initial position of the drone in the graph and insert it as the first node of the flight plan:
        int uav_initial_position_graph_index = -1;
        for (int j=0; j<graph_.size(); j++) {
            if (graph_[j].type == aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION && graph_[j].id == UAVs_[i].id) {
                uav_initial_position_graph_index = j;
                break;
            }
        }
        if (uav_initial_position_graph_index == -1) {
            ROS_ERROR("Centralized Planner: UAV id provided in _drone_info not found in _graph.");
            exit(EXIT_FAILURE);
        }
        current_flight_plan.nodes.push_back(uav_initial_position_graph_index);

        float battery = UAVs_[i].initial_battery;

        int index_graph_of_next_pylon;

        int index_edge_to_erase = -1;

        int index_graph_land_station_from_next_pylon;
        int index_graph_land_station_from_pylon_last;

        // Calculate closest pylon from the UAV initial pose:
        nearestGraphNodePylon(uav_initial_position_graph_index, index_graph_of_next_pylon, UAVs_[i].id);

        // Calculate closest land station from that closest pylon:
        nearestGraphNodeLandStation(index_graph_of_next_pylon, index_graph_land_station_from_next_pylon, UAVs_[i].id);

        while (connection_edges_.size()>0 && (battery
            - battery_drop_matrices_[ UAVs_[i].id ][ uav_initial_position_graph_index ][ index_graph_of_next_pylon ]
            - battery_drop_matrices_[ UAVs_[i].id ][ index_graph_of_next_pylon ][ index_graph_land_station_from_next_pylon ] >= UAVs_[i].minimum_battery) &&
            (regular_or_fast_inspection_ || (i==UAVs_.size()-1) || (battery - battery_drop_matrices_[ UAVs_[i].id ][ uav_initial_position_graph_index ][ index_graph_of_next_pylon ] - battery_drop_matrices_[ UAVs_[i].id ][ index_graph_of_next_pylon ][ index_graph_land_station_from_next_pylon ] >= proportional_battery_)) ) {

            // Update the battery left:
            battery -= battery_drop_matrices_[ UAVs_[i].id ][ current_flight_plan.nodes.back() ][ index_graph_of_next_pylon ];

            // Insert pylon because it can be reached with enough battery to go later to a land station:
            current_flight_plan.nodes.push_back(index_graph_of_next_pylon);

            if (index_edge_to_erase != -1) {
                connection_edges_.erase( connection_edges_.begin() + index_edge_to_erase );
            }

            // Calculate pylon with most benefit from current pylon:
            mostRewardedPylon(current_flight_plan.nodes.back(), index_graph_of_next_pylon, index_edge_to_erase, UAVs_[i].id);

            index_graph_land_station_from_pylon_last = index_graph_land_station_from_next_pylon;

            if (index_graph_of_next_pylon == -1) { // No next pylon connected with unserved edges, search pylons not connected to this one.
                int index_pylon_connected_with_unserved_edge;
                nearestGraphNodePylon(current_flight_plan.nodes.back(), index_graph_of_next_pylon, UAVs_[i].id);
                if (index_graph_of_next_pylon == -1) {
                    break; // No next pylon at all with unserved edges.
                } else {   // Pylon has edges unserved connected.
                    mostRewardedPylon(index_graph_of_next_pylon, index_pylon_connected_with_unserved_edge, index_edge_to_erase, UAVs_[i].id);
                    nearestGraphNodeLandStation(index_pylon_connected_with_unserved_edge, index_graph_land_station_from_next_pylon, UAVs_[i].id);
                    if ( (index_pylon_connected_with_unserved_edge!=-1) && (index_graph_land_station_from_next_pylon!=-1) && (battery
                        - battery_drop_matrices_[ UAVs_[i].id ][ current_flight_plan.nodes.back() ][ index_pylon_connected_with_unserved_edge ]
                        - battery_drop_matrices_[ UAVs_[i].id ][ index_pylon_connected_with_unserved_edge ][ index_graph_land_station_from_next_pylon ] >= UAVs_[i].minimum_battery) &&
                        (regular_or_fast_inspection_ || (i==UAVs_.size()-1) || (battery - battery_drop_matrices_[ UAVs_[i].id ][ current_flight_plan.nodes.back() ][ index_pylon_connected_with_unserved_edge ] - battery_drop_matrices_[ UAVs_[i].id ][ index_pylon_connected_with_unserved_edge ][ index_graph_land_station_from_next_pylon ] >= proportional_battery_)) ) {
                        battery -= battery_drop_matrices_[ UAVs_[i].id ][ current_flight_plan.nodes.back() ][ index_graph_of_next_pylon ];
                        current_flight_plan.nodes.push_back(index_graph_of_next_pylon);
                        index_graph_of_next_pylon = index_pylon_connected_with_unserved_edge;
                    } else {    // Only insert the next pylon if it has battery to at least fulfill one edge.
                        break;
                    }
                }
            } else {
                // Calculate closest land station from that most rewarded pylon:
                nearestGraphNodeLandStation(index_graph_of_next_pylon, index_graph_land_station_from_next_pylon, UAVs_[i].id);
            }
        }
        // Insert closest land station when inserting another pylon would result in low battery:
        current_flight_plan.nodes.push_back(index_graph_land_station_from_pylon_last);

        if (current_flight_plan.nodes.size()>3) {   // Consider the flight plan only if it visits two or more pylons.
            flight_plans_.push_back(current_flight_plan);
        }
    }

    fillFlightPlansFields(flight_plans_);

    return flight_plans_;

} // end getPlanGreedy method


// Brief method that returns the index of the nearest land station graph node given an initial index.
float CentralizedPlanner::nearestGraphNodeLandStation(int _from_this_index_graph, int& _index_graph_node_to_return, int _uav_id) {

    if (_from_this_index_graph==-1) {
        return -1;
    }

    _index_graph_node_to_return = -1;
    float time_cost = std::numeric_limits<float>::max();

    for (int i=0; i<graph_.size(); i++) {
        if (_from_this_index_graph == i) continue;
        if (graph_[i].type==aerialcore_msgs::GraphNode::TYPE_RECHARGE_LAND_STATION || graph_[i].type==aerialcore_msgs::GraphNode::TYPE_REGULAR_LAND_STATION) {

            float current_time_cost = time_cost_matrices_[_uav_id][ _from_this_index_graph ][ i ];
            if (current_time_cost <= 0.001) continue;     // It's the same graph node or path not found, ignore and continue.

            if ( time_cost > current_time_cost ) {
                _index_graph_node_to_return = i;
                time_cost = current_time_cost;
            }
        }
    }

    return time_cost;

} // end nearestGraphNodeLandStation method


// Brief method that returns the index of the nearest initial position graph node given an initial index.
float CentralizedPlanner::nearestGraphNodeUAVInitialPosition(int _from_this_index_graph, int& _index_graph_node_to_return, int& _uav_id) {

    if (_from_this_index_graph==-1) {
        return -1;
    }

    _index_graph_node_to_return = -1;
    _uav_id = -1;
    float time_cost = std::numeric_limits<float>::max();

    for (int i=0; i<graph_.size(); i++) {
        if (_from_this_index_graph == i) continue;
        if ( graph_[i].type==aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION) {

            float current_time_cost = time_cost_matrices_[ graph_[i].id ][ _from_this_index_graph ][ i ];
            if (current_time_cost <= 0.001) continue;     // It's the same graph node or path not found, ignore and continue.

            if ( time_cost > current_time_cost ) {
                _index_graph_node_to_return = i;
                _uav_id = graph_[i].id;
                time_cost = current_time_cost;
            }
        }
    }

    return time_cost;

} // end nearestGraphNodeUAVInitialPosition method


// Brief method that returns the index of the nearest pylon graph node given an initial index. The new pylon will have edges unserved and will not be directly connected.
float CentralizedPlanner::nearestGraphNodePylon(int _from_this_index_graph, int& _index_graph_node_to_return, int _uav_id) {

    if (_from_this_index_graph==-1) {
        return -1;
    }

    _index_graph_node_to_return = -1;
    float time_cost = std::numeric_limits<float>::max();

    for (int i=0; i<graph_.size(); i++) {
        if (_from_this_index_graph == i) continue;
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

            float current_time_cost = time_cost_matrices_[_uav_id][ _from_this_index_graph ][ i ];
            if (current_time_cost <= 0.001) continue;     // It's the same graph node or path not found, ignore and continue.

            // Only consider i (current node iterated) if it has edges unserved:
            bool has_edges_unserved = false;
            for (const int& current_connection_index : graph_[i].connections_indexes) {
                std::pair <int,int> current_edge;
                current_edge.first =  i < current_connection_index ? i : current_connection_index;
                current_edge.second = i < current_connection_index ? current_connection_index : i;
                for (int j=0; j<connection_edges_.size(); j++) {
                    if (connection_edges_[j] == current_edge) {
                        has_edges_unserved = true;
                        break;
                    }
                }
            }
            if (!has_edges_unserved) {
                continue;
            }

            if ( time_cost > current_time_cost ) {
                _index_graph_node_to_return = i;
                time_cost = current_time_cost;
            }
        }
    }

    return time_cost;

} // end nearestGraphNodePylon method


// Brief method that returns the index of the furthest connected pylon from an initial pylon.
float CentralizedPlanner::mostRewardedPylon(int _initial_pylon_index, int& _index_graph_node_to_return, int& _index_edge_to_erase, int _uav_id) {

    _index_graph_node_to_return = -1;
    float time_cost = 0;
    _index_edge_to_erase = -1;

    if (_initial_pylon_index==-1 || !graph_[_initial_pylon_index].type==aerialcore_msgs::GraphNode::TYPE_PYLON) {
        return -1;
    }

    for (const int& current_connection_index : graph_[_initial_pylon_index].connections_indexes) {

        // Calculate the current edge between both pylons:
        std::pair <int,int> current_edge;
        current_edge.first =  _initial_pylon_index < current_connection_index ? _initial_pylon_index : current_connection_index;
        current_edge.second = _initial_pylon_index < current_connection_index ? current_connection_index : _initial_pylon_index;

        // If current edge not found means it has already been served, so continue.
        bool edge_found = false;
        int i = 0;
        for (i=0; i<connection_edges_.size(); i++) {
            if (connection_edges_[i] == current_edge) {
                edge_found = true;
                break;
            }
        }
        if (!edge_found) {
            continue;
        }

        float current_time_cost = time_cost_matrices_[_uav_id][ _initial_pylon_index ][ current_connection_index ];
        if (current_time_cost <= 0.001) continue;     // It's the same graph node or path not found, ignore and continue.

        if ( time_cost < current_time_cost ) {
            _index_graph_node_to_return = current_connection_index;
            time_cost = current_time_cost;
            _index_edge_to_erase = i;
        }
    }

    return time_cost;

} // end mostRewardedPylon method


// Brief method to print the plan in the terminal.
void CentralizedPlanner::printPlan() {
    std::cout << std::endl;
    std::cout << "Printing flight plan from the planner:" << std::endl;
    std::cout << std::endl;
    for (int i=0; i<flight_plans_.size(); i++) {
        std::cout << "flight_plans[ " << i << " ].uav_id = " << flight_plans_[i].uav_id << std::endl;
        std::cout << "flight_plans[ " << i << " ].header.stamp.toSec() = " << flight_plans_[i].header.stamp.toSec() << std::endl;
        for (int j=0; j<flight_plans_[i].nodes.size(); j++) {
            std::cout << "flight_plans[ " << i << " ].nodes[ " << j << " ] = " << flight_plans_[i].nodes[j] << std::endl;
        }
        for (int j=0; j<flight_plans_[i].poses.size(); j++) {
            std::cout << "flight_plans[ " << i << " ].poses[ " << j << " ] = " << flight_plans_[i].poses[j];
        }
        for (int j=0; j<flight_plans_[i].type.size(); j++) {
            if (flight_plans_[i].type[j] == aerialcore_msgs::FlightPlan::TYPE_TAKEOFF_WP) {
                std::cout << "flight_plans[ " << i << " ].type[ " << j << " ] = TYPE_TAKEOFF_WP" << std::endl;
            } else if (flight_plans_[i].type[j] == aerialcore_msgs::FlightPlan::TYPE_PASS_PYLON_WP) {
                std::cout << "flight_plans[ " << i << " ].type[ " << j << " ] = TYPE_PASS_PYLON_WP" << std::endl;
            } else if (flight_plans_[i].type[j] == aerialcore_msgs::FlightPlan::TYPE_PASS_NFZ_WP) {
                std::cout << "flight_plans[ " << i << " ].type[ " << j << " ] = TYPE_PASS_NFZ_WP" << std::endl;
            } else if (flight_plans_[i].type[j] == aerialcore_msgs::FlightPlan::TYPE_LAND_WP) {
                std::cout << "flight_plans[ " << i << " ].type[ " << j << " ] = TYPE_LAND_WP" << std::endl;
            }
        }
    }
    std::cout << std::endl;

    std::cout << "Planner's results:" << std::endl;
    float total_time_cost = 0;
    float total_battery_cost = 0;
    for (int i=0; i<flight_plans_.size(); i++) {
        std::cout << "flight_plans[ " << i << " ].uav_id       = " << flight_plans_[i].uav_id << std::endl;
        float time_cost = 0;
        float battery_cost = 0;
        for (int j=0; j<flight_plans_[i].nodes.size()-1; j++) {
            time_cost += time_cost_matrices_[flight_plans_[i].uav_id][ flight_plans_[i].nodes[j] ][ flight_plans_[i].nodes[j+1] ];
            battery_cost += battery_drop_matrices_[flight_plans_[i].uav_id][ flight_plans_[i].nodes[j] ][ flight_plans_[i].nodes[j+1] ];
        }
        std::cout << "flight_plans[ " << i << " ].time_cost    = " << time_cost << std::endl;
        std::cout << "flight_plans[ " << i << " ].battery_cost = " << battery_cost << std::endl;
        total_time_cost += time_cost;
        total_battery_cost += battery_cost;
    }
    std::cout << "Total plan time cost:    " << total_time_cost << std::endl;
    std::cout << "Total plan battery cost: " << total_battery_cost << std::endl;
    std::cout << "UAVs involved:           " << flight_plans_.size() << std::endl << std::endl;
} // end printPlan method


std::vector<aerialcore_msgs::FlightPlan> CentralizedPlanner::getPlanMILP(std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector< std::tuple<float, float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::vector< geometry_msgs::Polygon >& _no_fly_zones, const geometry_msgs::Polygon& _geofence, const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices, const std::map<int, std::map<int, std::map<int, float> > >& _battery_drop_matrices, std::map<int, int> _last_flight_plan_graph_node) {

//     time_cost_matrices_.clear();
//     battery_drop_matrices_.clear();

//     constructUnordenedMaps(_time_cost_matrices, _battery_drop_matrices);

//     constructUAVs(_drone_info);
//     constructEdges(_graph);

//     flight_plans_.clear();

//     //////////////////// Start defining the MILP problem according to agarwal_icra20 and solve it using OR-Tools ////////////////////

//     // Create the mip solver with the SCIP backend:
//     std::unique_ptr<operations_research::MPSolver> solver(operations_research::MPSolver::CreateSolver("SCIP"));

//     const double infinity = solver->infinity();

//     // Variables:
//     // x[x_index_size_]       is a vector of binary variables.
//     // y[y_and_f_index_size_] is a vector of non-negative integer variables.
//     // f[y_and_f_index_size_] is a vector of non-negative continuous variables.
//     std::vector<operations_research::MPVariable *> x;
//     std::vector<operations_research::MPVariable *> y;
//     std::vector<operations_research::MPVariable *> f;
//     for (int index=0; index<x_index_size_; index++) {
//         x.push_back( solver->MakeBoolVar("") );
//     }
//     for (int index=0; index<y_and_f_index_size_; index++) {
//         y.push_back( solver->MakeIntVar(0, infinity, "") );
//         f.push_back( solver->MakeNumVar(0.0, 1.0, "") );
//     }
//     LOG(INFO) << "Number of variables = " << solver->NumVariables();

//     // Create the objective function:
//     // (1) Minimize all the inspection time (UAVs flying separately).
//     operations_research::MPObjective *const objective = solver->MutableObjective();
//     for (std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
//         for (int e_i=0; e_i<edges_.size(); e_i++) {
//             if (edges_[e_i].type == aerialcore_msgs::EdgeType::TYPE_INSPECTION) {
//                 objective->SetCoefficient(x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], it->second[ edges_[e_i].i ][ edges_[e_i].j ]);
//             }
//             if ( (edges_[e_i].type != aerialcore_msgs::EdgeType::TYPE_TAKEOFF_AND_NAVIGATION && edges_[e_i].type != aerialcore_msgs::EdgeType::TYPE_NAVIGATION_AND_LANDING) || it->first==edges_[e_i].k) {
//                 objective->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], it->second[ edges_[e_i].i ][ edges_[e_i].j ]);
//             }
//         }
//     }
//     objective->SetMinimization();


//     // Create the constraints:

//     // (2) For all pylon nodes and tours-drones, Sum_all_j(x_ij_k) plus Sum_all_j(y_ij_k) minus Sum_all_j(x_ji_k) and minus Sum_all_j(x_ji_k) is equal to zero (inputs equal to outputs in each node):
//     for (std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
//         for (int g_i=0; g_i<_graph.size(); g_i++) {
//             if (_graph[g_i].type == aerialcore_msgs::GraphNode::TYPE_PYLON) {
//                 operations_research::MPConstraint *constraint = solver->MakeRowConstraint(0, 0, "");
//                 for (int e_i=0; e_i<edges_.size(); e_i++) {
//                     if (edges_[e_i].i == g_i) {
//                         if (edges_[e_i].type == aerialcore_msgs::EdgeType::TYPE_INSPECTION) {
//                             constraint->SetCoefficient(x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], 1);
//                             constraint->SetCoefficient(x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].j ][ edges_[e_i].i ] ], -1);
//                         }
//                         if (edges_[e_i].type != aerialcore_msgs::EdgeType::TYPE_TAKEOFF_AND_NAVIGATION && edges_[e_i].type != aerialcore_msgs::EdgeType::TYPE_NAVIGATION_AND_LANDING) {
//                             constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], 1);
//                             constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].j ][ edges_[e_i].i ] ], -1);
//                         }
//                     }
//                 }
//             } else if (_graph[g_i].type == aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION) {
//                 operations_research::MPConstraint *constraint = solver->MakeRowConstraint(0, 0, "");
//                 for (int e_i=0; e_i<edges_.size(); e_i++) {
//                     if (edges_[e_i].i == g_i) {
//                         if (it->first==edges_[e_i].k) {
//                             constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], 1);
//                             constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].j ][ edges_[e_i].i ] ], -1);
//                         }
//                     }
//                 }
//             }
//         }
//     }

//     // (3) For all inspection edges, x_ij plus x_ji sums 1 (all inspection edges covered in any direction):
//     for (int e_i=0; e_i<edges_.size(); e_i++) {
//         if (edges_[e_i].type == aerialcore_msgs::EdgeType::TYPE_INSPECTION && edges_[e_i].i<edges_[e_i].j) {
//             operations_research::MPConstraint *constraint = solver->MakeRowConstraint(1, 1, "");
//             for (std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
//                 constraint->SetCoefficient(x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], 1);
//                 constraint->SetCoefficient(x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].j ][ edges_[e_i].i ] ], 1);
//             }
//         }
//     }

//     // (4) For all tours-drones, in edges coming from takeoff nodes, y_0j is 0 or 1 (take off just once or zero times in each tour):
//     for (std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
//         operations_research::MPConstraint *constraint = solver->MakeRowConstraint(0, 1, "");
//         for (int e_i=0; e_i<edges_.size(); e_i++) {
//             if (edges_[e_i].type == aerialcore_msgs::EdgeType::TYPE_TAKEOFF_AND_NAVIGATION && it->first==edges_[e_i].k) {
//                 constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], 1);
//             }
//         }
//     }

//     // // (4.1) For all tours-drones, in edges going to land nodes, y_j0 is 0 or 1 (land just once in each tour):
//     // for (std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
//     //     operations_research::MPConstraint *constraint = solver->MakeRowConstraint(1, 1, "");
//     //     for (int e_i=0; e_i<edges_.size(); e_i++) {
//     //         if (edges_[e_i].type == aerialcore_msgs::EdgeType::TYPE_NAVIGATION_AND_LANDING && it->first==edges_[e_i].k) {
//     //             constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], 1);
//     //         }
//     //     }
//     // }

//     // (5) For all tours-drones and pylon nodes, equation of flow consumed in node (flow in equals to flow destroyed in that node plus flow out):
//     for (std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > >::iterator it = battery_drop_matrices_.begin(); it != battery_drop_matrices_.end(); it++) {
//         for (int g_i=0; g_i<_graph.size(); g_i++) {
//             if (_graph[g_i].type == aerialcore_msgs::GraphNode::TYPE_PYLON) {
//                 operations_research::MPConstraint *constraint = solver->MakeRowConstraint(0.0, 0.0, "");
//                 for (int e_i=0; e_i<edges_.size(); e_i++) {
//                     if (edges_[e_i].i == g_i) {
//                         constraint->SetCoefficient(f[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].j ][ edges_[e_i].i ] ], 1);
//                         constraint->SetCoefficient(f[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], -1);
//                         if (edges_[e_i].type == aerialcore_msgs::EdgeType::TYPE_INSPECTION) {
//                             constraint->SetCoefficient(x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].j ][ edges_[e_i].i ] ], -it->second[ edges_[e_i].j ][ edges_[e_i].i ]);
//                         }
//                         if ((edges_[e_i].type != aerialcore_msgs::EdgeType::TYPE_TAKEOFF_AND_NAVIGATION && edges_[e_i].type != aerialcore_msgs::EdgeType::TYPE_NAVIGATION_AND_LANDING) || it->first==edges_[e_i].k) {
//                             constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].j ][ edges_[e_i].i ] ], -it->second[ edges_[e_i].j ][ edges_[e_i].i ]);
//                         }
//                     }
//                 }
//             }
//         }
//     }
//     // for (std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > >::iterator it = battery_drop_matrices_.begin(); it != battery_drop_matrices_.end(); it++) {
//     //     for (int e_i=0; e_i<edges_.size(); e_i++) {
//     //         if (edges_[e_i].type != aerialcore_msgs::EdgeType::TYPE_TAKEOFF_AND_NAVIGATION) {
//     //             operations_research::MPConstraint *constraint = solver->MakeRowConstraint(0.0, 0.0, "");
//     //             constraint->SetCoefficient(f[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], 1);
//     //             for (int e_j=0; e_j<edges_.size(); e_j++) {
//     //                 if (e_j == e_i) continue;
//     //                 if (edges_[e_i].j == edges_[e_j].i) {
//     //                     constraint->SetCoefficient(f[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_j].i ][ edges_[e_j].j ] ], -1);
//     //                     if (edges_[e_j].type == aerialcore_msgs::EdgeType::TYPE_INSPECTION) {
//     //                         constraint->SetCoefficient(x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_j].i ][ edges_[e_j].j ] ], it->second[ edges_[e_j].i ][ edges_[e_j].j ]);
//     //                     }
//     //                     if ((edges_[e_j].type != aerialcore_msgs::EdgeType::TYPE_TAKEOFF_AND_NAVIGATION && edges_[e_j].type != aerialcore_msgs::EdgeType::TYPE_NAVIGATION_AND_LANDING) || it->first==edges_[e_j].k) {
//     //                         constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_j].i ][ edges_[e_j].j ] ], it->second[ edges_[e_j].i ][ edges_[e_j].j ]);
//     //                     }
//     //                 }
//     //             }
//     //         }
//     //     }
//     // }

//     // (6) For all tours-drones, all flow is created in the takeoff nodes:
//     for (std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > >::iterator it = battery_drop_matrices_.begin(); it != battery_drop_matrices_.end(); it++) {
//         operations_research::MPConstraint *constraint = solver->MakeRowConstraint(0.0, 0.0, "");
//         for (int e_i=0; e_i<edges_.size(); e_i++) {
//             if (edges_[e_i].type == aerialcore_msgs::EdgeType::TYPE_TAKEOFF_AND_NAVIGATION && it->first==edges_[e_i].k) {
//                 constraint->SetCoefficient(f[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], 1);
//             }
//             if (edges_[e_i].type == aerialcore_msgs::EdgeType::TYPE_INSPECTION) {
//                 constraint->SetCoefficient(x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], -it->second[ edges_[e_i].i ][ edges_[e_i].j ]);
//             }
//             if ((edges_[e_i].type != aerialcore_msgs::EdgeType::TYPE_TAKEOFF_AND_NAVIGATION && edges_[e_i].type != aerialcore_msgs::EdgeType::TYPE_NAVIGATION_AND_LANDING) || it->first==edges_[e_i].k) {
//                 constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], -it->second[ edges_[e_i].i ][ edges_[e_i].j ]);
//             }
//         }
//     }

//     // (7) For all tours-drones, land nodes receive all the remaining flow:
//     for (std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > >::iterator it = battery_drop_matrices_.begin(); it != battery_drop_matrices_.end(); it++) {
//         operations_research::MPConstraint *constraint = solver->MakeRowConstraint(0.0, 0.0, "");
//         for (int e_i=0; e_i<edges_.size(); e_i++) {
//             if (edges_[e_i].type == aerialcore_msgs::EdgeType::TYPE_NAVIGATION_AND_LANDING && it->first==edges_[e_i].k) {
//                 constraint->SetCoefficient(f[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], 1);
//                 constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], -it->second[ edges_[e_i].i ][ edges_[e_i].j ]);
//             }
//         }
//     }

//     // (8) For all tours-drones and edges, there is only flow if edge served:
//     for (std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > >::iterator it = battery_drop_matrices_.begin(); it != battery_drop_matrices_.end(); it++) {
//         for (int e_i=0; e_i<edges_.size(); e_i++) {
//             operations_research::MPConstraint *constraint = solver->MakeRowConstraint(-infinity, 0.0, "");
//             constraint->SetCoefficient(f[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], 1);
//             if (edges_[e_i].type == aerialcore_msgs::EdgeType::TYPE_INSPECTION) {
//                 constraint->SetCoefficient(x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], -1.0);
//             }
//             if ((edges_[e_i].type != aerialcore_msgs::EdgeType::TYPE_TAKEOFF_AND_NAVIGATION && edges_[e_i].type != aerialcore_msgs::EdgeType::TYPE_NAVIGATION_AND_LANDING) || it->first==edges_[e_i].k) {
//                 constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], -1.0);
//             }
//         }
//     }


//     // (1.14) For all tours-drones and edges, flow has a minimum value:
//     for (std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > >::iterator it = battery_drop_matrices_.begin(); it != battery_drop_matrices_.end(); it++) {
//         for (int e_i=0; e_i<edges_.size(); e_i++) {
//             operations_research::MPConstraint *constraint = solver->MakeRowConstraint(0.0, infinity, "");
//             constraint->SetCoefficient(f[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], 1);
//             if (edges_[e_i].type == aerialcore_msgs::EdgeType::TYPE_INSPECTION) {
//                 constraint->SetCoefficient(x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], -it->second[ edges_[e_i].i ][ edges_[e_i].j ]);
//             }
//             if ((edges_[e_i].type != aerialcore_msgs::EdgeType::TYPE_TAKEOFF_AND_NAVIGATION && edges_[e_i].type != aerialcore_msgs::EdgeType::TYPE_NAVIGATION_AND_LANDING) || it->first==edges_[e_i].k) {
//                 constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], -it->second[ edges_[e_i].i ][ edges_[e_i].j ]);
//             }
//         }
//     }

//     // (9) and (10) are assumed to be met in the dataset. Also, the robot capacity is assumed to be sufficiently large to service any edge.

//     LOG(INFO) << "Number of constraints = " << solver->NumConstraints();


//     // SOLVE THE PROBLEM:
//     const operations_research::MPSolver::ResultStatus result_status = solver->Solve();

//     // Check that the problem has an optimal solution.
//     if (result_status != operations_research::MPSolver::OPTIMAL) {
//         LOG(FATAL) << "The problem does not have an optimal solution.";
//     }
//     LOG(INFO) << "Solution:";
//     LOG(INFO) << "Optimal objective value = " << objective->Value();

// # ifdef DEBUG
//     // Print the variables of the solution:
//     for (std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
//         std::cout << "x[" << it->first << "]" << std::endl;
//         for (int i=0; i<it->second.size(); i++) {
//             for (int j=0; j<it->second[i].size(); j++) {
//                 if (i==0 || j==0) {
//                     std::cout << it->second[i][j] << " ";
//                     continue;
//                 }
//                 bool inspection_edge_found = false;
//                 for (aerialcore_msgs::Edge& edge_struct : edges_) {
//                     if (edge_struct.type == aerialcore_msgs::EdgeType::TYPE_INSPECTION && edge_struct.i == i && edge_struct.j == j) {
//                         std::cout << std::setprecision(6) << x[ from_k_i_j_to_x_index_[ it->first ][i][j] ]->solution_value() << " ";
//                         inspection_edge_found = true;
//                         break;
//                     }
//                 }
//                 if (!inspection_edge_found) {
//                     std::cout << "- ";
//                 }
//             }
//             std::cout << std::endl;
//         }
//     }
//     std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > >::iterator map_begin = time_cost_matrices_.begin();
//     for (std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
//         std::cout << "y[" << it->first << "]" << std::endl;
//         for (int i=0; i<it->second.size(); i++) {
//             for (int j=0; j<it->second[i].size(); j++) {
//                 if (i==0 || j==0) {
//                     std::cout << it->second[i][j] << " ";
//                     continue;
//                 }
//                 if (time_cost_matrices_[map_begin->first][i][j]==-1) {
//                     std::cout << "- ";
//                 } else {
//                     std::cout << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[ it->first ][i][j] ]->solution_value() << " ";
//                 }
//             }
//             std::cout << std::endl;
//         }
//     }
//     for (std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
//         std::cout << "f[" << it->first << "]" << std::endl;
//         for (int i=0; i<it->second.size(); i++) {
//             for (int j=0; j<it->second[i].size(); j++) {
//                 if (i==0 || j==0) {
//                     std::cout << it->second[i][j] << " ";
//                     continue;
//                 }
//                 if (time_cost_matrices_[map_begin->first][i][j]==-1) {
//                     std::cout << "- ";
//                 } else {
//                     std::cout << std::setprecision(6) << f[ from_k_i_j_to_y_and_f_index_[ it->first ][i][j] ]->solution_value() << " ";
//                 }
//             }
//             std::cout << std::endl;
//         }
//     }

//     //////////////////// End defining the MILP problem according to agarwal_icra20 and solve it using OR-Tools ////////////////////

//     // Print the objective and restrictions for debug:
//     std::cout<< std::endl << "Print the objective and restrictions for debug:" << std::endl << std::endl;

//     // (1) Minimize all the inspection time (UAVs flying separately).
//     std::cout << "(1):" << std::endl;
//     for (std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
//         for (int e_i=0; e_i<edges_.size(); e_i++) {
//             if (edges_[e_i].type == aerialcore_msgs::EdgeType::TYPE_INSPECTION) {
//                 std::cout << "x[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " · " << it->second[ edges_[e_i].i ][ edges_[e_i].j ] << " + ";
//                 // std::cout << std::endl;
//                 // std::cout << "e_i = " << e_i << std::endl;
//                 // std::cout << "k = " << it->first << std::endl;
//                 // std::cout << "edges_[e_i].i = " << edges_[e_i].i << std::endl;
//                 // std::cout << "edges_[e_i].j = " << edges_[e_i].j << std::endl;
//                 // std::cout << "from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] = " << from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] << std::endl;
//                 // std::cout << "<< std::setprecision(6) << from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ] = " << "x " << x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << std::endl;
//             }
//             if ( (edges_[e_i].type != aerialcore_msgs::EdgeType::TYPE_TAKEOFF_AND_NAVIGATION && edges_[e_i].type != aerialcore_msgs::EdgeType::TYPE_NAVIGATION_AND_LANDING) || it->first==edges_[e_i].k) {
//                 std::cout << "y[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " · " << it->second[ edges_[e_i].i ][ edges_[e_i].j ] << " + ";
//                 // std::cout << std::endl;
//                 // std::cout << "e_i = " << e_i << std::endl;
//                 // std::cout << "k = " << it->first << std::endl;
//                 // std::cout << "edges_[e_i].i = " << edges_[e_i].i << std::endl;
//                 // std::cout << "edges_[e_i].j = " << edges_[e_i].j << std::endl;
//                 // std::cout << "from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] = " << from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] << std::endl;
//                 // std::cout << "<< std::setprecision(6) << from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ] = " << "y " << y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << std::endl;
//             }
//         }
//     }
//     std::cout << std::endl << std::endl;

//     // (2) For all pylon nodes and tours-drones, Sum_all_j(x_ij_k) plus Sum_all_j(y_ij_k) minus Sum_all_j(x_ji_k) and minus Sum_all_j(x_ji_k) is equal to zero (inputs equal to outputs in each node):
//     std::cout << "(2):" << std::endl;
//     for (std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
//         for (int g_i=0; g_i<_graph.size(); g_i++) {
//             if (_graph[g_i].type == aerialcore_msgs::GraphNode::TYPE_PYLON) {
//                 std::cout << "node_" << g_i << " : ";
//                 for (int e_i=0; e_i<edges_.size(); e_i++) {
//                     if (edges_[e_i].i == g_i) {
//                         if (edges_[e_i].type == aerialcore_msgs::EdgeType::TYPE_INSPECTION) {
//                             std::cout << "x[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " · " << 1 << " + ";
//                             std::cout << "x[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "] " << std::setprecision(6) << x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].j ][ edges_[e_i].i ] ]->solution_value() << " · " << -1 << " + ";
//                         }
//                         if (edges_[e_i].type != aerialcore_msgs::EdgeType::TYPE_TAKEOFF_AND_NAVIGATION && edges_[e_i].type != aerialcore_msgs::EdgeType::TYPE_NAVIGATION_AND_LANDING) {
//                             std::cout << "y[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " · " << 1 << " + ";
//                             std::cout << "y[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].j ][ edges_[e_i].i ] ]->solution_value() << " · " << -1 << " + ";
//                         }
//                     }
//                 }
//                 std::cout << std::endl;
//             } else if (_graph[g_i].type == aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION) {
//                 std::cout << "node_" << g_i << " : ";
//                 for (int e_i=0; e_i<edges_.size(); e_i++) {
//                     if (edges_[e_i].i == g_i) {
//                         if (it->first==edges_[e_i].k) {
//                             std::cout << "y[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " · " << 1 << " + ";
//                         }
//                     } else if (edges_[e_i].j == g_i) {
//                         if (it->first==edges_[e_i].k) {
//                             std::cout << "y[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].j ][ edges_[e_i].i ] ]->solution_value() << " · " << -1 << " + ";
//                         }
//                     }
//                 }
//                 std::cout << std::endl;
//             }
//         }
//     }
//     std::cout << std::endl;

//     // (3) For all inspection edges, x_ij plus x_ji sums 1 (all inspection edges covered in any direction):
//     std::cout << "(3):" << std::endl;
//     for (int e_i=0; e_i<edges_.size(); e_i++) {
//         if (edges_[e_i].type == aerialcore_msgs::EdgeType::TYPE_INSPECTION && edges_[e_i].i<edges_[e_i].j) {
//         std::cout << "inspection_edge_" << e_i << " : ";
//             for (std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
//                 std::cout << "x[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " · " << 1 << " + ";
//                 std::cout << "x[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "] " << std::setprecision(6) << x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].j ][ edges_[e_i].i ] ]->solution_value() << " · " << 1 << " + ";
//             }
//             std::cout << std::endl;
//         }
//     }
//     std::cout << std::endl;

//     // (4) For all tours-drones, in edges coming from takeoff nodes, y_0j is 0 or 1 (take off just once in each tour):
//     std::cout << "(4):" << std::endl;
//     for (std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
//         std::cout << "k_" << it->first << " : ";
//         for (int e_i=0; e_i<edges_.size(); e_i++) {
//             if (edges_[e_i].type == aerialcore_msgs::EdgeType::TYPE_TAKEOFF_AND_NAVIGATION && it->first==edges_[e_i].k) {
//                 std::cout << "y[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " · " << 1 << " + ";
//             }
//         }
//         std::cout << std::endl;
//     }
//     std::cout << std::endl;

//     // // (4.1) For all tours-drones, in edges going to land nodes, y_j0 is 0 or 1 (land just once in each tour):
//     // std::cout << "(4.1):" << std::endl;
//     // for (std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
//     //     std::cout << "k_" << it->first << " : ";
//     //     for (int e_i=0; e_i<edges_.size(); e_i++) {
//     //         if (edges_[e_i].type == aerialcore_msgs::EdgeType::TYPE_NAVIGATION_AND_LANDING && it->first==edges_[e_i].k) {
//     //             std::cout << "y[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " · " << 1 << " + ";
//     //         }
//     //     }
//     //     std::cout << std::endl;
//     // }
//     // std::cout << std::endl;

//     // (5) For all tours-drones and pylon nodes, equation of flow consumed in node (flow in equals to flow destroyed in that node plus flow out):
//     std::cout << "(5):" << std::endl;
//     for (std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > >::iterator it = battery_drop_matrices_.begin(); it != battery_drop_matrices_.end(); it++) {
//         for (int g_i=0; g_i<_graph.size(); g_i++) {
//             if (_graph[g_i].type == aerialcore_msgs::GraphNode::TYPE_PYLON) {
//                 std::cout << "pylon_node_" << g_i << " : ";
//                 for (int e_i=0; e_i<edges_.size(); e_i++) {
//                     if (edges_[e_i].i == g_i) {
//                         std::cout << "f[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "] " << std::setprecision(6) << f[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].j ][ edges_[e_i].i ] ]->solution_value() << " · " << 1 << " + ";
//                         std::cout << "f[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << f[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " · " << -1 << " + ";
//                         if (edges_[e_i].type == aerialcore_msgs::EdgeType::TYPE_INSPECTION) {
//                             std::cout << "x[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "] " << std::setprecision(6) << x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].j ][ edges_[e_i].i ] ]->solution_value() << " · " << -it->second[ edges_[e_i].j ][ edges_[e_i].i ] << " + ";
//                         }
//                         if ((edges_[e_i].type != aerialcore_msgs::EdgeType::TYPE_TAKEOFF_AND_NAVIGATION && edges_[e_i].type != aerialcore_msgs::EdgeType::TYPE_NAVIGATION_AND_LANDING) || it->first==edges_[e_i].k) {
//                             std::cout << "y[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].j ][ edges_[e_i].i ] ]->solution_value() << " · " << -it->second[ edges_[e_i].j ][ edges_[e_i].i ] << " + ";
//                         }
//                     }
//                 }
//                 std::cout << std::endl;
//             }
//         }
//     }
//     std::cout << std::endl;

//     // (6) For all tours-drones, all flow is created in the takeoff nodes:
//     std::cout << "(6):" << std::endl;
//     for (std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > >::iterator it = battery_drop_matrices_.begin(); it != battery_drop_matrices_.end(); it++) {
//         std::cout << "k_" << it->first << " : ";
//         for (int e_i=0; e_i<edges_.size(); e_i++) {
//             if (edges_[e_i].type == aerialcore_msgs::EdgeType::TYPE_TAKEOFF_AND_NAVIGATION && it->first==edges_[e_i].k) {
//                 std::cout << "f[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << f[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " · " << 1 << " + ";
//             }
//             if (edges_[e_i].type == aerialcore_msgs::EdgeType::TYPE_INSPECTION) {
//                 std::cout << "x[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " · " << -it->second[ edges_[e_i].i ][ edges_[e_i].j ] << " + ";
//             }
//             if ((edges_[e_i].type != aerialcore_msgs::EdgeType::TYPE_TAKEOFF_AND_NAVIGATION && edges_[e_i].type != aerialcore_msgs::EdgeType::TYPE_NAVIGATION_AND_LANDING) || it->first==edges_[e_i].k) {
//                 std::cout << "y[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " · " << -it->second[ edges_[e_i].i ][ edges_[e_i].j ] << " + ";
//             }
//         }
//         std::cout << std::endl;
//     }
//     std::cout << std::endl;

//     // (7) For all tours-drones, land nodes receive all the remaining flow:
//     std::cout << "(7):" << std::endl;
//     for (std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > >::iterator it = battery_drop_matrices_.begin(); it != battery_drop_matrices_.end(); it++) {
//         std::cout << "k_" << it->first << " : ";
//         for (int e_i=0; e_i<edges_.size(); e_i++) {
//             if (edges_[e_i].type == aerialcore_msgs::EdgeType::TYPE_NAVIGATION_AND_LANDING) {
//                 std::cout << "f[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << f[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " · " << 1 << " + ";
//                 std::cout << "y[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " · " << -it->second[ edges_[e_i].i ][ edges_[e_i].j ] << " + ";
//             }
//         }
//         std::cout << std::endl;
//     }
//     std::cout << std::endl;

//     // (8) For all tours-drones and edges, there is only flow if edge served:
//     std::cout << "(8):" << std::endl;
//     for (std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > >::iterator it = battery_drop_matrices_.begin(); it != battery_drop_matrices_.end(); it++) {
//         for (int e_i=0; e_i<edges_.size(); e_i++) {
//             std::cout << "edge_" << e_i << " : ";
//             std::cout << "f[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << f[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " + ";
//             if (edges_[e_i].type == aerialcore_msgs::EdgeType::TYPE_INSPECTION) {
//                 std::cout << "x[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " + ";
//             }
//             if ((edges_[e_i].type != aerialcore_msgs::EdgeType::TYPE_TAKEOFF_AND_NAVIGATION && edges_[e_i].type != aerialcore_msgs::EdgeType::TYPE_NAVIGATION_AND_LANDING) || it->first==edges_[e_i].k) {
//                 std::cout << "y[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " + ";
//             }
//             std::cout << std::endl;
//         }
//     }
//     std::cout << std::endl;


//     /* Once printed the results, you can tidy up those terminal results copying them to VS Code and replace the following to just zero 0 (search using Regular Expressions):

//      -0 

//     -*\d*\.\d*e-\d* 


//     And deleting these:

//     [fxy]\[\d+?\]\[\d+?\]\[\d+?\] -*0 · -*\d\.*\d* \+ 

//     [xyf]\[\d+?\]\[\d+?\]\[\d+?\] 0 · \d*\.\d* \+ 

//     [fxy]\[\d+?\]\[\d+?\]\[\d+?\] 0 \+ 

//     */

// # endif

//     // Assign flight_plans_ according to the edges in the solution:
// //    for (std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > >::iterator it = battery_drop_matrices_.begin(); it != battery_drop_matrices_.end(); it++) {

// //         // All this commented because the solution isn't totally sorted in f descendent:
// //         struct fStruct {
// //             float f;
// //             int x;
// //             int y;
// //             int k;
// //             int i;
// //             int j;
// //         };
// //         std::vector<fStruct> f_struct_vector;

// //         for (int i=1; i<it->second.size(); i++) {
// //             for (int j=1; j<it->second[i].size(); j++) {
// //                 if (f[ from_k_i_j_to_y_and_f_index_[ it->first ][i][j] ]->solution_value()>0.001) {
// //                     fStruct new_f_struct;
// //                     new_f_struct.f = f[ from_k_i_j_to_y_and_f_index_[ it->first ][i][j] ]->solution_value();
// //                     new_f_struct.x = x[ from_k_i_j_to_x_index_[ it->first ][i][j] ]->solution_value();
// //                     new_f_struct.y = y[ from_k_i_j_to_y_and_f_index_[ it->first ][i][j] ]->solution_value();
// //                     new_f_struct.k = k;
// //                     new_f_struct.i = from_matrix_index_to_graph_index_[i];
// //                     new_f_struct.j = from_matrix_index_to_graph_index_[j];
// //                     f_struct_vector.push_back(new_f_struct);
// //                 }
// //             }
// //         }

// //         std::sort(f_struct_vector.begin(), f_struct_vector.end(), [](const fStruct& a, const fStruct& b) { 
// //             return a.f > b.f; 
// //         }); // Sort in descending order.

// //         aerialcore_msgs::FlightPlan current_flight_plan;

// //         for (const auto& f_struct : f_struct_vector) {
// //             current_flight_plan.nodes.push_back(f_struct.i);
// //         }
// //         if (f_struct_vector.size()>0) {
// //             current_flight_plan.uav_id = _graph[ f_struct_vector.front().i ].id;
// //             current_flight_plan.nodes.push_back( f_struct_vector.back().j );
// //         }

// //         if (current_flight_plan.nodes.size()>3) {   // Consider the flight plan only if it visits two or more pylons.
// // # ifdef DEBUG
// //             std::cout << "current_flight_plan.uav_id: " << current_flight_plan.uav_id << std::endl;
// //             std::cout << "current_flight_plan.nodes:" << std::endl;

// //             for (int node : current_flight_plan.nodes) {
// //                 std::cout << node << std::endl;
// //             }

// //             bool milp_solution_coherent = f_struct_vector.front().i == f_struct_vector.back().j ? true : false;
// //             for (int index = 0; index<f_struct_vector.size()-1; index++) {
// //                 if (milp_solution_coherent) {
// //                     milp_solution_coherent = f_struct_vector[index].j == f_struct_vector[index + 1].i ? true : false;
// //                 } else if (!milp_solution_coherent) {
// //                     break;
// //                 }
// //             }
// //             std::cout << "milp_solution_coherent = " << milp_solution_coherent << std::endl;
// // # endif




//     //         std::vector< std::pair<int,int> > solution_tree_raw;    // First is the father node, second the current node (son).
//     //         std::vector<int> open_nodes;
//     //         std::vector<int> solution_nodes;

//     //         for (int e_i=0; e_i<edges_.size(); e_i++) {
//     //             if (edges_[e_i].type == aerialcore_msgs::EdgeType::TYPE_TAKEOFF_AND_NAVIGATION && f[ from_k_i_j_to_y_and_f_index_[ it->first ][i][j] ]->solution_value()>0.001) {
//     //                 std::pair<int,int> current_solution_tree_raw;
//     //                 current_solution_tree_raw.first = -1;                   // Takeoff node without father node.
//     //                 current_solution_tree_raw.second = edges_[e_i].i;
//     //                 solution_tree_raw.push_back(current_solution_tree_raw);

//     //                 open_nodes.push_back(edges_[e_i].i);
//     //                 break;
//     //             }
//     //         }

//     //         while (open_nodes.size() > 0) {
//     //             int current_open_node = open_nodes[0];
//     //             open_nodes.erase(open_nodes.begin());

//     //             for (int e_i=0; e_i<edges_.size(); e_i++) {
//     //                 if (edges_[e_i].j == current_open_node) {
//     //                     std::pair<int,int> current_solution_tree_raw;
//     //                     current_solution_tree_raw.first  = edges_[e_i].i;
//     //                     current_solution_tree_raw.second = edges_[e_i].j;
//     //                     solution_tree_raw.push_back(current_solution_tree_raw);

//     //                     current_open_nodes.push_back(edges_[e_i].j);
//     //                 }
//     //             }
//     //         }

//     //         for (int i=0; i<solution_tree_raw.size(); i++) {
//     //             int current_node = solution_tree_raw[i].second;
//     //             solution_nodes.push_back(current_node);
//     //             for (int j=current_node+1; j<solution_tree_raw.size(); j++) {
//     //                 if (solution_tree_raw[i].second == solution_tree_raw[j].second) {
//     //                 }
//     //             }
//     //         }

//     //         aerialcore_msgs::FlightPlan current_flight_plan;

//     //         for (int current_node : solution_nodes) {
//     //             current_flight_plan.nodes.push_back(current_node);
//     //         }
//     //         if (current_flight_plan.nodes.size()>0) {
//     //             current_flight_plan.uav_id = _graph[ current_flight_plan.nodes.front().i ].id;
//     //         }

//     //         flight_plans_.push_back(current_flight_plan);
//     //     }
//     // }

//     // Calculate the path free of obstacles between nodes outside in the Mission Controller. There is a path warantied between the nodes.

//     fillFlightPlansFields(flight_plans_);

    return flight_plans_;

} // end getPlanMILP method


void CentralizedPlanner::constructUAVs(const std::vector< std::tuple<float, float, float, int, int, int, int, int, int, bool, bool> >& _drone_info) {

    UAVs_.clear();

    // _drone_info it's a vector of tuples, each tuple with 10 elements. The first in the tuple is the initial battery, and so on with all the elements in the "UAV" structure defined here below.
    for (const std::tuple<float, float, float, int, int, int, int, int, int, bool, bool>& current_drone : _drone_info) {
        UAV actual_UAV;
        actual_UAV.initial_battery = std::get<0>(current_drone);
        // std::get<1>(current_drone) not being used.
        actual_UAV.minimum_battery = std::get<2>(current_drone);
        actual_UAV.time_until_fully_charged = std::get<3>(current_drone);
        actual_UAV.time_max_flying = std::get<4>(current_drone);
        actual_UAV.speed_xy = std::get<5>(current_drone);
        actual_UAV.speed_z_down = std::get<6>(current_drone);
        actual_UAV.speed_z_up = std::get<7>(current_drone);
        actual_UAV.id = std::get<8>(current_drone);
        actual_UAV.flying_or_landed_initially = std::get<9>(current_drone);
        actual_UAV.recharging_initially = std::get<10>(current_drone);
        UAVs_.push_back(actual_UAV);
    }
} // end constructUAVs


void CentralizedPlanner::constructEdges(std::vector<aerialcore_msgs::GraphNode>& _graph) {

    edges_.clear();
    from_k_i_j_to_x_index_.clear();
    from_k_i_j_to_y_and_f_index_.clear();

    // Construct edges_ that contain all the possible edges, with information of if it is a pure dead-heading edge or both inspection and dead-heading edge:
    x_index_size_ = 0;
    y_and_f_index_size_ = 0;
    for (std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > >::iterator it_k = time_cost_matrices_.begin(); it_k != time_cost_matrices_.end(); it_k++) {
        for (std::unordered_map<int, std::unordered_map<int, float> >::iterator it_i = it_k->second.begin(); it_i != it_k->second.end(); it_i++) {
            for (std::unordered_map<int, float>::iterator it_j = it_i->second.begin(); it_j != it_i->second.end(); it_j++) {
                if (time_cost_matrices_[ it_k->first ][ it_i->first ][ it_j->first ]==-1) {
                    continue;
                } else {
                    Edge new_edge;
                    new_edge.i = it_i->first;
                    new_edge.j = it_j->first;
                    new_edge.k = -1;

                    if (_graph[ new_edge.i ].type==aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION && _graph[ new_edge.j ].type==aerialcore_msgs::GraphNode::TYPE_PYLON ) {
                        if ( ! UAVs_[new_edge.k].flying_or_landed_initially )  {
                            new_edge.k = findUavIndexById( _graph[ new_edge.i ].id );
                            edges_[new_edge] = EdgeType::TYPE_TAKEOFF_AND_NAVIGATION;
                        }

                    } else if ( ( _graph[ new_edge.i ].type==aerialcore_msgs::GraphNode::TYPE_REGULAR_LAND_STATION || _graph[ new_edge.i ].type==aerialcore_msgs::GraphNode::TYPE_RECHARGE_LAND_STATION ) && _graph[ new_edge.j ].type==aerialcore_msgs::GraphNode::TYPE_PYLON ) {
                        edges_[new_edge] = EdgeType::TYPE_TAKEOFF_AND_NAVIGATION;

                    } else if ( _graph[ new_edge.i ].type==aerialcore_msgs::GraphNode::TYPE_PYLON && ( _graph[ new_edge.j ].type==aerialcore_msgs::GraphNode::/*TYPE_UAV_INITIAL_POSITION*/TYPE_REGULAR_LAND_STATION || _graph[ new_edge.j ].type==aerialcore_msgs::GraphNode::TYPE_RECHARGE_LAND_STATION ) ) {
                        edges_[new_edge] = EdgeType::TYPE_NAVIGATION_AND_LANDING;
                    } else {
                        edges_[new_edge] = EdgeType::TYPE_NAVIGATION;
                    }

                    if (edges_[new_edge] == EdgeType::TYPE_TAKEOFF_AND_NAVIGATION) {
                        from_k_i_j_to_y_and_f_index_[new_edge.k][new_edge.i][new_edge.j] = y_and_f_index_size_++;
                    } else {
                        for (std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
                            from_k_i_j_to_y_and_f_index_[ new_edge.k ][new_edge.i][new_edge.j] = y_and_f_index_size_++;
                        }
                    }
                }
            }
        }
        break;
    }
    for (int g_i=0; g_i<_graph.size(); g_i++) {
        if (_graph[g_i].type == aerialcore_msgs::GraphNode::TYPE_PYLON) {
            for (int j=0; j<_graph[g_i].connections_indexes.size(); j++) {
                Edge edge_struct;
                edge_struct.i = g_i;
                edge_struct.j = _graph[g_i].connections_indexes[j];
                edge_struct.k = -1;

                edges_[edge_struct] = EdgeType::TYPE_INSPECTION;

                for (std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
                    from_k_i_j_to_x_index_[ it->first ][edge_struct.i][edge_struct.j] = x_index_size_++;
                }
            }
        }
    }

# ifdef DEBUG
    std::cout << "Number of edges            = " << edges_.size() << std::endl;
    std::cout << "Number of inspection edges = " << x_index_size_/time_cost_matrices_.size() << std::endl;
    std::cout << "Number of k                = " << time_cost_matrices_.size() << std::endl;
    for (std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, float> > >::iterator it_k = time_cost_matrices_.begin(); it_k != time_cost_matrices_.end(); it_k++) {
        std::cout << "Number of i and j          = " << time_cost_matrices_[it_k->first].size() << std::endl;
        break;
    }
    int pylons_counter = 0;
    for (int g_i=0; g_i<_graph.size(); g_i++) {
        if (_graph[g_i].type == aerialcore_msgs::GraphNode::TYPE_PYLON) {
            pylons_counter++;
        }
    }
    std::cout << "Number of pylons           = " << pylons_counter << std::endl;
    int i = 0;
    for (std::map<Edge, EdgeType>::iterator it = edges_.begin(); it != edges_.end(); it++) {
        std::cout << "edges_[" << i << "].i         = " << it->first.i << std::endl;
        std::cout << "edges_[" << i << "].j         = " << it->first.j << std::endl;
        if (it->second == EdgeType::TYPE_INSPECTION) {
            std::cout << "edges_[" << i << "].type = INSPECTION" << std::endl;
        } else if (it->second == EdgeType::TYPE_NAVIGATION) {
            std::cout << "edges_[" << i << "].type = NAVIGATION" << std::endl;
        } else if (it->second == EdgeType::TYPE_TAKEOFF_AND_NAVIGATION) {
            std::cout << "edges_[" << i << "].type = TAKEOFF_AND_NAVIGATION" << std::endl;
        } else if (it->second == EdgeType::TYPE_NAVIGATION_AND_LANDING) {
            std::cout << "edges_[" << i << "].type = NAVIGATION_AND_LANDING" << std::endl;
        }
        i++;
    }
# endif
} // end constructEdges


void CentralizedPlanner::constructConnectionEdges(const std::vector<aerialcore_msgs::GraphNode>& _graph, std::map<int, int> _last_flight_plan_graph_node) {
    if (reset_connection_edges_ || connection_edges_.size()==0) {
        connection_edges_.clear();
        // Construct connection_edges from scratch:
        for (int i=0; i<_graph.size(); i++) {
            if (_graph[i].type == aerialcore_msgs::GraphNode::TYPE_PYLON) {
                for (int j=0; j<_graph[i].connections_indexes.size(); j++) {
                    if (i < _graph[i].connections_indexes[j]) {
                        std::pair <int,int> current_edge (i, _graph[i].connections_indexes[j]);
                        connection_edges_.push_back(current_edge);
                    }
                }
            }
        }

        reset_connection_edges_ = false;
    } else {    // Only for replanning.

        // For each UAV in the plan, delete from the inspection edges (connection_edges_) those inspected edges:
        for (int i=0; i<flight_plans_.size(); i++) {
            for (int j=0; j<flight_plans_[i].nodes.size()-1; j++) {
                if (flight_plans_[i].nodes[j] == _last_flight_plan_graph_node[ flight_plans_[i].uav_id ]) {
                    // If this node is already the last one that was visited don't keep deleting edges as the next ones weren't inspected yet.
                    break;
                }
                if (_graph[ flight_plans_[i].nodes[j] ].type==aerialcore_msgs::GraphNode::TYPE_PYLON && _graph[ flight_plans_[i].nodes[j+1] ].type==aerialcore_msgs::GraphNode::TYPE_PYLON) {
                    bool is_inspection_edge = false;
                    for (int k=0; k<_graph[ flight_plans_[i].nodes[j] ].connections_indexes.size(); k++) {
                        if (_graph[ flight_plans_[i].nodes[j] ].connections_indexes[k] == flight_plans_[i].nodes[j+1]) {
                            is_inspection_edge = true;
                            break;
                        }
                    }
                    if (!is_inspection_edge) {
                        continue;
                    }
                    // If code reached here, the edge being studied now from node[j] to node[j+1] is a inspection edge that has been covered (inspected).

                    std::pair <int,int> inspection_edge_covered;
                    inspection_edge_covered.first  = flight_plans_[i].nodes[j] < flight_plans_[i].nodes[j+1] ? flight_plans_[i].nodes[j] : flight_plans_[i].nodes[j+1];
                    inspection_edge_covered.second = flight_plans_[i].nodes[j] < flight_plans_[i].nodes[j+1] ? flight_plans_[i].nodes[j+1] : flight_plans_[i].nodes[j];

                    // Search for this inspection_edge_covered in the connection_edges_ vector and delete it.
                    for (int k=0; k<connection_edges_.size(); k++) {
                        if (inspection_edge_covered == connection_edges_[k]) {
                            connection_edges_.erase(connection_edges_.begin() + k);
                            break;
                        }
                    }

                }
            }
        }

    }
} // end constructConnectionEdges


void CentralizedPlanner::resetInspectedEdges() {
    reset_connection_edges_ = true;
} // end resetInspectedEdges


int CentralizedPlanner::findUavIndexById(int _UAV_id) {
    int uav_index = -1;
    for (int i=0; i<UAVs_.size(); i++) {
        if (UAVs_[i].id == _UAV_id) {
            uav_index = i;
            break;
        }
    }
    if (uav_index == -1) {
        ROS_ERROR("Centralized Planner: UAV id=%d provided not found on the Mission Controller.", _UAV_id);
    }
    return uav_index;
} // end findUavIndexById


int CentralizedPlanner::findTourIndexById(int _tour_id, const std::vector<Tour>& _R_tours_array) {
    int tour_index = -1;
    for (int i=0; i<_R_tours_array.size(); i++) {
        if (_R_tours_array[i].tour_id == _tour_id) {
            tour_index = i;
            break;
        }
    }
    return tour_index;
} // end findTourIndexById


std::vector<aerialcore_msgs::FlightPlan> CentralizedPlanner::getPlanVNS(std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector< std::tuple<float, float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::vector< geometry_msgs::Polygon >& _no_fly_zones, const geometry_msgs::Polygon& _geofence, const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices, const std::map<int, std::map<int, std::map<int, float> > >& _battery_drop_matrices, std::map<int, int> _last_flight_plan_graph_node) {

# ifdef NUMERICAL_EXPERIMENTS
    ros::Time time_just_before_planning = ros::Time::now();
    time_computing_plan_ = 0;
    accumulated_time_checking_solutions_ = 0;
    int number_of_shakes = 0;
# endif

    // Get the initial plan with the greedy method:
    std::vector<aerialcore_msgs::FlightPlan> current_solution;
    if (regular_or_fast_inspection_) {
        current_solution = getPlanGreedy(_graph, _drone_info, _no_fly_zones, _geofence, _time_cost_matrices, _battery_drop_matrices, _last_flight_plan_graph_node);
        // current_solution = getPlanMEM(_graph, _drone_info, _no_fly_zones, _geofence, _time_cost_matrices, _battery_drop_matrices, _last_flight_plan_graph_node);
    } else {
        current_solution = getPlanMinimaxGreedy(_graph, _drone_info, _no_fly_zones, _geofence, _time_cost_matrices, _battery_drop_matrices, _last_flight_plan_graph_node);
        // current_solution = getPlanMinimaxMEM(_graph, _drone_info, _no_fly_zones, _geofence, _time_cost_matrices, _battery_drop_matrices, _last_flight_plan_graph_node);
    }

    // TODO: right now VNS only consider inspection edges in the initial solution. Think solution? Should be enough as long as there are enough UAVs to cover all the graph.

    constructEdges(_graph);

# ifdef DEBUG
    std::cout << std::endl << "Printing initial plan (Greedy or MEM):" << std::endl;
    printPlan();
# ifdef NUMERICAL_EXPERIMENTS
    time_computing_plan_ = (ros::Time::now().toNSec()-time_just_before_planning.toNSec())*1.0e-9;
    std::cout << "Time computing initial plan (Greedy or MEM) [s] = " << time_computing_plan_ << std::endl;
    time_just_before_planning = ros::Time::now();
# endif
# endif

    ros::Time last_improvement_time = ros::Time::now();

    while ( ros::Time::now().toSec()-last_improvement_time.toSec()<=maximum_time_without_improvement_ ) {
# ifdef NUMERICAL_EXPERIMENTS
        number_of_shakes++;
# endif

        shake(current_solution);
        localSearch(current_solution);

        if ( ( regular_or_fast_inspection_ && ( solutionTimeCostTotal(current_solution) < solutionTimeCostTotal(flight_plans_) ) ) || ( !regular_or_fast_inspection_ && ( solutionTimeCostMaximum(current_solution) < solutionTimeCostMaximum(flight_plans_) ) ) ) {   // flight_plans_ stores the best solution up until now, if current solution is better, update it.
            flight_plans_.clear();
            flight_plans_ = current_solution;
            last_improvement_time = ros::Time::now();
        } else {
            current_solution.clear();
            current_solution = flight_plans_;
        }
    }

    fillFlightPlansFields(flight_plans_);

# ifdef DEBUG
    std::cout << std::endl << "Printing improved plan with VNS:" << std::endl;
    printPlan();
# ifdef NUMERICAL_EXPERIMENTS
    time_computing_plan_ = (ros::Time::now().toNSec()-time_just_before_planning.toNSec())*1.0e-9;
    std::cout << "Time computing with VNS the improved plan [s] = " << time_computing_plan_ << std::endl;
    std::cout << "accumulated_time_checking_solutions_ [s] = " << accumulated_time_checking_solutions_ << std::endl;
    double percentage_checking_solutions = accumulated_time_checking_solutions_/time_computing_plan_ * 100.0;
    std::cout << "percentage_checking_solutions        [%] = " << percentage_checking_solutions << std::endl;
    std::cout << "number_of_shakes                         = " << number_of_shakes << std::endl;
    int maximum_iterations = y_and_f_index_size_ * local_search_iteration_multiplier_;
    std::cout << "number_of_local_searchs_per_shake        = " << maximum_iterations << std::endl;
# endif
# endif

    return flight_plans_;

} // end getPlanVNS


float CentralizedPlanner::solutionTimeCostTotal(const std::vector<aerialcore_msgs::FlightPlan>& _flight_plans) {
# ifdef NUMERICAL_EXPERIMENTS
    ros::Time time_begin_check = ros::Time::now();
# endif

    float solution_time_cost = 0;

    for (int i=0; i<_flight_plans.size(); i++) {
        for (int j=0; j<_flight_plans[i].nodes.size()-1; j++) {
            solution_time_cost += time_cost_matrices_.at(_flight_plans[i].uav_id).at( _flight_plans[i].nodes[j] ).at( _flight_plans[i].nodes[j+1] );
        }
    }

# ifdef NUMERICAL_EXPERIMENTS
    accumulated_time_checking_solutions_ += (ros::Time::now().toNSec()-time_begin_check.toNSec())*1.0e-9;
# endif

    return solution_time_cost;
} // end solutionTimeCostTotal


float CentralizedPlanner::solutionTimeCostMaximum(const std::vector<aerialcore_msgs::FlightPlan>& _flight_plans) {

# ifdef NUMERICAL_EXPERIMENTS
    ros::Time time_begin_check = ros::Time::now();
# endif

    float solution_maximum_cost = 0;

    for (int i=0; i<_flight_plans.size(); i++) {
        float current_solution_maximum_cost = 0;
        for (int j=0; j<_flight_plans[i].nodes.size()-1; j++) {
            current_solution_maximum_cost += time_cost_matrices_.at(_flight_plans[i].uav_id).at( _flight_plans[i].nodes[j] ).at( _flight_plans[i].nodes[j+1] );
        }
        if (current_solution_maximum_cost > solution_maximum_cost) {
            solution_maximum_cost = current_solution_maximum_cost;
        }
    }

# ifdef NUMERICAL_EXPERIMENTS
    accumulated_time_checking_solutions_ += (ros::Time::now().toNSec()-time_begin_check.toNSec())*1.0e-9;
# endif

    return solution_maximum_cost;
} // end solutionTimeCostMaximum


bool CentralizedPlanner::solutionValidOrNot(const std::vector<aerialcore_msgs::FlightPlan>& _flight_plans) {

# ifdef NUMERICAL_EXPERIMENTS
    ros::Time time_begin_check = ros::Time::now();
# endif

    bool solution_battery_drop_valid_or_not = true;

    for (int i=0; i<_flight_plans.size(); i++) {
        float battery_left = UAVs_[i].initial_battery;
        for (int j=0; j<_flight_plans[i].nodes.size()-1; j++) {
            battery_left -= battery_drop_matrices_.at(_flight_plans[i].uav_id).at( _flight_plans[i].nodes[j] ).at( _flight_plans[i].nodes[j+1] );
        }
        if (battery_left < UAVs_[i].minimum_battery) {
            solution_battery_drop_valid_or_not = false;
            break;
        }
    }

# ifdef NUMERICAL_EXPERIMENTS
        accumulated_time_checking_solutions_ += (ros::Time::now().toNSec()-time_begin_check.toNSec())*1.0e-9;
# endif

    return solution_battery_drop_valid_or_not;
} // end solutionValidOrNot


void CentralizedPlanner::shake(std::vector<aerialcore_msgs::FlightPlan>& _flight_plans) {
    std::vector<aerialcore_msgs::FlightPlan> shaked;

    bool shaked_valid = false;
    while (!shaked_valid) {
        shaked = _flight_plans;

        std::vector< std::vector< std::pair<int,int> > > planned_inspection_edges = buildPlannedInspectionEdgesFromNodes(shaked);
        std::vector< std::vector< std::pair<int,int> > > planned_inspection_edges_original = planned_inspection_edges;

        // for (int i=0; i<planned_inspection_edges.size(); i++) { std::cout << std::endl << "planned_inspection_edges (shake 1, i="<< i << "): "; for (int j=0; j<planned_inspection_edges[i].size(); j++) { std::cout << planned_inspection_edges[i][j].first << " " << planned_inspection_edges[i][j].second << " , "; } } std::cout << std::endl;
        // std::cout << "solutionValidOrNot (shake 1): " << solutionValidOrNot(_flight_plans) << std::endl;
        // std::cout << "solutionTimeCostTotal (shake 1): " << solutionTimeCostTotal(_flight_plans) << std::endl;
        // std::cout << "solutionTimeCostMaximum (shake 1): " << solutionTimeCostMaximum(_flight_plans) << std::endl;

        int uav_from  = rand() % planned_inspection_edges.size();
        int uav_to    = rand() % planned_inspection_edges.size();
        int edge_path_from_begin = rand() % planned_inspection_edges[uav_from].size();
        int edge_path_from_end  = edge_path_from_begin + rand() % (planned_inspection_edges[uav_from].size()-edge_path_from_begin);
        int path_reversed       = rand() % 2;    // 0 if same direction, 1 if reversed.

        int initial_number_of_edges = planned_inspection_edges[uav_to].size() + planned_inspection_edges[uav_from].size();

        std::vector< std::pair<int,int> > path_from(planned_inspection_edges[uav_from].begin() + edge_path_from_begin, planned_inspection_edges[uav_from].begin() + edge_path_from_end +1);
        planned_inspection_edges[uav_from].erase(planned_inspection_edges[uav_from].begin() + edge_path_from_begin, planned_inspection_edges[uav_from].begin() + edge_path_from_end +1);
        if (path_reversed==1) {
            std::vector< std::pair <int,int> > reversed_path_from;
            for (int i=path_from.size()-1; i>=0; i--) {
                std::pair <int,int> reversed_edge (path_from[i].second, path_from[i].first);
                reversed_path_from.push_back(reversed_edge);
            }

            path_from.clear();
            path_from = reversed_path_from;
        }
        // std::cout << "path_from: "; for (int i=0; i<path_from.size(); i++) { std::cout << path_from[i].first << " " << path_from[i].second << " , " << std::endl; }

        int neighborhood_operator = rand() % 2;  // neighborhood_operator in the range 0 to 1
        int edge_path_to_begin = 0;
        int edge_path_to_end = 0;
        if ( planned_inspection_edges[uav_to].size()>0 ) {
            if (neighborhood_operator == 0) {
                edge_path_to_begin = rand() % (planned_inspection_edges[uav_to].size()+1);
                edge_path_to_end   = -1;
            } else {
                edge_path_to_begin = rand() % planned_inspection_edges[uav_to].size();
                edge_path_to_end   = edge_path_to_begin + rand() % (planned_inspection_edges[uav_to].size()-edge_path_to_begin);
            }
        }
        // std::cout << "uav_from                   = " << uav_from << std::endl;
        // std::cout << "uav_to                     = " << uav_to << std::endl;
        // std::cout << "edge_path_from_begin       = " << edge_path_from_begin << std::endl;
        // std::cout << "edge_path_from_end         = " << edge_path_from_end << std::endl;
        // std::cout << "edge_path_to_begin         = " << edge_path_to_begin << std::endl;
        // std::cout << "edge_path_to_end           = " << edge_path_to_end << std::endl;
        // std::cout << "path_reversed              = " << path_reversed << std::endl;
        // std::cout << "neighborhood_operator      = " << neighborhood_operator << std::endl;
        // for (int i=0; i<planned_inspection_edges.size(); i++) { std::cout << std::endl << "planned_inspection_edges (shake 2, i="<< i << "): "; for (int j=0; j<planned_inspection_edges[i].size(); j++) { std::cout << planned_inspection_edges[i][j].first << " " << planned_inspection_edges[i][j].second << " , "; } } std::cout << std::endl;

        if (neighborhood_operator==0 || planned_inspection_edges[uav_to].size()==0) {             // Path move.

            planned_inspection_edges[uav_to].insert(planned_inspection_edges[uav_to].begin() + edge_path_to_begin, path_from.begin(), path_from.end());

        } else if (neighborhood_operator==1) {      // Path exchange.

            std::vector< std::pair<int,int> > path_to(planned_inspection_edges[uav_to].begin() + edge_path_to_begin, planned_inspection_edges[uav_to].begin() + edge_path_to_end +1);
            planned_inspection_edges[uav_to].erase(planned_inspection_edges[uav_to].begin() + edge_path_to_begin, planned_inspection_edges[uav_to].begin() + edge_path_to_end +1);

            if (planned_inspection_edges[uav_to].size() >= edge_path_to_begin) {
                planned_inspection_edges[uav_to].insert(planned_inspection_edges[uav_to].begin() + edge_path_to_begin, path_from.begin(), path_from.end());
            } else {
                planned_inspection_edges[uav_to].insert(planned_inspection_edges[uav_to].begin(), path_from.begin(), path_from.end());
            }

            if (planned_inspection_edges[uav_from].size() >= edge_path_from_begin) {
                planned_inspection_edges[uav_from].insert(planned_inspection_edges[uav_from].begin() + edge_path_from_begin, path_to.begin(), path_to.end());
            } else {
                planned_inspection_edges[uav_from].insert(planned_inspection_edges[uav_from].begin(), path_to.begin(), path_to.end());
            }
        }

        // for (int i=0; i<planned_inspection_edges.size(); i++) { std::cout << std::endl << "planned_inspection_edges (shake 3, i="<< i << "): "; for (int j=0; j<planned_inspection_edges[i].size(); j++) { std::cout << planned_inspection_edges[i][j].first << " " << planned_inspection_edges[i][j].second << " , "; } } std::cout << std::endl;

        shaked = buildPlanNodesFromInspectionEdges(planned_inspection_edges, _flight_plans);

        // std::cout << "solutionValidOrNot (shake 3): " << solutionValidOrNot(shaked) << std::endl;
        // std::cout << "solutionTimeCostTotal (shake 3): " << solutionTimeCostTotal(shaked) << std::endl;
        // std::cout << "solutionTimeCostMaximum (shake 3): " << solutionTimeCostMaximum(shaked) << std::endl;

        shaked_valid = solutionValidOrNot(shaked);

        if (initial_number_of_edges != planned_inspection_edges[uav_to].size() + planned_inspection_edges[uav_from].size()) {
            ROS_ERROR("Centralized Planner: number of edges variation.");
            std::cout << "uav_from                   = " << uav_from << std::endl;
            std::cout << "uav_to                     = " << uav_to << std::endl;
            std::cout << "edge_path_from_begin       = " << edge_path_from_begin << std::endl;
            std::cout << "edge_path_from_end         = " << edge_path_from_end << std::endl;
            std::cout << "edge_path_to_begin         = " << edge_path_to_begin << std::endl;
            std::cout << "edge_path_to_end           = " << edge_path_to_end << std::endl;
            std::cout << "path_reversed              = " << path_reversed << std::endl;
            std::cout << "neighborhood_operator      = " << neighborhood_operator << std::endl;
            std::cout << "planned_inspection_edges_original (shake 1): "; for (int i=0; i<planned_inspection_edges_original.size(); i++) { for (int j=0; j<planned_inspection_edges_original[i].size(); j++) { std::cout << planned_inspection_edges_original[i][j].first << " " << planned_inspection_edges_original[i][j].second << " , "; } } std::cout << std::endl;
            std::cout << "planned_inspection_edges          (shake 2): "; for (int i=0; i<planned_inspection_edges.size(); i++) { for (int j=0; j<planned_inspection_edges[i].size(); j++) { std::cout << planned_inspection_edges[i][j].first << " " << planned_inspection_edges[i][j].second << " , "; } } std::cout << std::endl;
        }
    }

    _flight_plans.clear();
    _flight_plans = shaked;
} // end shake


void CentralizedPlanner::localSearch(std::vector<aerialcore_msgs::FlightPlan>& _flight_plans) {
    std::vector<aerialcore_msgs::FlightPlan> local_optimum = _flight_plans;

    int maximum_iterations = y_and_f_index_size_ * local_search_iteration_multiplier_;
    for (int iteration=0; iteration<maximum_iterations; iteration++) {
        std::vector< std::vector< std::pair<int,int> > > planned_inspection_edges = buildPlannedInspectionEdgesFromNodes(local_optimum);
        std::vector< std::vector< std::pair<int,int> > > planned_inspection_edges_original = planned_inspection_edges;
        // for (int i=0; i<planned_inspection_edges.size(); i++) { std::cout << std::endl << "planned_inspection_edges (localSearch 1, i="<< i << "): "; for (int j=0; j<planned_inspection_edges[i].size(); j++) { std::cout << planned_inspection_edges[i][j].first << " " << planned_inspection_edges[i][j].second << " , "; } } std::cout << std::endl;
        // std::cout << "solutionValidOrNot (localSearch 1): " << solutionValidOrNot(_flight_plans) << std::endl;
        // std::cout << "solutionTimeCostTotal (localSearch 1): " << solutionTimeCostTotal(_flight_plans) << std::endl;
        // std::cout << "solutionTimeCostMaximum (localSearch 1): " << solutionTimeCostMaximum(_flight_plans) << std::endl;

        int uav_from  = rand() % planned_inspection_edges.size();
        int uav_to    = rand() % planned_inspection_edges.size();
        int neighborhood_operator = rand() % 2; // neighborhood_operator in the range 0 to 1
        int edge_from = rand() % planned_inspection_edges[uav_from].size();
        int edge_to   = neighborhood_operator == 0 ? rand() % (planned_inspection_edges[uav_to].size()+1) : rand() % planned_inspection_edges[uav_to].size();
        int edge_reversed = rand() % 2;         // 0 if same direction, 1 if reversed.

        int initial_number_of_edges = planned_inspection_edges[uav_from].size() + planned_inspection_edges[uav_to].size();

        // std::cout << "uav_from              = " << uav_from << std::endl;
        // std::cout << "uav_to                = " << uav_to << std::endl;
        // std::cout << "edge_from             = " << edge_from << std::endl;
        // std::cout << "edge_to               = " << edge_to << std::endl;
        // std::cout << "edge_reversed         = " << edge_reversed << std::endl;
        // std::cout << "neighborhood_operator = " << neighborhood_operator << std::endl;

        if (neighborhood_operator==0) {             // One inspection edge move.

            // Insert the edge in the new position:
            if (edge_reversed==1) {
                std::pair <int,int> reversed_edge (planned_inspection_edges[uav_from][edge_from].second, planned_inspection_edges[uav_from][edge_from].first);
                planned_inspection_edges[uav_to].insert(planned_inspection_edges[uav_to].begin() + edge_to, reversed_edge);
            } else {
                planned_inspection_edges[uav_to].insert(planned_inspection_edges[uav_to].begin() + edge_to, planned_inspection_edges[uav_from][edge_from]);
            }

            // Erase the edge from the previous position:
            if (uav_from!=uav_to || edge_from<=edge_to) {
                planned_inspection_edges[uav_from].erase(planned_inspection_edges[uav_from].begin()+edge_from);
            } else {
                planned_inspection_edges[uav_from].erase(planned_inspection_edges[uav_from].begin()+edge_from+1);
            }

        } else if (neighborhood_operator==1) {      // One inspection edge exchange.

            iter_swap(planned_inspection_edges[uav_from].begin() + edge_from, planned_inspection_edges[uav_to].begin() + edge_to);

            if (edge_reversed==1) {
                std::pair <int,int> reversed_edge (planned_inspection_edges[uav_to][edge_to].second, planned_inspection_edges[uav_to][edge_to].first);
                planned_inspection_edges[uav_to][edge_to] = reversed_edge;
            }

        }

        local_optimum = buildPlanNodesFromInspectionEdges(planned_inspection_edges, _flight_plans);

        // for (int i=0; i<planned_inspection_edges.size(); i++) { std::cout << std::endl << "planned_inspection_edges (localSearch 2, i="<< i << "): "; for (int j=0; j<planned_inspection_edges[i].size(); j++) { std::cout << planned_inspection_edges[i][j].first << " " << planned_inspection_edges[i][j].second << " , "; } } std::cout << std::endl;
        // std::cout << "solutionValidOrNot (localSearch 2): " << solutionValidOrNot(local_optimum) << std::endl;
        // std::cout << "solutionTimeCostTotal (localSearch 2): " << solutionTimeCostTotal(local_optimum) << std::endl;
        // std::cout << "solutionTimeCostMaximum (localSearch 2): " << solutionTimeCostMaximum(local_optimum) << std::endl;

        if ( solutionValidOrNot(local_optimum) && (regular_or_fast_inspection_&&(solutionTimeCostTotal(local_optimum) < solutionTimeCostTotal(_flight_plans)) || !regular_or_fast_inspection_&&(solutionTimeCostMaximum(local_optimum) < solutionTimeCostMaximum(_flight_plans))) ) {   // _flight_plans stores the best solution up until now, if current solution is better, update it.
            _flight_plans.clear();
            _flight_plans = local_optimum;
        } else {
            local_optimum.clear();
            local_optimum = _flight_plans;
        }

        if (initial_number_of_edges != planned_inspection_edges[uav_from].size() + planned_inspection_edges[uav_to].size()) {
            ROS_ERROR("Centralized Planner: number of edges variation.");
            std::cout << "uav_from                   = " << uav_from << std::endl;
            std::cout << "uav_to                     = " << uav_to << std::endl;
            std::cout << "edge_from                  = " << edge_from << std::endl;
            std::cout << "edge_to                    = " << edge_to << std::endl;
            std::cout << "edge_reversed              = " << edge_reversed << std::endl;
            std::cout << "neighborhood_operator      = " << neighborhood_operator << std::endl;
            std::cout << "planned_inspection_edges_original (localSearch 1): "; for (int i=0; i<planned_inspection_edges_original.size(); i++) { for (int j=0; j<planned_inspection_edges_original[i].size(); j++) { std::cout << planned_inspection_edges_original[i][j].first << " " << planned_inspection_edges_original[i][j].second << " , "; } } std::cout << std::endl;
            std::cout << "planned_inspection_edges          (localSearch 2): "; for (int i=0; i<planned_inspection_edges.size(); i++) { for (int j=0; j<planned_inspection_edges[i].size(); j++) { std::cout << planned_inspection_edges[i][j].first << " " << planned_inspection_edges[i][j].second << " , "; } } std::cout << std::endl;
        }
    }

    // std::cout << "solutionValidOrNot (localSearch 3): " << solutionValidOrNot(local_optimum) << std::endl;
    // std::cout << "solutionTimeCostTotal (localSearch 3): " << solutionTimeCostTotal(local_optimum) << std::endl;
    // std::cout << "solutionTimeCostMaximum (localSearch 3): " << solutionTimeCostMaximum(local_optimum) << std::endl;
} // end localSearch


std::vector< std::vector< std::pair<int,int> > > CentralizedPlanner::buildPlannedInspectionEdgesFromNodes(const std::vector<aerialcore_msgs::FlightPlan>& _flight_plans) {

    std::vector< std::vector< std::pair<int,int> > > planned_inspection_edges;

    for (int i=0; i<_flight_plans.size(); i++) {
        std::vector< std::pair<int,int> > uav_planned_inspection_edges;

        for (int j=0; j<_flight_plans[i].nodes.size()-1; j++) {

            if (graph_[ _flight_plans[i].nodes[j] ].type == aerialcore_msgs::GraphNode::TYPE_PYLON && graph_[ _flight_plans[i].nodes[j+1] ].type == aerialcore_msgs::GraphNode::TYPE_PYLON) {
                // TODO: anidated loops, to enhance?
                for (int m=0; m<graph_.size(); m++) {
                    if (m==_flight_plans[i].nodes[j]) {
                        for (int n=0; n<graph_[m].connections_indexes.size(); n++) {
                            if (graph_[m].connections_indexes[n]==_flight_plans[i].nodes[j+1]) {
                                bool repeated = false;
                                for (int it=0; it<uav_planned_inspection_edges.size(); it++) {
                                    if ( (m==uav_planned_inspection_edges[it].first && graph_[m].connections_indexes[n]==uav_planned_inspection_edges[it].second) || (m==uav_planned_inspection_edges[it].second && graph_[m].connections_indexes[n]==uav_planned_inspection_edges[it].first) ) {
                                        repeated = true;
                                        break;
                                    }
                                }
                                if (!repeated) {
                                    std::pair <int,int> current_planned_edge (_flight_plans[i].nodes[j], _flight_plans[i].nodes[j+1]);
                                    uav_planned_inspection_edges.push_back(current_planned_edge);
                                }
                            }
                        }
                    }
                }
            }

        }

        planned_inspection_edges.push_back(uav_planned_inspection_edges);
    }

    return planned_inspection_edges;
} // end buildPlannedInspectionEdgesFromNodes


std::vector<aerialcore_msgs::FlightPlan> CentralizedPlanner::buildPlanNodesFromInspectionEdges(const std::vector< std::vector< std::pair<int,int> > >& _planned_inspection_edges, const std::vector<aerialcore_msgs::FlightPlan>& _flight_plans) {

    std::vector<aerialcore_msgs::FlightPlan> flight_plans_to_return = _flight_plans;

    for (int i=0; i<_flight_plans.size(); i++) {
        int first_node = _flight_plans[i].nodes.front();
        int last_node = _flight_plans[i].nodes.back();
        flight_plans_to_return[i].nodes.clear();
        flight_plans_to_return[i].nodes.push_back(first_node);
        for (int j=0; j<_planned_inspection_edges[i].size(); j++) {
            if (flight_plans_to_return[i].nodes.back() != _planned_inspection_edges[i][j].first) {
                flight_plans_to_return[i].nodes.push_back( _planned_inspection_edges[i][j].first );
            }
            flight_plans_to_return[i].nodes.push_back( _planned_inspection_edges[i][j].second );
        }
        flight_plans_to_return[i].nodes.push_back(last_node);
    }

    return flight_plans_to_return;
} // end buildPlanNodesFromInspectionEdges


std::vector<aerialcore_msgs::FlightPlan> CentralizedPlanner::getPlanMEM(std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector< std::tuple<float, float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::vector< geometry_msgs::Polygon >& _no_fly_zones, const geometry_msgs::Polygon& _geofence, const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices, const std::map<int, std::map<int, std::map<int, float> > >& _battery_drop_matrices, std::map<int, int> _last_flight_plan_graph_node) {

    graph_.clear();
    edges_.clear();
    time_cost_matrices_.clear();
    battery_drop_matrices_.clear();

    graph_ = _graph;

    constructUnordenedMaps(_time_cost_matrices, _battery_drop_matrices);

    if (_geofence.points.size()>0 && _no_fly_zones.size()>0) {
        std::vector< std::vector<geographic_msgs::GeoPoint> > obstacle_polygon_vector_geo;
        std::vector<geographic_msgs::GeoPoint> geofence_polygon_geo;

        for (geometry_msgs::Polygon no_fly_zone: _no_fly_zones) {
            std::vector<geographic_msgs::GeoPoint> no_fly_zone_geo;
            for (geometry_msgs::Point32 nfz_point: no_fly_zone.points) {
                geographic_msgs::GeoPoint nfz_point_geo = cartesian_to_geographic(nfz_point, map_origin_geo_);
                no_fly_zone_geo.push_back(nfz_point_geo);
            }
            obstacle_polygon_vector_geo.push_back(no_fly_zone_geo);
        }

        for (geometry_msgs::Point32 geofence_point: _geofence.points) {
            geographic_msgs::GeoPoint nfz_point_geo = cartesian_to_geographic(geofence_point, map_origin_geo_);
            geofence_polygon_geo.push_back(nfz_point_geo);
        }

        path_planner_ = grvc::PathPlanner(obstacle_polygon_vector_geo, geofence_polygon_geo);
    }

    constructUAVs(_drone_info);

    constructConnectionEdges(_graph, _last_flight_plan_graph_node);
    flight_plans_.clear();  // Need to be later than constructConnectionEdges()

    // constructEdges(_graph);

    // MEM algorithm (Agarwal, Line Coverage with Multiple Robots, ICRA 2020):
    // Modified for accepting heterogeneous fleet of UAVs.

    std::vector<Tour> R_tours_array;

    std::vector<Saving> S_savings;

    int latest_tour_id = 0;

    // Initialization of tours:
    for (int i=0; i<connection_edges_.size(); i++) {
        Tour new_tour;

        int first_node_direct,  first_node_reversed;
        int second_node_direct, second_node_reversed;
        int uav_id_direct,      uav_id_reversed;
        float cost_direct = 0;  float cost_reversed = 0;

        cost_direct += nearestGraphNodeUAVInitialPosition(connection_edges_[i].first, first_node_direct, uav_id_direct);
        cost_direct += time_cost_matrices_[uav_id_direct][connection_edges_[i].first][connection_edges_[i].second];
        cost_direct += nearestGraphNodeLandStation(connection_edges_[i].second, second_node_direct, uav_id_direct);

        cost_reversed += nearestGraphNodeUAVInitialPosition(connection_edges_[i].second, first_node_reversed, uav_id_reversed);
        cost_reversed += time_cost_matrices_[uav_id_reversed][connection_edges_[i].second][connection_edges_[i].first];
        cost_reversed += nearestGraphNodeLandStation(connection_edges_[i].first, second_node_reversed, uav_id_reversed);

        if (cost_direct < cost_reversed) {
            new_tour.uav_id = uav_id_direct;
            new_tour.nodes.push_back(first_node_direct);
            new_tour.nodes.push_back(connection_edges_[i].first);
            new_tour.nodes.push_back(connection_edges_[i].second);
            new_tour.nodes.push_back(second_node_direct);
            new_tour.cost = cost_direct;
        } else {
            new_tour.uav_id = uav_id_reversed;
            new_tour.nodes.push_back(first_node_reversed);
            new_tour.nodes.push_back(connection_edges_[i].second);
            new_tour.nodes.push_back(connection_edges_[i].first);
            new_tour.nodes.push_back(second_node_reversed);
            new_tour.cost = cost_reversed;
        }

        new_tour.demand = 0;
        new_tour.demand += battery_drop_matrices_[new_tour.uav_id][new_tour.nodes[0]][new_tour.nodes[1]];
        new_tour.demand += battery_drop_matrices_[new_tour.uav_id][new_tour.nodes[1]][new_tour.nodes[2]];
        new_tour.demand += battery_drop_matrices_[new_tour.uav_id][new_tour.nodes[2]][new_tour.nodes[3]];

        new_tour.tour_id = latest_tour_id++;

        R_tours_array.push_back(new_tour);
    }

    // Compute savings:
    for (int i=0; R_tours_array.size()>0 && i<R_tours_array.size()-1; i++) {
        for (int j=i+1; j<R_tours_array.size(); j++) {
            if (R_tours_array[i].uav_id==R_tours_array[j].uav_id) {

                Saving new_saving;
                new_saving.tour_id_1 = R_tours_array[i].tour_id;
                new_saving.tour_id_2 = R_tours_array[j].tour_id;

                Saving best_saving = bestSavingPermutation(new_saving, i, j, R_tours_array);
                if (best_saving.saving >= 0 && UAVs_[ findUavIndexById(best_saving.uav_id) ].initial_battery-best_saving.demand>=UAVs_[ findUavIndexById(best_saving.uav_id) ].minimum_battery) {   // Only insert the saving if it actually saves cost and the battery is enough.
                    S_savings.push_back(best_saving);
                }

            }
        }
    }

# ifdef DEBUG
    for (int i=0; i<R_tours_array.size(); i++) {
        std::cout << "R_tours_array["<< i <<"].cost     = " << R_tours_array[i].cost << std::endl;
        std::cout << "R_tours_array["<< i <<"].demand   = " << R_tours_array[i].demand << std::endl;
        std::cout << "R_tours_array["<< i <<"].tour_id  = " << R_tours_array[i].tour_id << std::endl;
        std::cout << "R_tours_array["<< i <<"].uav_id   = " << R_tours_array[i].uav_id << std::endl;
        for (int j=0; j<R_tours_array[i].nodes.size(); j++) {
            std::cout << "R_tours_array["<< i <<"].nodes["<< j << "] = " << R_tours_array[i].nodes[j] << std::endl;
        }
    }
    for (int i=0; i<S_savings.size(); i++) {
        std::cout << "S_savings["<< i <<"].cost      = " << S_savings[i].cost << std::endl;
        std::cout << "S_savings["<< i <<"].demand    = " << S_savings[i].demand << std::endl;
        std::cout << "S_savings["<< i <<"].uav_id    = " << S_savings[i].uav_id << std::endl;
        std::cout << "S_savings["<< i <<"].saving    = " << S_savings[i].saving << std::endl;
        std::cout << "S_savings["<< i <<"].tour_id_1 = " << S_savings[i].tour_id_1 << std::endl;
        std::cout << "S_savings["<< i <<"].tour_id_2 = " << S_savings[i].tour_id_2 << std::endl;
        for (int j=0; j<S_savings[i].nodes.size(); j++) {
            std::cout << "S_savings["<< i <<"].nodes["<< j << "] = " << S_savings[i].nodes[j] << std::endl;
        }
    }
# endif

    // Repeated Merge and Embed:
    while (S_savings.size()>0) {
        // Find the saving with the most saving, extract it and delete it:
        float best_saving_value = -1;
        int best_saving_index = -1;
        for (int i=0; i<S_savings.size(); i++) {
            if (S_savings[i].saving > best_saving_value) {
                best_saving_value = S_savings[i].saving;
                best_saving_index = i;
            }
        }
        Saving best_saving = S_savings[best_saving_index];
        S_savings.erase(S_savings.begin()+best_saving_index);

        // If the current saving involves tours already deleted, continue to the next iteration:
        if ( findTourIndexById(best_saving.tour_id_1, R_tours_array)==-1 || findTourIndexById(best_saving.tour_id_2, R_tours_array)==-1 ) {
            continue;
        }

        // Merge: Insert the tour of the best saving...
        Tour best_tour;
        best_tour.nodes = best_saving.nodes;
        best_tour.cost = best_saving.cost;
        best_tour.demand = best_saving.demand;
        best_tour.tour_id = latest_tour_id++;
        best_tour.uav_id = best_saving.uav_id;
        R_tours_array.push_back(best_tour);

        // ... and delete the tours that compose it.
        R_tours_array.erase(R_tours_array.begin() + findTourIndexById(best_saving.tour_id_1, R_tours_array) );
        R_tours_array.erase(R_tours_array.begin() + findTourIndexById(best_saving.tour_id_2, R_tours_array) );

        // Embed:
        int last_tour_index = R_tours_array.size()-1;
        for (int i=0; i<R_tours_array.size()-1; i++) {  // Don't iterate the last tour element (doesn't make sense to calculate the saving embeding it with itself).
            Saving new_saving;
            new_saving.tour_id_1 = R_tours_array[i].tour_id;
            new_saving.tour_id_2 = R_tours_array[last_tour_index].tour_id;

            Saving best_new_saving = bestSavingPermutation(new_saving, i, last_tour_index, R_tours_array);

            if (best_new_saving.saving >= 0 && UAVs_[ findUavIndexById(best_new_saving.uav_id) ].initial_battery-best_new_saving.demand>=UAVs_[ findUavIndexById(best_new_saving.uav_id) ].minimum_battery) {   // Only insert the saving if it actually saves cost and the battery is enough.
                S_savings.push_back(best_new_saving);
            }
        }

    }

    flight_plans_ = buildFlightPlansFromTours(R_tours_array);

    return flight_plans_;

} // end getPlanMEM


CentralizedPlanner::Saving CentralizedPlanner::bestSavingPermutation(const Saving& _saving_input, int _i, int _j, const std::vector<Tour>& _R_tours_array) {

    std::vector<Saving> saving_permutations;

    std::vector<int> aux_nodes_i, aux_nodes_j;  // vector of nodes without the first and last ones.
    for (int i=1; i<_R_tours_array[_i].nodes.size()-1; i++) {
        aux_nodes_i.push_back( _R_tours_array[_i].nodes[i] );
    }
    for (int i=1; i<_R_tours_array[_j].nodes.size()-1; i++) {
        aux_nodes_j.push_back( _R_tours_array[_j].nodes[i] );
    }

    // From _i straight to _j straight:
    Saving saving_permutation_1 = _saving_input;
    saving_permutation_1.nodes.clear();
    saving_permutation_1.nodes.insert(saving_permutation_1.nodes.end(), aux_nodes_i.begin(), aux_nodes_i.end());
    saving_permutation_1.nodes.insert(saving_permutation_1.nodes.end(), aux_nodes_j.begin(), aux_nodes_j.end());
    for (int i=0; i<saving_permutation_1.nodes.size()-1; i++) {
        if (saving_permutation_1.nodes[i]==saving_permutation_1.nodes[i+1]) {
            saving_permutation_1.nodes.erase( saving_permutation_1.nodes.begin() + i );
            break;
        }
    }
    calculateSaving(saving_permutation_1, _R_tours_array);
    saving_permutations.push_back(saving_permutation_1);

    // From _i straight to _j reversed:
    Saving saving_permutation_2 = _saving_input;
    saving_permutation_2.nodes.clear();
    saving_permutation_2.nodes.insert(saving_permutation_2.nodes.end(), aux_nodes_i.begin(), aux_nodes_i.end());
    saving_permutation_2.nodes.insert(saving_permutation_2.nodes.end(), aux_nodes_j.rbegin(), aux_nodes_j.rend());
    for (int i=0; i<saving_permutation_2.nodes.size()-1; i++) {
        if (saving_permutation_2.nodes[i]==saving_permutation_2.nodes[i+1]) {
            saving_permutation_2.nodes.erase( saving_permutation_2.nodes.begin() + i );
            break;
        }
    }
    calculateSaving(saving_permutation_2, _R_tours_array);
    saving_permutations.push_back(saving_permutation_2);

    // From _i reversed to _j straight:
    Saving saving_permutation_3 = _saving_input;
    saving_permutation_3.nodes.clear();
    saving_permutation_3.nodes.insert(saving_permutation_3.nodes.end(), aux_nodes_i.rbegin(), aux_nodes_i.rend());
    saving_permutation_3.nodes.insert(saving_permutation_3.nodes.end(), aux_nodes_j.begin(), aux_nodes_j.end());
    for (int i=0; i<saving_permutation_3.nodes.size()-1; i++) {
        if (saving_permutation_3.nodes[i]==saving_permutation_3.nodes[i+1]) {
            saving_permutation_3.nodes.erase( saving_permutation_3.nodes.begin() + i );
            break;
        }
    }
    calculateSaving(saving_permutation_3, _R_tours_array);
    saving_permutations.push_back(saving_permutation_3);

    // From _i reversed to _j reversed:
    Saving saving_permutation_4 = _saving_input;
    saving_permutation_4.nodes.clear();
    saving_permutation_4.nodes.insert(saving_permutation_4.nodes.end(), aux_nodes_i.rbegin(), aux_nodes_i.rend());
    saving_permutation_4.nodes.insert(saving_permutation_4.nodes.end(), aux_nodes_j.rbegin(), aux_nodes_j.rend());
    for (int i=0; i<saving_permutation_4.nodes.size()-1; i++) {
        if (saving_permutation_4.nodes[i]==saving_permutation_4.nodes[i+1]) {
            saving_permutation_4.nodes.erase( saving_permutation_4.nodes.begin() + i );
            break;
        }
    }
    calculateSaving(saving_permutation_4, _R_tours_array);
    saving_permutations.push_back(saving_permutation_4);

    // From _j straight to _i straight:
    Saving saving_permutation_5 = _saving_input;
    saving_permutation_5.nodes.clear();
    saving_permutation_5.nodes.insert(saving_permutation_5.nodes.end(), aux_nodes_j.begin(), aux_nodes_j.end());
    saving_permutation_5.nodes.insert(saving_permutation_5.nodes.end(), aux_nodes_i.begin(), aux_nodes_i.end());
    for (int i=0; i<saving_permutation_5.nodes.size()-1; i++) {
        if (saving_permutation_5.nodes[i]==saving_permutation_5.nodes[i+1]) {
            saving_permutation_5.nodes.erase( saving_permutation_5.nodes.begin() + i );
            break;
        }
    }
    calculateSaving(saving_permutation_5, _R_tours_array);
    saving_permutations.push_back(saving_permutation_5);

    // From _j straight to _i reversed:
    Saving saving_permutation_6 = _saving_input;
    saving_permutation_6.nodes.clear();
    saving_permutation_6.nodes.insert(saving_permutation_6.nodes.end(), aux_nodes_j.begin(), aux_nodes_j.end());
    saving_permutation_6.nodes.insert(saving_permutation_6.nodes.end(), aux_nodes_i.rbegin(), aux_nodes_i.rend());
    for (int i=0; i<saving_permutation_6.nodes.size()-1; i++) {
        if (saving_permutation_6.nodes[i]==saving_permutation_6.nodes[i+1]) {
            saving_permutation_6.nodes.erase( saving_permutation_6.nodes.begin() + i );
            break;
        }
    }
    calculateSaving(saving_permutation_6, _R_tours_array);
    saving_permutations.push_back(saving_permutation_6);

    // From _j reversed to _i straight:
    Saving saving_permutation_7 = _saving_input;
    saving_permutation_7.nodes.clear();
    saving_permutation_7.nodes.insert(saving_permutation_7.nodes.end(), aux_nodes_j.rbegin(), aux_nodes_j.rend());
    saving_permutation_7.nodes.insert(saving_permutation_7.nodes.end(), aux_nodes_i.begin(), aux_nodes_i.end());
    for (int i=0; i<saving_permutation_7.nodes.size()-1; i++) {
        if (saving_permutation_7.nodes[i]==saving_permutation_7.nodes[i+1]) {
            saving_permutation_7.nodes.erase( saving_permutation_7.nodes.begin() + i );
            break;
        }
    }
    calculateSaving(saving_permutation_7, _R_tours_array);
    saving_permutations.push_back(saving_permutation_7);

    // From _j reversed to _i reversed:
    Saving saving_permutation_8 = _saving_input;
    saving_permutation_8.nodes.clear();
    saving_permutation_8.nodes.insert(saving_permutation_8.nodes.end(), aux_nodes_j.rbegin(), aux_nodes_j.rend());
    saving_permutation_8.nodes.insert(saving_permutation_8.nodes.end(), aux_nodes_i.rbegin(), aux_nodes_i.rend());
    for (int i=0; i<saving_permutation_8.nodes.size()-1; i++) {
        if (saving_permutation_8.nodes[i]==saving_permutation_8.nodes[i+1]) {
            saving_permutation_8.nodes.erase( saving_permutation_8.nodes.begin() + i );
            break;
        }
    }
    calculateSaving(saving_permutation_8, _R_tours_array);
    saving_permutations.push_back(saving_permutation_8);

    // Calculate the permutation with the most saving and return it:
    float best_saving_value = 0;
    int best_saving_index = -1;
    for (int i=0; i<saving_permutations.size(); i++) {
        if (saving_permutations[i].saving >= best_saving_value) {
            best_saving_value = saving_permutations[i].saving;
            best_saving_index = i;
        }
    }

//     std::cout << "Permutation _R_tours_array[ " << _i << " ]:"<< std::endl;
//     for (int i=0; i<_R_tours_array[_i].nodes.size(); i++) {
//         std::cout << _R_tours_array[_i].nodes[i] << std::endl;
//     }
//     std::cout << "Permutation _R_tours_array[ " << _j << " ]:"<< std::endl;
//     for (int i=0; i<_R_tours_array[_j].nodes.size(); i++) {
//         std::cout << _R_tours_array[_j].nodes[i] << std::endl;
//     }
//     std::cout << "Permutation " << i << " " << saving_permutations[i].saving << std::endl;
//     for (int i=0; i<saving_permutations.size(); i++) {
//         for (int j=0; j<saving_permutations[i].nodes.size(); j++) {
//             std::cout << saving_permutations[i].nodes[j] << std::endl;
//         }
//     }

    if (best_saving_index!=-1) {
        return saving_permutations[best_saving_index];
    } else {
        Saving emtpy_saving;
        return emtpy_saving;
    }

} // end bestSavingPermutation


void CentralizedPlanner::calculateSaving(Saving& _saving, const std::vector<Tour>& _R_tours_array) {

    int first_node;
    int last_node;
    _saving.cost = 0;
    _saving.demand = 0;

    _saving.cost += nearestGraphNodeUAVInitialPosition(_saving.nodes.front(), first_node, _saving.uav_id);
    for (int i=0; i<_saving.nodes.size()-1; i++) {
        _saving.cost += time_cost_matrices_[_saving.uav_id][_saving.nodes[i]][_saving.nodes[i+1]];
    }
    _saving.cost += nearestGraphNodeLandStation(_saving.nodes.back(), last_node, _saving.uav_id);

    _saving.nodes.insert(_saving.nodes.begin(), first_node);
    _saving.nodes.push_back(last_node);

    for (int i=0; i<_saving.nodes.size()-1; i++) {
        _saving.demand += battery_drop_matrices_[_saving.uav_id][_saving.nodes[i]][_saving.nodes[i+1]];
    }

    _saving.saving = _R_tours_array[ findTourIndexById(_saving.tour_id_1, _R_tours_array) ].cost + _R_tours_array[ findTourIndexById(_saving.tour_id_2, _R_tours_array) ].cost - _saving.cost;

} // end calculateSaving


void CentralizedPlanner::fillFlightPlansFields(std::vector<aerialcore_msgs::FlightPlan>& _flight_plans) {

    // Fill the poses and their type:
    // Postprocess to calculate the path free of obstacles between nodes:
    for (int i=0; i<_flight_plans.size(); i++) {

        _flight_plans[i].poses.clear();
        _flight_plans[i].type.clear();

        bool previous_iteration_landing;

        for (int j=0; j<_flight_plans[i].nodes.size()-1; j++) {
            geographic_msgs::GeoPoint test_point_1, test_point_2;

            test_point_1.latitude  = graph_[ _flight_plans[i].nodes[j] ].latitude;
            test_point_1.longitude = graph_[ _flight_plans[i].nodes[j] ].longitude;
            test_point_1.altitude  = graph_[ _flight_plans[i].nodes[j] ].z;

            test_point_2.latitude  = graph_[ _flight_plans[i].nodes[j+1] ].latitude;
            test_point_2.longitude = graph_[ _flight_plans[i].nodes[j+1] ].longitude;
            test_point_2.altitude  = graph_[ _flight_plans[i].nodes[j+1] ].z;

            geometry_msgs::PoseStamped pose_to_insert;
            pose_to_insert.pose.position.x = graph_[ _flight_plans[i].nodes[j] ].x;
            pose_to_insert.pose.position.y = graph_[ _flight_plans[i].nodes[j] ].y;
            pose_to_insert.pose.position.z = graph_[ _flight_plans[i].nodes[j] ].altitude - map_origin_geo_.altitude + graph_[ _flight_plans[i].nodes[j] ].z;;
            _flight_plans[i].poses.push_back(pose_to_insert);

            // Fill the type of pose:
            if (j==0) {     // The first node, aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION, can be on the air or not:
                if (UAVs_[ findUavIndexById(_flight_plans[i].uav_id) ].flying_or_landed_initially) {
                    _flight_plans[i].type.push_back(aerialcore_msgs::FlightPlan::TYPE_PASS_PYLON_WP);
                    previous_iteration_landing = false;
                    _flight_plans[i].poses.back().pose.position.z = graph_[ _flight_plans[i].nodes[j] ].z;
                } else {
                    _flight_plans[i].type.push_back(aerialcore_msgs::FlightPlan::TYPE_TAKEOFF_WP);
                    previous_iteration_landing = false;
                    _flight_plans[i].poses.back().pose.position.z += graph_[ _flight_plans[i].nodes[j+1] ].z;       // Takeoff to the next node height.
                }
            } else {        // Any other node different from the first one:
                if (graph_[ _flight_plans[i].nodes[j] ].type==aerialcore_msgs::GraphNode::TYPE_RECHARGE_LAND_STATION || graph_[ _flight_plans[i].nodes[j] ].type==aerialcore_msgs::GraphNode::TYPE_REGULAR_LAND_STATION) {
                    if (previous_iteration_landing) {
                        _flight_plans[i].type.push_back(aerialcore_msgs::FlightPlan::TYPE_TAKEOFF_WP);
                        previous_iteration_landing = false;
                        _flight_plans[i].poses.back().pose.position.z += graph_[ _flight_plans[i].nodes[j+1] ].z;   // Takeoff to the next node height.
                    } else {
                        _flight_plans[i].type.push_back(aerialcore_msgs::FlightPlan::TYPE_LAND_WP);
                        previous_iteration_landing = true;
                    }
                } else if (graph_[ _flight_plans[i].nodes[j] ].type==aerialcore_msgs::GraphNode::TYPE_PYLON) {
                    _flight_plans[i].type.push_back(aerialcore_msgs::FlightPlan::TYPE_PASS_PYLON_WP);
                    previous_iteration_landing = false;
                }
            }

            if (!path_planner_.checkIfTwoPointsAreVisible(test_point_1, test_point_2)) {

                if (graph_[ _flight_plans[i].nodes[j+1] ].type==aerialcore_msgs::GraphNode::TYPE_RECHARGE_LAND_STATION || graph_[ _flight_plans[i].nodes[j+1] ].type==aerialcore_msgs::GraphNode::TYPE_REGULAR_LAND_STATION) {
                    test_point_2.altitude  = test_point_1.altitude;
                }

                std::vector<geographic_msgs::GeoPoint> path = path_planner_.getPathWithRelativeAltitude(test_point_1, test_point_2, map_origin_geo_.altitude);
                int path_size = path.size();    // If not like this .size() treated as unsigned int and can't be negative.

                for (int k=0; k<path_size-1; k++) {
                    geometry_msgs::Point32 cartesian_point = geographic_to_cartesian(path[k], map_origin_geo_);
                    cartesian_point.z = path[k].altitude;

                    geometry_msgs::PoseStamped pose_to_insert;
                    pose_to_insert.pose.position.x = cartesian_point.x;
                    pose_to_insert.pose.position.y = cartesian_point.y;
                    pose_to_insert.pose.position.z = cartesian_point.z;
                    _flight_plans[i].poses.push_back(pose_to_insert);

                    _flight_plans[i].type.push_back(aerialcore_msgs::FlightPlan::TYPE_PASS_NFZ_WP);
                }

            }
        }

        geometry_msgs::PoseStamped pose_to_insert;
        pose_to_insert.pose.position.x = graph_[ _flight_plans[i].nodes.back() ].x;
        pose_to_insert.pose.position.y = graph_[ _flight_plans[i].nodes.back() ].y;
        pose_to_insert.pose.position.z = graph_[ _flight_plans[i].nodes.back() ].altitude - map_origin_geo_.altitude + graph_[ _flight_plans[i].nodes.back() ].z;
        _flight_plans[i].poses.push_back(pose_to_insert);

        _flight_plans[i].type.push_back(aerialcore_msgs::FlightPlan::TYPE_LAND_WP);
    }

    ros::Time planning_time = ros::Time::now();
    for (int i=0; i<_flight_plans.size(); i++) {
        _flight_plans[i].header.stamp = planning_time;
    }

} // end fillFlightPlansFields


std::vector<aerialcore_msgs::FlightPlan> CentralizedPlanner::buildFlightPlansFromTours(const std::vector<Tour>& _R_tours_array) {
    std::vector<aerialcore_msgs::FlightPlan> flight_plans;

    for (int i=0; i<_R_tours_array.size(); i++) {
        aerialcore_msgs::FlightPlan current_flight_plan;
        current_flight_plan.uav_id = _R_tours_array[i].uav_id;
        for (int j=0; j<_R_tours_array[i].nodes.size(); j++) {
            current_flight_plan.nodes.push_back(_R_tours_array[i].nodes[j]);
        }
        flight_plans.push_back(current_flight_plan);
    }

    fillFlightPlansFields(flight_plans);

    return flight_plans;
} // buildFlightPlansFromTours


std::vector<aerialcore_msgs::FlightPlan> CentralizedPlanner::getPlanMinimaxGreedy(std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector< std::tuple<float, float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::vector< geometry_msgs::Polygon >& _no_fly_zones, const geometry_msgs::Polygon& _geofence, const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices, const std::map<int, std::map<int, std::map<int, float> > >& _battery_drop_matrices, std::map<int, int> _last_flight_plan_graph_node) {

    // Run the regular greedy solver, but stopping when each UAV has it's % of inpection edges asignated.

    regular_or_fast_inspection_ = false;
    std::vector<aerialcore_msgs::FlightPlan> solution = getPlanGreedy(_graph, _drone_info, _no_fly_zones, _geofence, _time_cost_matrices, _battery_drop_matrices, _last_flight_plan_graph_node);
    regular_or_fast_inspection_ = true;
    return solution;
} // end getPlanMinimaxGreedy


std::vector<aerialcore_msgs::FlightPlan> CentralizedPlanner::getPlanMinimaxMEM(std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector< std::tuple<float, float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::vector< geometry_msgs::Polygon >& _no_fly_zones, const geometry_msgs::Polygon& _geofence, const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices, const std::map<int, std::map<int, std::map<int, float> > >& _battery_drop_matrices, std::map<int, int> _last_flight_plan_graph_node) {
    // TODO
    return flight_plans_;
} // end getPlanMinimaxMEM


std::vector<aerialcore_msgs::FlightPlan> CentralizedPlanner::getPlanMinimaxVNS(std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector< std::tuple<float, float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::vector< geometry_msgs::Polygon >& _no_fly_zones, const geometry_msgs::Polygon& _geofence, const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices, const std::map<int, std::map<int, std::map<int, float> > >& _battery_drop_matrices, std::map<int, int> _last_flight_plan_graph_node) {
    regular_or_fast_inspection_ = false;
    std::vector<aerialcore_msgs::FlightPlan> solution = getPlanVNS(_graph, _drone_info, _no_fly_zones, _geofence, _time_cost_matrices, _battery_drop_matrices, _last_flight_plan_graph_node);
    regular_or_fast_inspection_ = true;
    return solution;
} // end getPlanMinimaxVNS


void CentralizedPlanner::constructUnordenedMaps(const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices, const std::map<int, std::map<int, std::map<int, float> > >& _battery_drop_matrices) {
    time_cost_matrices_.clear();
    battery_drop_matrices_.clear();
    for (std::map<int, std::map<int, std::map<int, float> > >::const_iterator it_k = _time_cost_matrices.begin(); it_k != _time_cost_matrices.end(); it_k++) {
        std::unordered_map<int, std::unordered_map<int, float> > aux_time_1;
        std::unordered_map<int, std::unordered_map<int, float> > aux_batt_1;
        for (std::map<int, std::map<int, float> >::const_iterator it_i = it_k->second.begin(); it_i != it_k->second.end(); it_i++) {
            std::unordered_map<int, float> aux_time_2;
            std::unordered_map<int, float> aux_batt_2;
            for (std::map<int, float>::const_iterator it_j = it_i->second.begin(); it_j != it_i->second.end(); it_j++) {
                aux_time_2[it_j->first] = _time_cost_matrices.at(it_k->first).at(it_i->first).at(it_j->first);
                aux_batt_2[it_j->first] = _battery_drop_matrices.at(it_k->first).at(it_i->first).at(it_j->first);
            }
            aux_time_1[it_i->first] = aux_time_2;
            aux_batt_1[it_i->first] = aux_batt_2;
        }
        time_cost_matrices_[it_k->first] = aux_time_1;
        battery_drop_matrices_[it_k->first] = aux_batt_1;
    }
} // end constructUnordenedMaps


} // end namespace aerialcore
