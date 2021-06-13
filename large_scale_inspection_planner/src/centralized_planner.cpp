/**
 * AERIALCORE Project:
 *
 * Centralized planner.
 * 
 */

#include <centralized_planner.h>

#include <ros/ros.h>
#include <math.h>

#include "ortools/linear_solver/linear_solver.h"
#include "ortools/base/logging.h"

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

#define DEBUG       // UNCOMMENT FOR PRINTING VISUALIZATION OF RESULTS (DEBUG MODE)

namespace aerialcore {


// Brief Constructor
CentralizedPlanner::CentralizedPlanner() : path_planner_() {
    ros::NodeHandle nh;

    // map_origin_geo is the geographic coordinate origin of the cartesian coordinates. Loaded to the param server in the YAML config file.
    std::vector<double> map_origin_geo_vector;
    nh.getParam("map_origin_geo", map_origin_geo_vector);
    map_origin_geo_.latitude  = map_origin_geo_vector[0];
    map_origin_geo_.longitude = map_origin_geo_vector[1];
    map_origin_geo_.altitude  = map_origin_geo_vector[2];
}


// Brief Destructor
CentralizedPlanner::~CentralizedPlanner() {}


std::vector<aerialcore_msgs::FlightPlan> CentralizedPlanner::getPlanGreedy(std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector< std::tuple<float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::vector< geometry_msgs::Polygon >& _no_fly_zones, const geometry_msgs::Polygon& _geofence, const std::map<int, std::vector< std::vector<float> > >& _time_cost_matrices, const std::map<int, std::vector< std::vector<float> > >& _battery_drop_matrices) {

    graph_.clear();
    edges_pairs_.clear();
    edges_.clear();
    flight_plans_.clear();
    time_cost_matrices_.clear();
    battery_drop_matrices_.clear();

    graph_ = _graph;
    time_cost_matrices_ = _time_cost_matrices;
    battery_drop_matrices_ = _battery_drop_matrices;

    if (_geofence.points.size()>0 && _no_fly_zones.size()>0) {
        path_planner_ = grvc::PathPlanner(_no_fly_zones, _geofence);
    }

    constructUAVs(_drone_info);
    constructEdges(_graph);

    // Construct edges_pairs_:
    for (int i=0; i<graph_.size(); i++) {
        if (graph_[i].type == aerialcore_msgs::GraphNode::TYPE_PYLON) {
            for (int j=0; j<graph_[i].connections_indexes.size(); j++) {
                if (i < graph_[i].connections_indexes[j]) {
                    std::pair <int,int> current_edge (i, graph_[i].connections_indexes[j]);
                    edges_pairs_.push_back(current_edge);
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

        int index_edge_to_erase = -1;

        int index_graph_land_station_from_next_pylon;
        int index_graph_land_station_from_pylon_last;

        // Calculate closest pylon from the UAV initial pose:
        nearestGraphNodePylon(uav_initial_position_graph_index, index_graph_of_next_pylon, current_uav.id);

        // Calculate closest land station from that closest pylon:
        nearestGraphNodeLandStation(index_graph_of_next_pylon, index_graph_land_station_from_next_pylon, current_uav.id);

        while (edges_pairs_.size()>0 && (battery
            - battery_drop_matrices_[ current_uav.id ][ from_graph_index_to_matrix_index_[ uav_initial_position_graph_index ] ][ from_graph_index_to_matrix_index_[ index_graph_of_next_pylon ] ]
            - battery_drop_matrices_[ current_uav.id ][ from_graph_index_to_matrix_index_[ index_graph_of_next_pylon ] ][ from_graph_index_to_matrix_index_[ index_graph_land_station_from_next_pylon ] ] ) ) {
            // Insert pylon because it can be reached with enough battery to go later to a land station:
            current_flight_plan.nodes.push_back(index_graph_of_next_pylon);

            if (index_edge_to_erase != -1) {
                edges_pairs_.erase( edges_pairs_.begin() + index_edge_to_erase );
            }

            // Update the battery left:            
            battery -= battery_drop_matrices_[ current_uav.id ][ from_graph_index_to_matrix_index_[ current_flight_plan.nodes.back() ] ][ from_graph_index_to_matrix_index_[ index_graph_of_next_pylon ] ];

            // Calculate pylon with most benefit from current pylon:
            mostRewardedPylon(current_flight_plan.nodes.back(), index_graph_of_next_pylon, index_edge_to_erase, current_uav.id);

            index_graph_land_station_from_pylon_last = index_graph_land_station_from_next_pylon;

            if (index_graph_of_next_pylon == -1) { // No next pylon connected with unserved edges, search pylons not connected to this one.
                int index_pylon_connected_with_unserved_edge;
                nearestGraphNodePylon(current_flight_plan.nodes.back(), index_graph_of_next_pylon, current_uav.id);
                if (index_graph_of_next_pylon == -1) break; // No next pylon at all with unserved edges.
                else {  // Pylon has edges unserved connected.
                    mostRewardedPylon(index_graph_of_next_pylon, index_pylon_connected_with_unserved_edge, index_edge_to_erase, current_uav.id);
                    nearestGraphNodeLandStation(index_pylon_connected_with_unserved_edge, index_graph_land_station_from_next_pylon, current_uav.id);
                    if ( (index_pylon_connected_with_unserved_edge!=-1) && (index_graph_land_station_from_next_pylon!=-1) && (battery
                        - battery_drop_matrices_[ current_uav.id ][ from_graph_index_to_matrix_index_[ current_flight_plan.nodes.back() ] ][ from_graph_index_to_matrix_index_[ index_pylon_connected_with_unserved_edge ] ]
                        - battery_drop_matrices_[ current_uav.id ][ from_graph_index_to_matrix_index_[ index_pylon_connected_with_unserved_edge ] ][ from_graph_index_to_matrix_index_[ index_graph_land_station_from_next_pylon ] ] ) ) {
                        current_flight_plan.nodes.push_back(index_graph_of_next_pylon);
                        battery -= battery_drop_matrices_[ current_uav.id ][ from_graph_index_to_matrix_index_[ current_flight_plan.nodes.back() ] ][ from_graph_index_to_matrix_index_[ index_graph_of_next_pylon ] ];
                        index_graph_of_next_pylon = index_pylon_connected_with_unserved_edge;
                    } else {    // Only insert the next pylon if it has battery to at least fulfill one edge.
                        break;
                    }
                }
            } else {
                // Calculate closest land station from that most rewarded pylon:
                nearestGraphNodeLandStation(index_graph_of_next_pylon, index_graph_land_station_from_next_pylon, current_uav.id);
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


// Brief method that returns the index of the nearest land station graph node given an inital index.
void CentralizedPlanner::nearestGraphNodeLandStation(int _from_this_index_graph, int& _index_graph_node_to_return, int _uav_id) {

    _index_graph_node_to_return = -1;
    float time_cost = std::numeric_limits<float>::max();

    for (int i=0; i<graph_.size(); i++) {
        if (graph_[i].type==aerialcore_msgs::GraphNode::TYPE_RECHARGE_LAND_STATION || graph_[i].type==aerialcore_msgs::GraphNode::TYPE_REGULAR_LAND_STATION) {

            float current_time_cost = time_cost_matrices_[_uav_id][_from_this_index_graph][i];
            if (current_time_cost <= 0.001) continue;     // It's the same graph node or path not found, ignore and continue.

            if ( time_cost > current_time_cost ) {
                _index_graph_node_to_return = i;
                time_cost = current_time_cost;
            }
        }
    }

} // end nearestGraphNodeLandStation method


// Brief method that returns the index of the nearest pylon graph node given an inital index. The new pylon will have edges unserved and will not be directly connected.
void CentralizedPlanner::nearestGraphNodePylon(int _from_this_index_graph, int& _index_graph_node_to_return, int _uav_id) {

    _index_graph_node_to_return = -1;
    float time_cost = std::numeric_limits<float>::max();

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

            float current_time_cost = time_cost_matrices_[_uav_id][_from_this_index_graph][i];
            if (current_time_cost <= 0.001) continue;     // It's the same graph node or path not found, ignore and continue.

            // Only consider i (current node iterated) if it has edges unserved:
            bool has_edges_unserved = false;
            for (const int& current_connection_index : graph_[i].connections_indexes) {
                std::pair <int,int> current_edge;
                current_edge.first =  i < current_connection_index ? i : current_connection_index;
                current_edge.second = i < current_connection_index ? current_connection_index : i;
                for (int j=0; j<edges_pairs_.size(); j++) {
                    if (edges_pairs_[j] == current_edge) {
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

} // end nearestGraphNodePylon method


// Brief method that returns the index of the furthest connected pylon from an initial pylon.
void CentralizedPlanner::mostRewardedPylon(int _initial_pylon_index, int& _index_graph_node_to_return, int& _index_edge_to_erase, int _uav_id) {

    _index_graph_node_to_return = -1;
    float time_cost = 0;
    _index_edge_to_erase = -1;

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
        for (i=0; i<edges_pairs_.size(); i++) {
            if (edges_pairs_[i] == current_edge) {
                edge_found = true;
                break;
            }
        }
        if (!edge_found) {
            continue;
        }

        float current_time_cost = time_cost_matrices_[_uav_id][_initial_pylon_index][i];
        if (current_time_cost <= 0.001) continue;     // It's the same graph node or path not found, ignore and continue.

        if ( time_cost < current_time_cost ) {
            _index_graph_node_to_return = current_connection_index;
            time_cost = current_time_cost;
            _index_edge_to_erase = i;
        }
    }

} // end mostRewardedPylon method


// Brief method to print the plan in the terminal.
void CentralizedPlanner::printPlan() {
    std::cout << std::endl;
    std::cout << "Printing flight plan from the planner:" << std::endl;
    std::cout << std::endl;
    for (int i=0; i<flight_plans_.size(); i++) {
        std::cout << "flight_plans[ " << i << " ].id = " << flight_plans_[i].uav_id << std::endl;
        for (int j=0; j<flight_plans_[i].nodes.size(); j++) {
            std::cout << "flight_plans[ " << i << " ].nodes[ " << j << " ] = " << flight_plans_[i].nodes[j] << std::endl;
        }
        for (int j=0; j<flight_plans_[i].edges.size(); j++) {
            std::cout << "flight_plans[ " << i << " ].edges[ " << j << " ] = " << flight_plans_[i].edges[j] << std::endl;
        }
    }
    std::cout << std::endl;
} // end printPlan method


std::vector<aerialcore_msgs::FlightPlan> CentralizedPlanner::getPlanMILP(std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector< std::tuple<float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::vector< geometry_msgs::Polygon >& _no_fly_zones, const geometry_msgs::Polygon& _geofence, const std::map<int, std::vector< std::vector<float> > >& _time_cost_matrices, const std::map<int, std::vector< std::vector<float> > >& _battery_drop_matrices) {

    flight_plans_.clear();
    time_cost_matrices_.clear();
    battery_drop_matrices_.clear();

    constructUAVs(_drone_info);
    constructEdges(_graph);

    time_cost_matrices_ = _time_cost_matrices;
    battery_drop_matrices_ = _battery_drop_matrices;

    //////////////////// Start defining the MILP problem according to agarwal_icra20 and solve it using OR-Tools ////////////////////

    // Create the mip solver with the SCIP backend:
    std::unique_ptr<operations_research::MPSolver> solver(operations_research::MPSolver::CreateSolver("SCIP"));

    const double infinity = solver->infinity();

    // Variables:
    // x[x_index_size_]       is a vector of binary variables.
    // y[y_and_f_index_size_] is a vector of non-negative integer variables.
    // f[y_and_f_index_size_] is a vector of non-negative continuous variables.
    std::vector<operations_research::MPVariable *> x;
    std::vector<operations_research::MPVariable *> y;
    std::vector<operations_research::MPVariable *> f;
    for (int index=0; index<x_index_size_; index++) {
        x.push_back( solver->MakeBoolVar("") );
    }
    for (int index=0; index<y_and_f_index_size_; index++) {
        y.push_back( solver->MakeIntVar(0, infinity, "") );
        f.push_back( solver->MakeNumVar(0.0, 1.0, "") );
    }
    LOG(INFO) << "Number of variables = " << solver->NumVariables();

    // Create the objective function:
    // (1) Minimize all the inspection time (UAVs flying separately).
    operations_research::MPObjective *const objective = solver->MutableObjective();
    for (std::map<int, std::vector< std::vector<float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
        for (int e_i=0; e_i<edges_.size(); e_i++) {
            if (edges_[e_i].type == aerialcore_msgs::Edge::TYPE_INSPECTION) {
                objective->SetCoefficient(x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], it->second[ edges_[e_i].i ][ edges_[e_i].j ]);
            }
            if ( (edges_[e_i].type != aerialcore_msgs::Edge::TYPE_TAKEOFF_AND_NAVIGATION && edges_[e_i].type != aerialcore_msgs::Edge::TYPE_NAVIGATION_AND_LANDING) || it->first==edges_[e_i].k) {
                objective->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], it->second[ edges_[e_i].i ][ edges_[e_i].j ]);
            }
        }
    }
    objective->SetMinimization();


    // Create the constraints:

    // (2) For all pylon nodes and tours-drones, Sum_all_j(x_ij_k) plus Sum_all_j(y_ij_k) minus Sum_all_j(x_ji_k) and minus Sum_all_j(x_ji_k) is equal to zero (inputs equal to outputs in each node):
    for (std::map<int, std::vector< std::vector<float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
        for (int g_i=0; g_i<_graph.size(); g_i++) {
            if (_graph[g_i].type == aerialcore_msgs::GraphNode::TYPE_PYLON) {
                operations_research::MPConstraint *constraint = solver->MakeRowConstraint(0, 0, "");
                for (int e_i=0; e_i<edges_.size(); e_i++) {
                    if (edges_[e_i].i == from_graph_index_to_matrix_index_[g_i]) {
                        if (edges_[e_i].type == aerialcore_msgs::Edge::TYPE_INSPECTION) {
                            constraint->SetCoefficient(x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], 1);
                            constraint->SetCoefficient(x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].j ][ edges_[e_i].i ] ], -1);
                        }
                        if (edges_[e_i].type != aerialcore_msgs::Edge::TYPE_TAKEOFF_AND_NAVIGATION && edges_[e_i].type != aerialcore_msgs::Edge::TYPE_NAVIGATION_AND_LANDING) {
                            constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], 1);
                            constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].j ][ edges_[e_i].i ] ], -1);
                        }
                    }
                }
            } else if (_graph[g_i].type == aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION) {
                operations_research::MPConstraint *constraint = solver->MakeRowConstraint(0, 0, "");
                for (int e_i=0; e_i<edges_.size(); e_i++) {
                    if (edges_[e_i].i == from_graph_index_to_matrix_index_[g_i]) {
                        if (it->first==edges_[e_i].k) {
                            constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], 1);
                            constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].j ][ edges_[e_i].i ] ], -1);
                        }
                    }
                }
            }
        }
    }

    // (3) For all inspection edges, x_ij plus x_ji sums 1 (all inspection edges covered in any direction):
    for (int e_i=0; e_i<edges_.size(); e_i++) {
        if (edges_[e_i].type == aerialcore_msgs::Edge::TYPE_INSPECTION && edges_[e_i].i<edges_[e_i].j) {
            operations_research::MPConstraint *constraint = solver->MakeRowConstraint(1, 1, "");
            for (std::map<int, std::vector< std::vector<float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
                constraint->SetCoefficient(x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], 1);
                constraint->SetCoefficient(x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].j ][ edges_[e_i].i ] ], 1);
            }
        }
    }

    // (4) For all tours-drones, in edges coming from takeoff nodes, y_0j is 0 or 1 (take off just once or zero times in each tour):
    for (std::map<int, std::vector< std::vector<float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
        operations_research::MPConstraint *constraint = solver->MakeRowConstraint(0, 1, "");
        for (int e_i=0; e_i<edges_.size(); e_i++) {
            if (edges_[e_i].type == aerialcore_msgs::Edge::TYPE_TAKEOFF_AND_NAVIGATION && it->first==edges_[e_i].k) {
                constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], 1);
            }
        }
    }

    // // (4.1) For all tours-drones, in edges going to land nodes, y_j0 is 0 or 1 (land just once in each tour):
    // for (std::map<int, std::vector< std::vector<float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
    //     operations_research::MPConstraint *constraint = solver->MakeRowConstraint(1, 1, "");
    //     for (int e_i=0; e_i<edges_.size(); e_i++) {
    //         if (edges_[e_i].type == aerialcore_msgs::Edge::TYPE_NAVIGATION_AND_LANDING && it->first==edges_[e_i].k) {
    //             constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], 1);
    //         }
    //     }
    // }

    // (5) For all tours-drones and pylon nodes, equation of flow consumed in node (flow in equals to flow destroyed in that node plus flow out):
    for (std::map<int, std::vector< std::vector<float> > >::iterator it = battery_drop_matrices_.begin(); it != battery_drop_matrices_.end(); it++) {
        for (int g_i=0; g_i<_graph.size(); g_i++) {
            if (_graph[g_i].type == aerialcore_msgs::GraphNode::TYPE_PYLON) {
                operations_research::MPConstraint *constraint = solver->MakeRowConstraint(0.0, 0.0, "");
                for (int e_i=0; e_i<edges_.size(); e_i++) {
                    if (edges_[e_i].i == from_graph_index_to_matrix_index_[g_i]) {
                        constraint->SetCoefficient(f[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].j ][ edges_[e_i].i ] ], 1);
                        constraint->SetCoefficient(f[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], -1);
                        if (edges_[e_i].type == aerialcore_msgs::Edge::TYPE_INSPECTION) {
                            constraint->SetCoefficient(x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].j ][ edges_[e_i].i ] ], -it->second[ edges_[e_i].j ][ edges_[e_i].i ]);
                        }
                        if ((edges_[e_i].type != aerialcore_msgs::Edge::TYPE_TAKEOFF_AND_NAVIGATION && edges_[e_i].type != aerialcore_msgs::Edge::TYPE_NAVIGATION_AND_LANDING) || it->first==edges_[e_i].k) {
                            constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].j ][ edges_[e_i].i ] ], -it->second[ edges_[e_i].j ][ edges_[e_i].i ]);
                        }
                    }
                }
            }
        }
    }
    // for (std::map<int, std::vector< std::vector<float> > >::iterator it = battery_drop_matrices_.begin(); it != battery_drop_matrices_.end(); it++) {
    //     for (int e_i=0; e_i<edges_.size(); e_i++) {
    //         if (edges_[e_i].type != aerialcore_msgs::Edge::TYPE_TAKEOFF_AND_NAVIGATION) {
    //             operations_research::MPConstraint *constraint = solver->MakeRowConstraint(0.0, 0.0, "");
    //             constraint->SetCoefficient(f[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], 1);
    //             for (int e_j=0; e_j<edges_.size(); e_j++) {
    //                 if (e_j == e_i) continue;
    //                 if (edges_[e_i].j == edges_[e_j].i) {
    //                     constraint->SetCoefficient(f[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_j].i ][ edges_[e_j].j ] ], -1);
    //                     if (edges_[e_j].type == aerialcore_msgs::Edge::TYPE_INSPECTION) {
    //                         constraint->SetCoefficient(x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_j].i ][ edges_[e_j].j ] ], it->second[ edges_[e_j].i ][ edges_[e_j].j ]);
    //                     }
    //                     if ((edges_[e_j].type != aerialcore_msgs::Edge::TYPE_TAKEOFF_AND_NAVIGATION && edges_[e_j].type != aerialcore_msgs::Edge::TYPE_NAVIGATION_AND_LANDING) || it->first==edges_[e_j].k) {
    //                         constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_j].i ][ edges_[e_j].j ] ], it->second[ edges_[e_j].i ][ edges_[e_j].j ]);
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }

    // (6) For all tours-drones, all flow is created in the takeoff nodes:
    for (std::map<int, std::vector< std::vector<float> > >::iterator it = battery_drop_matrices_.begin(); it != battery_drop_matrices_.end(); it++) {
        operations_research::MPConstraint *constraint = solver->MakeRowConstraint(0.0, 0.0, "");
        for (int e_i=0; e_i<edges_.size(); e_i++) {
            if (edges_[e_i].type == aerialcore_msgs::Edge::TYPE_TAKEOFF_AND_NAVIGATION && it->first==edges_[e_i].k) {
                constraint->SetCoefficient(f[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], 1);
            }
            if (edges_[e_i].type == aerialcore_msgs::Edge::TYPE_INSPECTION) {
                constraint->SetCoefficient(x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], -it->second[ edges_[e_i].i ][ edges_[e_i].j ]);
            }
            if ((edges_[e_i].type != aerialcore_msgs::Edge::TYPE_TAKEOFF_AND_NAVIGATION && edges_[e_i].type != aerialcore_msgs::Edge::TYPE_NAVIGATION_AND_LANDING) || it->first==edges_[e_i].k) {
                constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], -it->second[ edges_[e_i].i ][ edges_[e_i].j ]);
            }
        }
    }

    // (7) For all tours-drones, land nodes receive all the remaining flow:
    for (std::map<int, std::vector< std::vector<float> > >::iterator it = battery_drop_matrices_.begin(); it != battery_drop_matrices_.end(); it++) {
        operations_research::MPConstraint *constraint = solver->MakeRowConstraint(0.0, 0.0, "");
        for (int e_i=0; e_i<edges_.size(); e_i++) {
            if (edges_[e_i].type == aerialcore_msgs::Edge::TYPE_NAVIGATION_AND_LANDING && it->first==edges_[e_i].k) {
                constraint->SetCoefficient(f[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], 1);
                constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], -it->second[ edges_[e_i].i ][ edges_[e_i].j ]);
            }
        }
    }

    // (8) For all tours-drones and edges, there is only flow if edge served:
    for (std::map<int, std::vector< std::vector<float> > >::iterator it = battery_drop_matrices_.begin(); it != battery_drop_matrices_.end(); it++) {
        for (int e_i=0; e_i<edges_.size(); e_i++) {
            operations_research::MPConstraint *constraint = solver->MakeRowConstraint(-infinity, 0.0, "");
            constraint->SetCoefficient(f[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], 1);
            if (edges_[e_i].type == aerialcore_msgs::Edge::TYPE_INSPECTION) {
                constraint->SetCoefficient(x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], -1.0);
            }
            if ((edges_[e_i].type != aerialcore_msgs::Edge::TYPE_TAKEOFF_AND_NAVIGATION && edges_[e_i].type != aerialcore_msgs::Edge::TYPE_NAVIGATION_AND_LANDING) || it->first==edges_[e_i].k) {
                constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], -1.0);
            }
        }
    }


    // (1.14) For all tours-drones and edges, flow has a minimum value:
    for (std::map<int, std::vector< std::vector<float> > >::iterator it = battery_drop_matrices_.begin(); it != battery_drop_matrices_.end(); it++) {
        for (int e_i=0; e_i<edges_.size(); e_i++) {
            operations_research::MPConstraint *constraint = solver->MakeRowConstraint(0.0, infinity, "");
            constraint->SetCoefficient(f[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], 1);
            if (edges_[e_i].type == aerialcore_msgs::Edge::TYPE_INSPECTION) {
                constraint->SetCoefficient(x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], -it->second[ edges_[e_i].i ][ edges_[e_i].j ]);
            }
            if ((edges_[e_i].type != aerialcore_msgs::Edge::TYPE_TAKEOFF_AND_NAVIGATION && edges_[e_i].type != aerialcore_msgs::Edge::TYPE_NAVIGATION_AND_LANDING) || it->first==edges_[e_i].k) {
                constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ], -it->second[ edges_[e_i].i ][ edges_[e_i].j ]);
            }
        }
    }

    // (9) and (10) are assumed to be met in the dataset. Also, the robot capacity is assumed to be sufficiently large to service any edge.

    LOG(INFO) << "Number of constraints = " << solver->NumConstraints();


    // SOLVE THE PROBLEM:
    const operations_research::MPSolver::ResultStatus result_status = solver->Solve();

    // Check that the problem has an optimal solution.
    if (result_status != operations_research::MPSolver::OPTIMAL) {
        LOG(FATAL) << "The problem does not have an optimal solution.";
    }
    LOG(INFO) << "Solution:";
    LOG(INFO) << "Optimal objective value = " << objective->Value();

#ifdef DEBUG
    // Print the variables of the solution:
    for (std::map<int, std::vector< std::vector<float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
        std::cout << "x[" << it->first << "]" << std::endl;
        for (int i=0; i<it->second.size(); i++) {
            for (int j=0; j<it->second[i].size(); j++) {
                if (i==0 || j==0) {
                    std::cout << it->second[i][j] << " ";
                    continue;
                }
                bool inspection_edge_found = false;
                for (aerialcore_msgs::Edge& current_edge_struct : edges_) {
                    if (current_edge_struct.type == aerialcore_msgs::Edge::TYPE_INSPECTION && current_edge_struct.i == i && current_edge_struct.j == j) {
                        std::cout << std::setprecision(6) << x[ from_k_i_j_to_x_index_[ it->first ][i][j] ]->solution_value() << " ";
                        inspection_edge_found = true;
                        break;
                    }
                }
                if (!inspection_edge_found) {
                    std::cout << "- ";
                }
            }
            std::cout << std::endl;
        }
    }
    std::map<int, std::vector< std::vector<float> > >::iterator map_begin = time_cost_matrices_.begin();
    for (std::map<int, std::vector< std::vector<float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
        std::cout << "y[" << it->first << "]" << std::endl;
        for (int i=0; i<it->second.size(); i++) {
            for (int j=0; j<it->second[i].size(); j++) {
                if (i==0 || j==0) {
                    std::cout << it->second[i][j] << " ";
                    continue;
                }
                if (time_cost_matrices_[map_begin->first][i][j]==-1) {
                    std::cout << "- ";
                } else {
                    std::cout << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[ it->first ][i][j] ]->solution_value() << " ";
                }
            }
            std::cout << std::endl;
        }
    }
    for (std::map<int, std::vector< std::vector<float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
        std::cout << "f[" << it->first << "]" << std::endl;
        for (int i=0; i<it->second.size(); i++) {
            for (int j=0; j<it->second[i].size(); j++) {
                if (i==0 || j==0) {
                    std::cout << it->second[i][j] << " ";
                    continue;
                }
                if (time_cost_matrices_[map_begin->first][i][j]==-1) {
                    std::cout << "- ";
                } else {
                    std::cout << std::setprecision(6) << f[ from_k_i_j_to_y_and_f_index_[ it->first ][i][j] ]->solution_value() << " ";
                }
            }
            std::cout << std::endl;
        }
    }

    //////////////////// End defining the MILP problem according to agarwal_icra20 and solve it using OR-Tools ////////////////////

    // Print the objective and restrictions for debug:
    std::cout<< std::endl << "Print the objective and restrictions for debug:" << std::endl << std::endl;

    // (1) Minimize all the inspection time (UAVs flying separately).
    std::cout << "(1):" << std::endl;
    for (std::map<int, std::vector< std::vector<float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
        for (int e_i=0; e_i<edges_.size(); e_i++) {
            if (edges_[e_i].type == aerialcore_msgs::Edge::TYPE_INSPECTION) {
                std::cout << "x[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " · " << it->second[ edges_[e_i].i ][ edges_[e_i].j ] << " + ";
                // std::cout << std::endl;
                // std::cout << "e_i = " << e_i << std::endl;
                // std::cout << "k = " << it->first << std::endl;
                // std::cout << "edges_[e_i].i = " << edges_[e_i].i << std::endl;
                // std::cout << "edges_[e_i].j = " << edges_[e_i].j << std::endl;
                // std::cout << "from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] = " << from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] << std::endl;
                // std::cout << "<< std::setprecision(6) << from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ] = " << "x " << x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << std::endl;
            }
            if ( (edges_[e_i].type != aerialcore_msgs::Edge::TYPE_TAKEOFF_AND_NAVIGATION && edges_[e_i].type != aerialcore_msgs::Edge::TYPE_NAVIGATION_AND_LANDING) || it->first==edges_[e_i].k) {
                std::cout << "y[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " · " << it->second[ edges_[e_i].i ][ edges_[e_i].j ] << " + ";
                // std::cout << std::endl;
                // std::cout << "e_i = " << e_i << std::endl;
                // std::cout << "k = " << it->first << std::endl;
                // std::cout << "edges_[e_i].i = " << edges_[e_i].i << std::endl;
                // std::cout << "edges_[e_i].j = " << edges_[e_i].j << std::endl;
                // std::cout << "from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] = " << from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] << std::endl;
                // std::cout << "<< std::setprecision(6) << from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ] = " << "y " << y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << std::endl;
            }
        }
    }
    std::cout << std::endl << std::endl;

    // (2) For all pylon nodes and tours-drones, Sum_all_j(x_ij_k) plus Sum_all_j(y_ij_k) minus Sum_all_j(x_ji_k) and minus Sum_all_j(x_ji_k) is equal to zero (inputs equal to outputs in each node):
    std::cout << "(2):" << std::endl;
    for (std::map<int, std::vector< std::vector<float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
        for (int g_i=0; g_i<_graph.size(); g_i++) {
            if (_graph[g_i].type == aerialcore_msgs::GraphNode::TYPE_PYLON) {
                std::cout << "node_" << g_i << " : ";
                for (int e_i=0; e_i<edges_.size(); e_i++) {
                    if (edges_[e_i].i == from_graph_index_to_matrix_index_[g_i]) {
                        if (edges_[e_i].type == aerialcore_msgs::Edge::TYPE_INSPECTION) {
                            std::cout << "x[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " · " << 1 << " + ";
                            std::cout << "x[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "] " << std::setprecision(6) << x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].j ][ edges_[e_i].i ] ]->solution_value() << " · " << -1 << " + ";
                        }
                        if (edges_[e_i].type != aerialcore_msgs::Edge::TYPE_TAKEOFF_AND_NAVIGATION && edges_[e_i].type != aerialcore_msgs::Edge::TYPE_NAVIGATION_AND_LANDING) {
                            std::cout << "y[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " · " << 1 << " + ";
                            std::cout << "y[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].j ][ edges_[e_i].i ] ]->solution_value() << " · " << -1 << " + ";
                        }
                    }
                }
                std::cout << std::endl;
            } else if (_graph[g_i].type == aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION) {
                std::cout << "node_" << g_i << " : ";
                for (int e_i=0; e_i<edges_.size(); e_i++) {
                    if (edges_[e_i].i == from_graph_index_to_matrix_index_[g_i]) {
                        if (it->first==edges_[e_i].k) {
                            std::cout << "y[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " · " << 1 << " + ";
                        }
                    } else if (edges_[e_i].j == from_graph_index_to_matrix_index_[g_i]) {
                        if (it->first==edges_[e_i].k) {
                            std::cout << "y[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].j ][ edges_[e_i].i ] ]->solution_value() << " · " << -1 << " + ";
                        }
                    }
                }
                std::cout << std::endl;
            }
        }
    }
    std::cout << std::endl;

    // (3) For all inspection edges, x_ij plus x_ji sums 1 (all inspection edges covered in any direction):
    std::cout << "(3):" << std::endl;
    for (int e_i=0; e_i<edges_.size(); e_i++) {
        if (edges_[e_i].type == aerialcore_msgs::Edge::TYPE_INSPECTION && edges_[e_i].i<edges_[e_i].j) {
        std::cout << "inspection_edge_" << e_i << " : ";
            for (std::map<int, std::vector< std::vector<float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
                std::cout << "x[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " · " << 1 << " + ";
                std::cout << "x[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "] " << std::setprecision(6) << x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].j ][ edges_[e_i].i ] ]->solution_value() << " · " << 1 << " + ";
            }
            std::cout << std::endl;
        }
    }
    std::cout << std::endl;

    // (4) For all tours-drones, in edges coming from takeoff nodes, y_0j is 0 or 1 (take off just once in each tour):
    std::cout << "(4):" << std::endl;
    for (std::map<int, std::vector< std::vector<float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
        std::cout << "k_" << it->first << " : ";
        for (int e_i=0; e_i<edges_.size(); e_i++) {
            if (edges_[e_i].type == aerialcore_msgs::Edge::TYPE_TAKEOFF_AND_NAVIGATION && it->first==edges_[e_i].k) {
                std::cout << "y[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " · " << 1 << " + ";
            }
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

    // // (4.1) For all tours-drones, in edges going to land nodes, y_j0 is 0 or 1 (land just once in each tour):
    // std::cout << "(4.1):" << std::endl;
    // for (std::map<int, std::vector< std::vector<float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
    //     std::cout << "k_" << it->first << " : ";
    //     for (int e_i=0; e_i<edges_.size(); e_i++) {
    //         if (edges_[e_i].type == aerialcore_msgs::Edge::TYPE_NAVIGATION_AND_LANDING && it->first==edges_[e_i].k) {
    //             std::cout << "y[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " · " << 1 << " + ";
    //         }
    //     }
    //     std::cout << std::endl;
    // }
    // std::cout << std::endl;

    // (5) For all tours-drones and pylon nodes, equation of flow consumed in node (flow in equals to flow destroyed in that node plus flow out):
    std::cout << "(5):" << std::endl;
    for (std::map<int, std::vector< std::vector<float> > >::iterator it = battery_drop_matrices_.begin(); it != battery_drop_matrices_.end(); it++) {
        for (int g_i=0; g_i<_graph.size(); g_i++) {
            if (_graph[g_i].type == aerialcore_msgs::GraphNode::TYPE_PYLON) {
                std::cout << "pylon_node_" << g_i << " : ";
                for (int e_i=0; e_i<edges_.size(); e_i++) {
                    if (edges_[e_i].i == from_graph_index_to_matrix_index_[g_i]) {
                        std::cout << "f[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "] " << std::setprecision(6) << f[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].j ][ edges_[e_i].i ] ]->solution_value() << " · " << 1 << " + ";
                        std::cout << "f[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << f[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " · " << -1 << " + ";
                        if (edges_[e_i].type == aerialcore_msgs::Edge::TYPE_INSPECTION) {
                            std::cout << "x[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "] " << std::setprecision(6) << x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].j ][ edges_[e_i].i ] ]->solution_value() << " · " << -it->second[ edges_[e_i].j ][ edges_[e_i].i ] << " + ";
                        }
                        if ((edges_[e_i].type != aerialcore_msgs::Edge::TYPE_TAKEOFF_AND_NAVIGATION && edges_[e_i].type != aerialcore_msgs::Edge::TYPE_NAVIGATION_AND_LANDING) || it->first==edges_[e_i].k) {
                            std::cout << "y[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].j ][ edges_[e_i].i ] ]->solution_value() << " · " << -it->second[ edges_[e_i].j ][ edges_[e_i].i ] << " + ";
                        }
                    }
                }
                std::cout << std::endl;
            }
        }
    }
    std::cout << std::endl;

    // (6) For all tours-drones, all flow is created in the takeoff nodes:
    std::cout << "(6):" << std::endl;
    for (std::map<int, std::vector< std::vector<float> > >::iterator it = battery_drop_matrices_.begin(); it != battery_drop_matrices_.end(); it++) {
        std::cout << "k_" << it->first << " : ";
        for (int e_i=0; e_i<edges_.size(); e_i++) {
            if (edges_[e_i].type == aerialcore_msgs::Edge::TYPE_TAKEOFF_AND_NAVIGATION && it->first==edges_[e_i].k) {
                std::cout << "f[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << f[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " · " << 1 << " + ";
            }
            if (edges_[e_i].type == aerialcore_msgs::Edge::TYPE_INSPECTION) {
                std::cout << "x[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " · " << -it->second[ edges_[e_i].i ][ edges_[e_i].j ] << " + ";
            }
            if ((edges_[e_i].type != aerialcore_msgs::Edge::TYPE_TAKEOFF_AND_NAVIGATION && edges_[e_i].type != aerialcore_msgs::Edge::TYPE_NAVIGATION_AND_LANDING) || it->first==edges_[e_i].k) {
                std::cout << "y[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " · " << -it->second[ edges_[e_i].i ][ edges_[e_i].j ] << " + ";
            }
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

    // (7) For all tours-drones, land nodes receive all the remaining flow:
    std::cout << "(7):" << std::endl;
    for (std::map<int, std::vector< std::vector<float> > >::iterator it = battery_drop_matrices_.begin(); it != battery_drop_matrices_.end(); it++) {
        std::cout << "k_" << it->first << " : ";
        for (int e_i=0; e_i<edges_.size(); e_i++) {
            if (edges_[e_i].type == aerialcore_msgs::Edge::TYPE_NAVIGATION_AND_LANDING) {
                std::cout << "f[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << f[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " · " << 1 << " + ";
                std::cout << "y[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " · " << -it->second[ edges_[e_i].i ][ edges_[e_i].j ] << " + ";
            }
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

    // (8) For all tours-drones and edges, there is only flow if edge served:
    std::cout << "(8):" << std::endl;
    for (std::map<int, std::vector< std::vector<float> > >::iterator it = battery_drop_matrices_.begin(); it != battery_drop_matrices_.end(); it++) {
        for (int e_i=0; e_i<edges_.size(); e_i++) {
            std::cout << "edge_" << e_i << " : ";
            std::cout << "f[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << f[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " + ";
            if (edges_[e_i].type == aerialcore_msgs::Edge::TYPE_INSPECTION) {
                std::cout << "x[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << x[ from_k_i_j_to_x_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " + ";
            }
            if ((edges_[e_i].type != aerialcore_msgs::Edge::TYPE_TAKEOFF_AND_NAVIGATION && edges_[e_i].type != aerialcore_msgs::Edge::TYPE_NAVIGATION_AND_LANDING) || it->first==edges_[e_i].k) {
                std::cout << "y[" << it->first << "][" << from_matrix_index_to_graph_index_[edges_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_[e_i].j] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[ it->first ][ edges_[e_i].i ][ edges_[e_i].j ] ]->solution_value() << " + ";
            }
            std::cout << std::endl;
        }
    }
    std::cout << std::endl;


    /* Once printed the results, you can tidy up those terminal results copying them to VS Code and replace the following to just zero 0 (search using Regular Expressions):

     -0 

    -*\d*\.\d*e-\d* 


    And deleting these:

    [fxy]\[\d+?\]\[\d+?\]\[\d+?\] -*0 · -*\d\.*\d* \+ 

    [xyf]\[\d+?\]\[\d+?\]\[\d+?\] 0 · \d*\.\d* \+ 

    [fxy]\[\d+?\]\[\d+?\]\[\d+?\] 0 \+ 

    */

#endif

    // Assign flight_plans_ according to the edges in the solution:
//    for (std::map<int, std::vector< std::vector<float> > >::iterator it = battery_drop_matrices_.begin(); it != battery_drop_matrices_.end(); it++) {

//         // All this commented because the solution isn't totally sorted in f descendent:
//         struct fStruct {
//             float f;
//             int x;
//             int y;
//             int k;
//             int i;
//             int j;
//         };
//         std::vector<fStruct> f_struct_vector;

//         for (int i=1; i<it->second.size(); i++) {
//             for (int j=1; j<it->second[i].size(); j++) {
//                 if (f[ from_k_i_j_to_y_and_f_index_[ it->first ][i][j] ]->solution_value()>0.001) {
//                     fStruct new_f_struct;
//                     new_f_struct.f = f[ from_k_i_j_to_y_and_f_index_[ it->first ][i][j] ]->solution_value();
//                     new_f_struct.x = x[ from_k_i_j_to_x_index_[ it->first ][i][j] ]->solution_value();
//                     new_f_struct.y = y[ from_k_i_j_to_y_and_f_index_[ it->first ][i][j] ]->solution_value();
//                     new_f_struct.k = k;
//                     new_f_struct.i = from_matrix_index_to_graph_index_[i];
//                     new_f_struct.j = from_matrix_index_to_graph_index_[j];
//                     f_struct_vector.push_back(new_f_struct);
//                 }
//             }
//         }

//         std::sort(f_struct_vector.begin(), f_struct_vector.end(), [](const fStruct& a, const fStruct& b) { 
//             return a.f > b.f; 
//         }); // Sort in descending order.

//         aerialcore_msgs::FlightPlan current_flight_plan;

//         for (const auto& f_struct : f_struct_vector) {
//             current_flight_plan.nodes.push_back(f_struct.i);
//         }
//         if (f_struct_vector.size()>0) {
//             current_flight_plan.uav_id = _graph[ f_struct_vector.front().i ].id;
//             current_flight_plan.nodes.push_back( f_struct_vector.back().j );
//         }

//         if (current_flight_plan.nodes.size()>3) {   // Consider the flight plan only if it visits two or more pylons.
// #ifdef DEBUG
//             std::cout << "current_flight_plan.uav_id: " << current_flight_plan.uav_id << std::endl;
//             std::cout << "current_flight_plan.nodes:" << std::endl;

//             for (int node : current_flight_plan.nodes) {
//                 std::cout << node << std::endl;
//             }

//             bool milp_solution_coherent = f_struct_vector.front().i == f_struct_vector.back().j ? true : false;
//             for (int index = 0; index<f_struct_vector.size()-1; index++) {
//                 if (milp_solution_coherent) {
//                     milp_solution_coherent = f_struct_vector[index].j == f_struct_vector[index + 1].i ? true : false;
//                 } else if (!milp_solution_coherent) {
//                     break;
//                 }
//             }
//             std::cout << "milp_solution_coherent = " << milp_solution_coherent << std::endl;
// #endif




    //         std::vector< std::pair<int,int> > solution_tree_raw;    // First is the father node, second the current node (son).
    //         std::vector<int> open_nodes;
    //         std::vector<int> solution_nodes;

    //         for (int e_i=0; e_i<edges_.size(); e_i++) {
    //             if (edges_[e_i].type == aerialcore_msgs::Edge::TYPE_TAKEOFF_AND_NAVIGATION && f[ from_k_i_j_to_y_and_f_index_[ it->first ][i][j] ]->solution_value()>0.001) {
    //                 std::pair<int,int> current_solution_tree_raw;
    //                 current_solution_tree_raw.first = -1;                   // Takeoff node without father node.
    //                 current_solution_tree_raw.second = edges_[e_i].i;
    //                 solution_tree_raw.push_back(current_solution_tree_raw);

    //                 open_nodes.push_back(edges_[e_i].i);
    //                 break;
    //             }
    //         }

    //         while (open_nodes.size() > 0) {
    //             int current_open_node = open_nodes[0];
    //             open_nodes.erase(open_nodes.begin());

    //             for (int e_i=0; e_i<edges_.size(); e_i++) {
    //                 if (edges_[e_i].j == current_open_node) {
    //                     std::pair<int,int> current_solution_tree_raw;
    //                     current_solution_tree_raw.first  = edges_[e_i].i;
    //                     current_solution_tree_raw.second = edges_[e_i].j;
    //                     solution_tree_raw.push_back(current_solution_tree_raw);

    //                     current_open_nodes.push_back(edges_[e_i].j);
    //                 }
    //             }
    //         }

    //         for (int i=0; i<solution_tree_raw.size(); i++) {
    //             int current_node = solution_tree_raw[i].second;
    //             solution_nodes.push_back(current_node);
    //             for (int j=current_node+1; j<solution_tree_raw.size(); j++) {
    //                 if (solution_tree_raw[i].second == solution_tree_raw[j].second) {
    //                 }
    //             }
    //         }

    //         aerialcore_msgs::FlightPlan current_flight_plan;

    //         for (int current_node : solution_nodes) {
    //             current_flight_plan.nodes.push_back(current_node);
    //         }
    //         if (current_flight_plan.nodes.size()>0) {
    //             current_flight_plan.uav_id = _graph[ current_flight_plan.nodes.front().i ].id;
    //         }

    //         flight_plans_.push_back(current_flight_plan);
    //     }
    // }

    // Calculate the path free of obstacles between nodes outside in the Mission Controller. There is a path warantied between the nodes.

    fillFlightPlansFields(flight_plans_);

    return flight_plans_;

} // end getPlanMILP method


void CentralizedPlanner::constructUAVs(const std::vector< std::tuple<float, float, int, int, int, int, int, int, bool, bool> >& _drone_info) {

    UAVs_.clear();

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
} // end constructUAVs


void CentralizedPlanner::constructEdges(std::vector<aerialcore_msgs::GraphNode>& _graph) {

    edges_.clear();
    from_graph_index_to_matrix_index_.clear();
    from_matrix_index_to_graph_index_.clear();
    from_k_i_j_to_x_index_.clear();
    from_k_i_j_to_y_and_f_index_.clear();

    // Construct edges_ that contain all the possible edges, with information of if it is a pure dead-heading edge or both inspection and dead-heading edge:
    int x_index_size_ = 0;
    int y_and_f_index_size_ = 0;
    std::map<int, std::vector< std::vector<float> > >::iterator map_begin = time_cost_matrices_.begin();
    for (int i=1; i<time_cost_matrices_[map_begin->first].size(); i++) {
        from_graph_index_to_matrix_index_[ (int) time_cost_matrices_[map_begin->first][i][0] ] = i;
        from_matrix_index_to_graph_index_[i] = (int) time_cost_matrices_[map_begin->first][i][0];
        for (int j=1; j<time_cost_matrices_[map_begin->first][i].size(); j++) {
            if (time_cost_matrices_[map_begin->first][i][j]==-1) {
                continue;
            } else {
                aerialcore_msgs::Edge new_edge;
                new_edge.i = i;
                new_edge.j = j;
                new_edge.k = -1;
                edges_.push_back(new_edge);
            }
        }
    }
    for (int g_i=0; g_i<_graph.size(); g_i++) {
        if (_graph[g_i].type == aerialcore_msgs::GraphNode::TYPE_PYLON) {
            for (int j=0; j<_graph[g_i].connections_indexes.size(); j++) {
                for (aerialcore_msgs::Edge& current_edge_struct : edges_) {
                    if (current_edge_struct.i == from_graph_index_to_matrix_index_[g_i] && current_edge_struct.j == from_graph_index_to_matrix_index_[ _graph[g_i].connections_indexes[j] ]) {
                        current_edge_struct.type = aerialcore_msgs::Edge::TYPE_INSPECTION;

                        for (std::map<int, std::vector< std::vector<float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
                            from_k_i_j_to_x_index_[ it->first ][current_edge_struct.i][current_edge_struct.j] = x_index_size_++;
                        }
                    }
                }
            }
        }
    }
    for (aerialcore_msgs::Edge& current_edge_struct : edges_) {
        if (_graph[ from_matrix_index_to_graph_index_[current_edge_struct.i] ].type==aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION && _graph[ from_matrix_index_to_graph_index_[current_edge_struct.j] ].type==aerialcore_msgs::GraphNode::TYPE_PYLON ) {
            current_edge_struct.type = aerialcore_msgs::Edge::TYPE_TAKEOFF_AND_NAVIGATION;

            current_edge_struct.k = findUavIndexById( _graph[ from_matrix_index_to_graph_index_[current_edge_struct.i] ].id );

        } else if ( _graph[ from_matrix_index_to_graph_index_[current_edge_struct.i] ].type==aerialcore_msgs::GraphNode::TYPE_PYLON && ( _graph[ from_matrix_index_to_graph_index_[current_edge_struct.j] ].type==aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION/*TYPE_REGULAR_LAND_STATION || _graph[ from_matrix_index_to_graph_index_[current_edge_struct.j] ].type==aerialcore_msgs::GraphNode::TYPE_RECHARGE_LAND_STATION*/ ) ) {
            current_edge_struct.type = aerialcore_msgs::Edge::TYPE_NAVIGATION_AND_LANDING;

            current_edge_struct.k = findUavIndexById( _graph[ from_matrix_index_to_graph_index_[current_edge_struct.j] ].id );
        }

        if (current_edge_struct.type == aerialcore_msgs::Edge::TYPE_TAKEOFF_AND_NAVIGATION || current_edge_struct.type == aerialcore_msgs::Edge::TYPE_NAVIGATION_AND_LANDING) {
            from_k_i_j_to_y_and_f_index_[current_edge_struct.k][current_edge_struct.i][current_edge_struct.j] = y_and_f_index_size_++;
        } else {
            for (std::map<int, std::vector< std::vector<float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
                from_k_i_j_to_y_and_f_index_[ it->first ][current_edge_struct.i][current_edge_struct.j] = y_and_f_index_size_++;
            }
        }

    }
#ifdef DEBUG
    std::cout << "Number of edges            = " << edges_.size() << std::endl;
    std::cout << "Number of inspection edges = " << x_index_size_/time_cost_matrices_.size() << std::endl;
    std::cout << "Number of k                = " << time_cost_matrices_.size() << std::endl;
    std::cout << "Number of i and j          = " << time_cost_matrices_[map_begin->first].size()-1 << std::endl;
    int pylons_counter = 0;
    for (int g_i=0; g_i<_graph.size(); g_i++) {
        if (_graph[g_i].type == aerialcore_msgs::GraphNode::TYPE_PYLON) {
            pylons_counter++;
        }
    }
    std::cout << "Number of pylons           = " << pylons_counter << std::endl;
    for (int i=0; i<edges_.size(); i++) {
        std::cout << "edges_[" << i << "].i         = " << edges_[i].i << std::endl;
        std::cout << "edges_[" << i << "].j         = " << edges_[i].j << std::endl;
        if (edges_[i].type == aerialcore_msgs::Edge::TYPE_INSPECTION) {
            std::cout << "edges_[" << i << "].type = INSPECTION" << std::endl;
        } else if (edges_[i].type == aerialcore_msgs::Edge::TYPE_NAVIGATION) {
            std::cout << "edges_[" << i << "].type = NAVIGATION" << std::endl;
        } else if (edges_[i].type == aerialcore_msgs::Edge::TYPE_TAKEOFF_AND_NAVIGATION) {
            std::cout << "edges_[" << i << "].type = TAKEOFF_AND_NAVIGATION" << std::endl;
        } else if (edges_[i].type == aerialcore_msgs::Edge::TYPE_NAVIGATION_AND_LANDING) {
            std::cout << "edges_[" << i << "].type = NAVIGATION_AND_LANDING" << std::endl;
        }
    }
    for ( auto it = from_graph_index_to_matrix_index_.begin(); it != from_graph_index_to_matrix_index_.end(); it++ ) {
        std::cout << "from_graph_index_to_matrix_index_[" << it->first << "] = " << it->second << std::endl;
    }
    for ( auto it = from_matrix_index_to_graph_index_.begin(); it != from_matrix_index_to_graph_index_.end(); it++ ) {
        std::cout << "from_matrix_index_to_graph_index_[" << it->first << "] = " << it->second << std::endl;
    }
#endif
} // end constructEdges


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


std::vector<aerialcore_msgs::FlightPlan> CentralizedPlanner::getPlanHeuristic(std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector< std::tuple<float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::vector< geometry_msgs::Polygon >& _no_fly_zones, const geometry_msgs::Polygon& _geofence, const std::map<int, std::vector< std::vector<float> > >& _time_cost_matrices, const std::map<int, std::vector< std::vector<float> > >& _battery_drop_matrices) {

    // Get the initial plan with the greedy method:
    getPlanGreedy(_graph, _drone_info, _no_fly_zones, _geofence, _time_cost_matrices, _battery_drop_matrices);
    std::vector<aerialcore_msgs::FlightPlan> greedy_flight_plans = flight_plans_;
    flight_plans_.clear();

    // TODO:
    // 1) Agarwal: Merge-Embed-Merge algorithm
    // 1) VNS CTU
    // 3) Dubins for navigations with VTOL or fixed wing, may be needed navigations between non-parallel inspection edges.
    // 4) Compare with similar problems (maybe ask for the code or just do it myself)
    // 5) Online
    // 6) Wind adapt
    // 7) Problem with paths in the cost matrix (wind doesn't affect the same with paths)

    fillFlightPlansFields(flight_plans_);

    return flight_plans_;

} // end getPlanHeuristic


float CentralizedPlanner::solutionTimeCost(std::vector<aerialcore_msgs::FlightPlan> _flight_plans) {
    float solution_time_cost = 0;

    for (int i=0; i<_flight_plans.size(); i++) {
        for (int j=0; j<_flight_plans[i].nodes.size()-1; j++) {
            solution_time_cost += time_cost_matrices_[i][ from_graph_index_to_matrix_index_[_flight_plans[i].nodes[j]] ][ from_graph_index_to_matrix_index_[_flight_plans[i].nodes[j+1]] ];
        }
    }

    return solution_time_cost;
} // end solutionTimeCost


bool CentralizedPlanner::solutionBatteryDropValidOrNot(std::vector<aerialcore_msgs::FlightPlan> _flight_plans) {
    bool solution_battery_drop_valid_or_not = true;

    for (int i=0; i<_flight_plans.size(); i++) {
        float battery_left = UAVs_[i].initial_battery;
        for (int j=0; j<_flight_plans[i].nodes.size()-1; j++) {
            battery_left -= battery_drop_matrices_[i][ from_graph_index_to_matrix_index_[_flight_plans[i].nodes[j]] ][ from_graph_index_to_matrix_index_[_flight_plans[i].nodes[j+1]] ];
        }
        if (battery_left < UAVs_[i].minimum_battery) {
            solution_battery_drop_valid_or_not = false;
            break;
        }
    }

    return solution_battery_drop_valid_or_not;
} // end solutionBatteryDropValidOrNot


bool CentralizedPlanner::solutionValidOrNot(std::vector<aerialcore_msgs::FlightPlan> _flight_plans) {

    // Check that there aren't two navigations together:
    for (int i=0; i<_flight_plans.size(); i++) {
        bool previous_is_navigation = false;
        bool current_is_navigation;
        for (int j=0; j<_flight_plans[i].edges.size(); j++) {
            current_is_navigation = edges_[ _flight_plans[i].edges[j] ].type != aerialcore_msgs::Edge::TYPE_INSPECTION ? true : false ;
            if (previous_is_navigation && current_is_navigation) {
                return false;
            }
            previous_is_navigation = current_is_navigation;
        }
    }

    // Solution battery drop is acceptable or not:
    if (!solutionBatteryDropValidOrNot(_flight_plans)) {
        return false;
    }

    return true;
} // end solutionValidOrNot


void CentralizedPlanner::fillFlightPlansFields(std::vector<aerialcore_msgs::FlightPlan>& _flight_plans) {

    // Fill the edges:
    for (int i=0; i<_flight_plans.size(); i++) {
        _flight_plans[i].edges.clear();
        for (int j=0; j<_flight_plans[i].nodes.size()-1; j++) {
            for (int k=0; k<edges_.size(); k++) {
                if (_flight_plans[i].nodes[j]==edges_[k].i && _flight_plans[i].nodes[j+1]==edges_[k].j) {
                    _flight_plans[i].edges.push_back(k);
                    break;
                }
            }
        }
    }

    // Fill the poses and their type:
    // Postprocess to calculate the path free of obstacles between nodes:
    for (int i=0; i<_flight_plans.size(); i++) {

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
                    _flight_plans[i].type.push_back(aerialcore_msgs::FlightPlan::TYPE_PASS_NODE_WP);
                    previous_iteration_landing = false;
                } else {
                    _flight_plans[i].type.push_back(aerialcore_msgs::FlightPlan::TYPE_TAKEOFF_WP);
                    previous_iteration_landing = false;
                }
            } else {        // Any other node different from the first one:
                if (graph_[ _flight_plans[i].nodes[j] ].type==aerialcore_msgs::GraphNode::TYPE_RECHARGE_LAND_STATION || graph_[ _flight_plans[i].nodes[j] ].type==aerialcore_msgs::GraphNode::TYPE_REGULAR_LAND_STATION) {
                    if (previous_iteration_landing) {
                        _flight_plans[i].type.push_back(aerialcore_msgs::FlightPlan::TYPE_TAKEOFF_WP);
                        previous_iteration_landing = false;
                    } else {
                        _flight_plans[i].type.push_back(aerialcore_msgs::FlightPlan::TYPE_LAND_WP);
                        previous_iteration_landing = true;
                    }
                } else if (graph_[ _flight_plans[i].nodes[j] ].type==aerialcore_msgs::GraphNode::TYPE_PYLON) {
                    _flight_plans[i].type.push_back(aerialcore_msgs::FlightPlan::TYPE_PASS_NODE_WP);
                    previous_iteration_landing = false;
                }
            }

            if (!path_planner_.checkIfTwoPointsAreVisible(test_point_1, test_point_2)) {
                auto path = path_planner_.getPathWithRelativeAltitude(test_point_1, test_point_2, map_origin_geo_.altitude);

                for (int k=0; k<path.size()-1; k++) {
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


} // end namespace aerialcore
