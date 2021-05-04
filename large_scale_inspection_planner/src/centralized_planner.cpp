/**
 * AERIALCORE Project:
 *
 * Centralized planner.
 * 
 */

#include <centralized_planner.h>

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
CentralizedPlanner::CentralizedPlanner() : path_planner_() {}


// Brief Destructor
CentralizedPlanner::~CentralizedPlanner() {}


std::vector<aerialcore_msgs::FlightPlan> CentralizedPlanner::getPlanGreedy(std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector< std::tuple<float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::vector< geometry_msgs::Polygon >& _no_fly_zones, const geometry_msgs::Polygon& _geofence) {

    graph_.clear();
    edges_.clear();
    UAVs_.clear();
    flight_plans_.clear();

    graph_ = _graph;

    if (_geofence.points.size()>0 && _no_fly_zones.size()>0) {
        path_planner_ = grvc::PathPlanner(_no_fly_zones, _geofence);
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
            flight_plans_.push_back(current_flight_plan);
        }
    }

    // Postprocess to calculate the path free of obstacles between nodes:
    // Up until now the flight plan didn't store the intermediate points to avoid flying above no-fly zones. Run the path planning between pair of nodes to calculate and store the waypoints of those paths in the solution:
    // It should be a faster way of calculating this while the algorithm is running, but a lot should be changed to do that right now, easier to do it like this.
    for (int i=0; i<flight_plans_.size(); i++) {
        for (int j=0; j<flight_plans_[i].nodes.size()-1; j++) {
            geometry_msgs::Point32 test_point_1, test_point_2;
            test_point_1.x = _graph[ flight_plans_[i].nodes[j] ].x;       test_point_2.x = _graph[ flight_plans_[i].nodes[j+1] ].x;
            test_point_1.y = _graph[ flight_plans_[i].nodes[j] ].y;       test_point_2.y = _graph[ flight_plans_[i].nodes[j+1] ].y;
            test_point_1.z = _graph[ flight_plans_[i].nodes[j] ].z;       test_point_2.z = _graph[ flight_plans_[i].nodes[j+1] ].z;
            if (!path_planner_.checkIfTwoPointsAreVisible(test_point_1, test_point_2)) {
                auto path = path_planner_.getPath(test_point_1, test_point_2);

                for (int k=0; k<path.size()-1; k++) {
                    aerialcore_msgs::GraphNode graph_node;
                    graph_node.type = aerialcore_msgs::GraphNode::TYPE_PASS_WP_AVOIDING_NO_FLY_ZONE;
                    graph_node.x = path[k].x;
                    graph_node.y = path[k].y;
                    graph_node.z = path[k].z;
                    // graph_node.latitude = // TODO, cartesian to geographic
                    // graph_node.longitude = // TODO, cartesian to geographic
                    // graph_node.altitude = // TODO, correct environment altitude
                    _graph.push_back(graph_node);

                    int last_inserted_node_index = _graph.size()-1;
                    flight_plans_[i].nodes.insert(flight_plans_[i].nodes.begin()+j+1+k, last_inserted_node_index);
                }

            }
        }
    }

    return flight_plans_;

} // end getPlanGreedy method


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
    for (int i=0; i<flight_plans_.size(); i++) {
        std::cout << "flight_plans[ " << i << " ].id = " << flight_plans_[i].uav_id << std::endl;
        for (int j=0; j<flight_plans_[i].nodes.size(); j++) {
            std::cout << "flight_plans[ " << i << " ].nodes[ " << j << " ] = " << flight_plans_[i].nodes[j] << std::endl;
        }
    }
    std::cout << std::endl;
} // end printPlan method


std::vector<aerialcore_msgs::FlightPlan> CentralizedPlanner::getPlanMILP(std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector< std::tuple<float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::vector< geometry_msgs::Polygon >& _no_fly_zones, const geometry_msgs::Polygon& _geofence, const std::vector< std::vector< std::vector<float> > >& _time_cost_matrices, const std::vector< std::vector< std::vector<float> > >& _battery_drop_matrices) {

    UAVs_.clear();
    edges_MILP_.clear();
    from_graph_index_to_matrix_index_.clear();
    from_matrix_index_to_graph_index_.clear();
    from_k_i_j_to_x_index_.clear();
    from_k_i_j_to_y_and_f_index_.clear();
    flight_plans_.clear();

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

    // Construct edges_MILP_ that contain all the possible edges, with information of if it is a pure dead-heading edge or both inspection and dead-heading edge:
    int x_index = 0;
    int y_and_f_index = 0;
    for (int i=1; i<_time_cost_matrices[0].size(); i++) {
        from_graph_index_to_matrix_index_[ (int) _time_cost_matrices[0][i][0] ] = i;
        from_matrix_index_to_graph_index_[i] = (int) _time_cost_matrices[0][i][0];
        for (int j=1; j<_time_cost_matrices[0][i].size(); j++) {
            if (_time_cost_matrices[0][i][j]==-1) {
                continue;
            } else {
                Edge new_edge;
                new_edge.i = i;
                new_edge.j = j;
                edges_MILP_.push_back(new_edge);
            }
        }
    }
    for (int g_i=0; g_i<_graph.size(); g_i++) {
        if (_graph[g_i].type == aerialcore_msgs::GraphNode::TYPE_PYLON) {
            for (int j=0; j<_graph[g_i].connections_indexes.size(); j++) {
                for (Edge& current_edge_struct : edges_MILP_) {
                    if (current_edge_struct.i == from_graph_index_to_matrix_index_[g_i] && current_edge_struct.j == from_graph_index_to_matrix_index_[ _graph[g_i].connections_indexes[j] ]) {
                        current_edge_struct.edge_type = EdgeType::INSPECTION;

                        for (int k=0; k<_time_cost_matrices.size(); k++) {
                            from_k_i_j_to_x_index_[k][current_edge_struct.i][current_edge_struct.j] = x_index++;
                        }
                    }
                }
            }
        }
    }
    for (Edge& current_edge_struct : edges_MILP_) {
        if (_graph[ from_matrix_index_to_graph_index_[current_edge_struct.i] ].type==aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION && _graph[ from_matrix_index_to_graph_index_[current_edge_struct.j] ].type==aerialcore_msgs::GraphNode::TYPE_PYLON ) {
            current_edge_struct.edge_type = EdgeType::TAKEOFF_AND_NAVIGATION;

            current_edge_struct.k = findUavIndexById( _graph[ from_matrix_index_to_graph_index_[current_edge_struct.i] ].id );

        } else if ( _graph[ from_matrix_index_to_graph_index_[current_edge_struct.i] ].type==aerialcore_msgs::GraphNode::TYPE_PYLON && ( _graph[ from_matrix_index_to_graph_index_[current_edge_struct.j] ].type==aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION/*TYPE_REGULAR_LAND_STATION || _graph[ from_matrix_index_to_graph_index_[current_edge_struct.j] ].type==aerialcore_msgs::GraphNode::TYPE_RECHARGE_LAND_STATION*/ ) ) {
            current_edge_struct.edge_type = EdgeType::NAVIGATION_AND_LANDING;

            current_edge_struct.k = findUavIndexById( _graph[ from_matrix_index_to_graph_index_[current_edge_struct.j] ].id );
        }

        if (current_edge_struct.edge_type == EdgeType::TAKEOFF_AND_NAVIGATION || current_edge_struct.edge_type == EdgeType::NAVIGATION_AND_LANDING) {
            from_k_i_j_to_y_and_f_index_[current_edge_struct.k][current_edge_struct.i][current_edge_struct.j] = y_and_f_index++;
        } else {
            for (int k=0; k<_time_cost_matrices.size(); k++) {
                from_k_i_j_to_y_and_f_index_[k][current_edge_struct.i][current_edge_struct.j] = y_and_f_index++;
            }
        }

    }
    std::cout << "Number of edges            = " << edges_MILP_.size() << std::endl;
    std::cout << "Number of inspection edges = " << x_index/_time_cost_matrices.size() << std::endl;
    std::cout << "Number of k                = " << _time_cost_matrices.size() << std::endl;
    std::cout << "Number of i and j          = " << _time_cost_matrices[0].size()-1 << std::endl;
    int pylons_counter = 0;
    for (int g_i=0; g_i<_graph.size(); g_i++) {
        if (_graph[g_i].type == aerialcore_msgs::GraphNode::TYPE_PYLON) {
            pylons_counter++;
        }
    }
    std::cout << "Number of pylons           = " << pylons_counter << std::endl;
    for (int i=0; i<edges_MILP_.size(); i++) {
        std::cout << "edges_MILP_[" << i << "].i         = " << edges_MILP_[i].i << std::endl;
        std::cout << "edges_MILP_[" << i << "].j         = " << edges_MILP_[i].j << std::endl;
        if (edges_MILP_[i].edge_type == EdgeType::INSPECTION) {
            std::cout << "edges_MILP_[" << i << "].edge_type = INSPECTION" << std::endl;
        } else if (edges_MILP_[i].edge_type == EdgeType::NAVIGATION) {
            std::cout << "edges_MILP_[" << i << "].edge_type = NAVIGATION" << std::endl;
        } else if (edges_MILP_[i].edge_type == EdgeType::TAKEOFF_AND_NAVIGATION) {
            std::cout << "edges_MILP_[" << i << "].edge_type = TAKEOFF_AND_NAVIGATION" << std::endl;
        } else if (edges_MILP_[i].edge_type == EdgeType::NAVIGATION_AND_LANDING) {
            std::cout << "edges_MILP_[" << i << "].edge_type = NAVIGATION_AND_LANDING" << std::endl;
        }
    }
    for ( auto it = from_graph_index_to_matrix_index_.begin(); it != from_graph_index_to_matrix_index_.end(); it++ ) {
        std::cout << "from_graph_index_to_matrix_index_[" << it->first << "] = " << it->second << std::endl;
    }
    for ( auto it = from_matrix_index_to_graph_index_.begin(); it != from_matrix_index_to_graph_index_.end(); it++ ) {
        std::cout << "from_matrix_index_to_graph_index_[" << it->first << "] = " << it->second << std::endl;
    }


    //////////////////// Start defining the MILP problem according to agarwal_icra20 and solve it using OR-Tools ////////////////////

    // Create the mip solver with the SCIP backend:
    std::unique_ptr<operations_research::MPSolver> solver(operations_research::MPSolver::CreateSolver("SCIP"));

    const double infinity = solver->infinity();

    // Variables:
    // x[x_index]       is a vector of binary variables.
    // y[y_and_f_index] is a vector of non-negative integer variables.
    // f[y_and_f_index] is a vector of non-negative continuous variables.
    std::vector<operations_research::MPVariable *> x;
    std::vector<operations_research::MPVariable *> y;
    std::vector<operations_research::MPVariable *> f;
    for (int index=0; index<x_index; index++) {
        x.push_back( solver->MakeBoolVar("") );
    }
    for (int index=0; index<y_and_f_index; index++) {
        y.push_back( solver->MakeIntVar(0, infinity, "") );
        f.push_back( solver->MakeNumVar(0.0, 1.0, "") );
    }
    LOG(INFO) << "Number of variables = " << solver->NumVariables();

    // Create the objective function:
    // (1) Minimize all the inspection time (UAVs flying separately).
    operations_research::MPObjective *const objective = solver->MutableObjective();
    for (int k=0; k<_time_cost_matrices.size(); k++) {
        for (int e_i=0; e_i<edges_MILP_.size(); e_i++) {
            if (edges_MILP_[e_i].edge_type == EdgeType::INSPECTION) {
                objective->SetCoefficient(x[ from_k_i_j_to_x_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ], _time_cost_matrices[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ]);
            }
            if ( (edges_MILP_[e_i].edge_type != EdgeType::TAKEOFF_AND_NAVIGATION && edges_MILP_[e_i].edge_type != EdgeType::NAVIGATION_AND_LANDING) || k==edges_MILP_[e_i].k) {
                objective->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ], _time_cost_matrices[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ]);
            }
        }
    }
    objective->SetMinimization();


    // Create the constraints:

    // (2) For all pylon nodes and tours-drones, Sum_all_j(x_ij_k) plus Sum_all_j(y_ij_k) minus Sum_all_j(x_ji_k) and minus Sum_all_j(x_ji_k) is equal to zero (inputs equal to outputs in each node):
    for (int k=0; k<_time_cost_matrices.size(); k++) {
        for (int g_i=0; g_i<_graph.size(); g_i++) {
            if (_graph[g_i].type == aerialcore_msgs::GraphNode::TYPE_PYLON) {
                operations_research::MPConstraint *constraint = solver->MakeRowConstraint(0, 0, "");
                for (int e_i=0; e_i<edges_MILP_.size(); e_i++) {
                    if (edges_MILP_[e_i].i == from_graph_index_to_matrix_index_[g_i]) {
                        if (edges_MILP_[e_i].edge_type == EdgeType::INSPECTION) {
                            constraint->SetCoefficient(x[ from_k_i_j_to_x_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ], 1);
                            constraint->SetCoefficient(x[ from_k_i_j_to_x_index_[k][ edges_MILP_[e_i].j ][ edges_MILP_[e_i].i ] ], -1);
                        }
                        if (edges_MILP_[e_i].edge_type != EdgeType::TAKEOFF_AND_NAVIGATION && edges_MILP_[e_i].edge_type != EdgeType::NAVIGATION_AND_LANDING) {
                            constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ], 1);
                            constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].j ][ edges_MILP_[e_i].i ] ], -1);
                        }
                    }
                }
            } else if (_graph[g_i].type == aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION) {
                operations_research::MPConstraint *constraint = solver->MakeRowConstraint(0, 0, "");
                for (int e_i=0; e_i<edges_MILP_.size(); e_i++) {
                    if (edges_MILP_[e_i].i == from_graph_index_to_matrix_index_[g_i]) {
                        if (k==edges_MILP_[e_i].k) {
                            constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ], 1);
                            constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].j ][ edges_MILP_[e_i].i ] ], -1);
                        }
                    }
                }
            }
        }
    }

    // (3) For all inspection edges, x_ij plus x_ji sums 1 (all inspection edges covered in any direction):
    for (int e_i=0; e_i<edges_MILP_.size(); e_i++) {
        if (edges_MILP_[e_i].edge_type == EdgeType::INSPECTION && edges_MILP_[e_i].i<edges_MILP_[e_i].j) {
            operations_research::MPConstraint *constraint = solver->MakeRowConstraint(1, 1, "");
            for (int k=0; k<_time_cost_matrices.size(); k++) {
                constraint->SetCoefficient(x[ from_k_i_j_to_x_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ], 1);
                constraint->SetCoefficient(x[ from_k_i_j_to_x_index_[k][ edges_MILP_[e_i].j ][ edges_MILP_[e_i].i ] ], 1);
            }
        }
    }

    // (4) For all tours-drones, in edges coming from takeoff nodes, y_0j is 0 or 1 (take off just once or zero times in each tour):
    for (int k=0; k<_time_cost_matrices.size(); k++) {
        operations_research::MPConstraint *constraint = solver->MakeRowConstraint(0, 1, "");
        for (int e_i=0; e_i<edges_MILP_.size(); e_i++) {
            if (edges_MILP_[e_i].edge_type == EdgeType::TAKEOFF_AND_NAVIGATION && k==edges_MILP_[e_i].k) {
                constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ], 1);
            }
        }
    }

    // // (4.1) For all tours-drones, in edges going to land nodes, y_j0 is 0 or 1 (land just once in each tour):
    // for (int k=0; k<_time_cost_matrices.size(); k++) {
    //     operations_research::MPConstraint *constraint = solver->MakeRowConstraint(1, 1, "");
    //     for (int e_i=0; e_i<edges_MILP_.size(); e_i++) {
    //         if (edges_MILP_[e_i].edge_type == EdgeType::NAVIGATION_AND_LANDING && k==edges_MILP_[e_i].k) {
    //             constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ], 1);
    //         }
    //     }
    // }

    // (5) For all tours-drones and pylon nodes, equation of flow consumed in node (flow in equals to flow destroyed in that node plus flow out):
    for (int k=0; k<_battery_drop_matrices.size(); k++) {
        for (int g_i=0; g_i<_graph.size(); g_i++) {
            if (_graph[g_i].type == aerialcore_msgs::GraphNode::TYPE_PYLON) {
                operations_research::MPConstraint *constraint = solver->MakeRowConstraint(0.0, 0.0, "");
                for (int e_i=0; e_i<edges_MILP_.size(); e_i++) {
                    if (edges_MILP_[e_i].i == from_graph_index_to_matrix_index_[g_i]) {
                        constraint->SetCoefficient(f[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].j ][ edges_MILP_[e_i].i ] ], 1);
                        constraint->SetCoefficient(f[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ], -1);
                        if (edges_MILP_[e_i].edge_type == EdgeType::INSPECTION) {
                            constraint->SetCoefficient(x[ from_k_i_j_to_x_index_[k][ edges_MILP_[e_i].j ][ edges_MILP_[e_i].i ] ], -_battery_drop_matrices[k][ edges_MILP_[e_i].j ][ edges_MILP_[e_i].i ]);
                        }
                        if ((edges_MILP_[e_i].edge_type != EdgeType::TAKEOFF_AND_NAVIGATION && edges_MILP_[e_i].edge_type != EdgeType::NAVIGATION_AND_LANDING) || k==edges_MILP_[e_i].k) {
                            constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].j ][ edges_MILP_[e_i].i ] ], -_battery_drop_matrices[k][ edges_MILP_[e_i].j ][ edges_MILP_[e_i].i ]);
                        }
                    }
                }
            }
        }
    }

    // (6) For all tours-drones, all flow is created in the takeoff nodes:
    for (int k=0; k<_battery_drop_matrices.size(); k++) {
        operations_research::MPConstraint *constraint = solver->MakeRowConstraint(0.0, 0.0, "");
        for (int e_i=0; e_i<edges_MILP_.size(); e_i++) {
            if (edges_MILP_[e_i].edge_type == EdgeType::TAKEOFF_AND_NAVIGATION && k==edges_MILP_[e_i].k) {
                constraint->SetCoefficient(f[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ], 1);
            }
            if (edges_MILP_[e_i].edge_type == EdgeType::INSPECTION) {
                constraint->SetCoefficient(x[ from_k_i_j_to_x_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ], -_battery_drop_matrices[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ]);
            }
            if ((edges_MILP_[e_i].edge_type != EdgeType::TAKEOFF_AND_NAVIGATION && edges_MILP_[e_i].edge_type != EdgeType::NAVIGATION_AND_LANDING) || k==edges_MILP_[e_i].k) {
                constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ], -_battery_drop_matrices[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ]);
            }
        }
    }

    // (7) For all tours-drones, land nodes receive all the remaining flow:
    for (int k=0; k<_battery_drop_matrices.size(); k++) {
        operations_research::MPConstraint *constraint = solver->MakeRowConstraint(0.0, 0.0, "");
        for (int e_i=0; e_i<edges_MILP_.size(); e_i++) {
            if (edges_MILP_[e_i].edge_type == EdgeType::NAVIGATION_AND_LANDING && k==edges_MILP_[e_i].k) {
                constraint->SetCoefficient(f[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ], 1);
                constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ], -_battery_drop_matrices[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ]);
            }
        }
    }

    // (8) For all tours-drones and edges, there is only flow if edge served:
    for (int k=0; k<_battery_drop_matrices.size(); k++) {
        for (int e_i=0; e_i<edges_MILP_.size(); e_i++) {
            operations_research::MPConstraint *constraint = solver->MakeRowConstraint(-infinity, 0.0, "");
            constraint->SetCoefficient(f[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ], 1);
            if (edges_MILP_[e_i].edge_type == EdgeType::INSPECTION) {
                constraint->SetCoefficient(x[ from_k_i_j_to_x_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ], -1.0);
            }
            if ((edges_MILP_[e_i].edge_type != EdgeType::TAKEOFF_AND_NAVIGATION && edges_MILP_[e_i].edge_type != EdgeType::NAVIGATION_AND_LANDING) || k==edges_MILP_[e_i].k) {
                constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ], -1.0);
            }
        }
    }

    // (8.1) For all tours-drones and edges, flow has a minimum value:
    for (int k=0; k<_battery_drop_matrices.size(); k++) {
        for (int e_i=0; e_i<edges_MILP_.size(); e_i++) {
            operations_research::MPConstraint *constraint = solver->MakeRowConstraint(0, infinity, "");
            constraint->SetCoefficient(f[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ], 1);
            if (edges_MILP_[e_i].edge_type == EdgeType::INSPECTION) {
                constraint->SetCoefficient(x[ from_k_i_j_to_x_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ], -_battery_drop_matrices[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ]);
            }
            if ((edges_MILP_[e_i].edge_type != EdgeType::TAKEOFF_AND_NAVIGATION && edges_MILP_[e_i].edge_type != EdgeType::NAVIGATION_AND_LANDING) || k==edges_MILP_[e_i].k) {
                constraint->SetCoefficient(y[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ], -_battery_drop_matrices[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ]);
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
    for (int k=0; k<_time_cost_matrices.size(); k++) {
        std::cout << "x[" << k << "]" << std::endl;
        for (int i=0; i<_time_cost_matrices[k].size(); i++) {
            for (int j=0; j<_time_cost_matrices[k][i].size(); j++) {
                if (i==0 || j==0) {
                    std::cout << _time_cost_matrices[k][i][j] << " ";
                    continue;
                }
                bool inspection_edge_found = false;
                for (Edge& current_edge_struct : edges_MILP_) {
                    if (current_edge_struct.edge_type == EdgeType::INSPECTION && current_edge_struct.i == i && current_edge_struct.j == j) {
                        std::cout << std::setprecision(6) << x[ from_k_i_j_to_x_index_[k][i][j] ]->solution_value() << " ";
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
    for (int k=0; k<_time_cost_matrices.size(); k++) {
        std::cout << "y[" << k << "]" << std::endl;
        for (int i=0; i<_time_cost_matrices[k].size(); i++) {
            for (int j=0; j<_time_cost_matrices[k][i].size(); j++) {
                if (i==0 || j==0) {
                    std::cout << _time_cost_matrices[k][i][j] << " ";
                    continue;
                }
                if (_time_cost_matrices[0][i][j]==-1) {
                    std::cout << "- ";
                } else {
                    std::cout << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[k][i][j] ]->solution_value() << " ";
                }
            }
            std::cout << std::endl;
        }
    }
    for (int k=0; k<_time_cost_matrices.size(); k++) {
        std::cout << "f[" << k << "]" << std::endl;
        for (int i=0; i<_time_cost_matrices[k].size(); i++) {
            for (int j=0; j<_time_cost_matrices[k][i].size(); j++) {
                if (i==0 || j==0) {
                    std::cout << _time_cost_matrices[k][i][j] << " ";
                    continue;
                }
                if (_time_cost_matrices[0][i][j]==-1) {
                    std::cout << "- ";
                } else {
                    std::cout << std::setprecision(6) << f[ from_k_i_j_to_y_and_f_index_[k][i][j] ]->solution_value() << " ";
                }
            }
            std::cout << std::endl;
        }
    }


    // Print the objective and restrictions for debug:
    std::cout<< std::endl << "Print the objective and restrictions for debug:" << std::endl << std::endl;

    // (1) Minimize all the inspection time (UAVs flying separately).
    std::cout << "(1):" << std::endl;
    for (int k=0; k<_time_cost_matrices.size(); k++) {
        for (int e_i=0; e_i<edges_MILP_.size(); e_i++) {
            if (edges_MILP_[e_i].edge_type == EdgeType::INSPECTION) {
                std::cout << "x[" << k << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].j] << "] " << std::setprecision(6) << x[ from_k_i_j_to_x_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ]->solution_value() << " · " << _time_cost_matrices[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] << " + ";
                // std::cout << std::endl;
                // std::cout << "e_i = " << e_i << std::endl;
                // std::cout << "k = " << k << std::endl;
                // std::cout << "edges_MILP_[e_i].i = " << edges_MILP_[e_i].i << std::endl;
                // std::cout << "edges_MILP_[e_i].j = " << edges_MILP_[e_i].j << std::endl;
                // std::cout << "from_k_i_j_to_x_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] = " << from_k_i_j_to_x_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] << std::endl;
                // std::cout << "<< std::setprecision(6) << from_k_i_j_to_x_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ] = " << "x " << x[ from_k_i_j_to_x_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ]->solution_value() << std::endl;
            }
            if ( (edges_MILP_[e_i].edge_type != EdgeType::TAKEOFF_AND_NAVIGATION && edges_MILP_[e_i].edge_type != EdgeType::NAVIGATION_AND_LANDING) || k==edges_MILP_[e_i].k) {
                std::cout << "y[" << k << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].j] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ]->solution_value() << " · " << _time_cost_matrices[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] << " + ";
                // std::cout << std::endl;
                // std::cout << "e_i = " << e_i << std::endl;
                // std::cout << "k = " << k << std::endl;
                // std::cout << "edges_MILP_[e_i].i = " << edges_MILP_[e_i].i << std::endl;
                // std::cout << "edges_MILP_[e_i].j = " << edges_MILP_[e_i].j << std::endl;
                // std::cout << "from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] = " << from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] << std::endl;
                // std::cout << "<< std::setprecision(6) << from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ] = " << "y " << y[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ]->solution_value() << std::endl;
            }
        }
    }
    std::cout << std::endl << std::endl;

    // (2) For all pylon nodes and tours-drones, Sum_all_j(x_ij_k) plus Sum_all_j(y_ij_k) minus Sum_all_j(x_ji_k) and minus Sum_all_j(x_ji_k) is equal to zero (inputs equal to outputs in each node):
    std::cout << "(2):" << std::endl;
    for (int k=0; k<_time_cost_matrices.size(); k++) {
        for (int g_i=0; g_i<_graph.size(); g_i++) {
            if (_graph[g_i].type == aerialcore_msgs::GraphNode::TYPE_PYLON) {
                std::cout << "node_" << g_i << " : ";
                for (int e_i=0; e_i<edges_MILP_.size(); e_i++) {
                    if (edges_MILP_[e_i].i == from_graph_index_to_matrix_index_[g_i]) {
                        if (edges_MILP_[e_i].edge_type == EdgeType::INSPECTION) {
                            std::cout << "x[" << k << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].j] << "] " << std::setprecision(6) << x[ from_k_i_j_to_x_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ]->solution_value() << " · " << 1 << " + ";
                            std::cout << "x[" << k << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].j] << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].i] << "] " << std::setprecision(6) << x[ from_k_i_j_to_x_index_[k][ edges_MILP_[e_i].j ][ edges_MILP_[e_i].i ] ]->solution_value() << " · " << -1 << " + ";
                        }
                        if (edges_MILP_[e_i].edge_type != EdgeType::TAKEOFF_AND_NAVIGATION && edges_MILP_[e_i].edge_type != EdgeType::NAVIGATION_AND_LANDING) {
                            std::cout << "y[" << k << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].j] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ]->solution_value() << " · " << 1 << " + ";
                            std::cout << "y[" << k << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].j] << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].i] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].j ][ edges_MILP_[e_i].i ] ]->solution_value() << " · " << -1 << " + ";
                        }
                    }
                }
                std::cout << std::endl;
            } else if (_graph[g_i].type == aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION) {
                std::cout << "node_" << g_i << " : ";
                for (int e_i=0; e_i<edges_MILP_.size(); e_i++) {
                    if (edges_MILP_[e_i].i == from_graph_index_to_matrix_index_[g_i]) {
                        if (k==edges_MILP_[e_i].k) {
                            std::cout << "y[" << k << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].j] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ]->solution_value() << " · " << 1 << " + ";
                        }
                    } else if (edges_MILP_[e_i].j == from_graph_index_to_matrix_index_[g_i]) {
                        if (k==edges_MILP_[e_i].k) {
                            std::cout << "y[" << k << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].j] << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].i] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].j ][ edges_MILP_[e_i].i ] ]->solution_value() << " · " << -1 << " + ";
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
    for (int e_i=0; e_i<edges_MILP_.size(); e_i++) {
        if (edges_MILP_[e_i].edge_type == EdgeType::INSPECTION && edges_MILP_[e_i].i<edges_MILP_[e_i].j) {
        std::cout << "inspection_edge_" << e_i << " : ";
            for (int k=0; k<_time_cost_matrices.size(); k++) {
                std::cout << "x[" << k << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].j] << "] " << std::setprecision(6) << x[ from_k_i_j_to_x_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ]->solution_value() << " · " << 1 << " + ";
                std::cout << "x[" << k << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].j] << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].i] << "] " << std::setprecision(6) << x[ from_k_i_j_to_x_index_[k][ edges_MILP_[e_i].j ][ edges_MILP_[e_i].i ] ]->solution_value() << " · " << 1 << " + ";
            }
            std::cout << std::endl;
        }
    }
    std::cout << std::endl;

    // (4) For all tours-drones, in edges coming from takeoff nodes, y_0j is 0 or 1 (take off just once in each tour):
    std::cout << "(4):" << std::endl;
    for (int k=0; k<_time_cost_matrices.size(); k++) {
        std::cout << "k_" << k << " : ";
        for (int e_i=0; e_i<edges_MILP_.size(); e_i++) {
            if (edges_MILP_[e_i].edge_type == EdgeType::TAKEOFF_AND_NAVIGATION && k==edges_MILP_[e_i].k) {
                std::cout << "y[" << k << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].j] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ]->solution_value() << " · " << 1 << " + ";
            }
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

    // // (4.1) For all tours-drones, in edges going to land nodes, y_j0 is 0 or 1 (land just once in each tour):
    // std::cout << "(4.1):" << std::endl;
    // for (int k=0; k<_time_cost_matrices.size(); k++) {
    //     std::cout << "k_" << k << " : ";
    //     for (int e_i=0; e_i<edges_MILP_.size(); e_i++) {
    //         if (edges_MILP_[e_i].edge_type == EdgeType::NAVIGATION_AND_LANDING && k==edges_MILP_[e_i].k) {
    //             std::cout << "y[" << k << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].j] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ]->solution_value() << " · " << 1 << " + ";
    //         }
    //     }
    //     std::cout << std::endl;
    // }
    // std::cout << std::endl;

    // (5) For all tours-drones and pylon nodes, equation of flow consumed in node (flow in equals to flow destroyed in that node plus flow out):
    std::cout << "(5):" << std::endl;
    for (int k=0; k<_battery_drop_matrices.size(); k++) {
        for (int g_i=0; g_i<_graph.size(); g_i++) {
            if (_graph[g_i].type == aerialcore_msgs::GraphNode::TYPE_PYLON) {
                std::cout << "pylon_node_" << g_i << " : ";
                for (int e_i=0; e_i<edges_MILP_.size(); e_i++) {
                    if (edges_MILP_[e_i].i == from_graph_index_to_matrix_index_[g_i]) {
                        std::cout << "f[" << k << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].j] << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].i] << "] " << std::setprecision(6) << f[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].j ][ edges_MILP_[e_i].i ] ]->solution_value() << " · " << 1 << " + ";
                        std::cout << "f[" << k << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].j] << "] " << std::setprecision(6) << f[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ]->solution_value() << " · " << -1 << " + ";
                        if (edges_MILP_[e_i].edge_type == EdgeType::INSPECTION) {
                            std::cout << "x[" << k << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].j] << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].i] << "] " << std::setprecision(6) << x[ from_k_i_j_to_x_index_[k][ edges_MILP_[e_i].j ][ edges_MILP_[e_i].i ] ]->solution_value() << " · " << -_battery_drop_matrices[k][ edges_MILP_[e_i].j ][ edges_MILP_[e_i].i ] << " + ";
                        }
                        if ((edges_MILP_[e_i].edge_type != EdgeType::TAKEOFF_AND_NAVIGATION && edges_MILP_[e_i].edge_type != EdgeType::NAVIGATION_AND_LANDING) || k==edges_MILP_[e_i].k) {
                            std::cout << "y[" << k << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].j] << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].i] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].j ][ edges_MILP_[e_i].i ] ]->solution_value() << " · " << -_battery_drop_matrices[k][ edges_MILP_[e_i].j ][ edges_MILP_[e_i].i ] << " + ";
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
    for (int k=0; k<_battery_drop_matrices.size(); k++) {
        std::cout << "k_" << k << " : ";
        for (int e_i=0; e_i<edges_MILP_.size(); e_i++) {
            if (edges_MILP_[e_i].edge_type == EdgeType::TAKEOFF_AND_NAVIGATION && k==edges_MILP_[e_i].k) {
                std::cout << "f[" << k << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].j] << "] " << std::setprecision(6) << f[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ]->solution_value() << " · " << 1 << " + ";
            }
            if (edges_MILP_[e_i].edge_type == EdgeType::INSPECTION) {
                std::cout << "x[" << k << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].j] << "] " << std::setprecision(6) << x[ from_k_i_j_to_x_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ]->solution_value() << " · " << -_battery_drop_matrices[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] << " + ";
            }
            if ((edges_MILP_[e_i].edge_type != EdgeType::TAKEOFF_AND_NAVIGATION && edges_MILP_[e_i].edge_type != EdgeType::NAVIGATION_AND_LANDING) || k==edges_MILP_[e_i].k) {
                std::cout << "y[" << k << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].j] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ]->solution_value() << " · " << -_battery_drop_matrices[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] << " + ";
            }
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

    // (7) For all tours-drones, land nodes receive all the remaining flow:
    std::cout << "(7):" << std::endl;
    for (int k=0; k<_battery_drop_matrices.size(); k++) {
        std::cout << "k_" << k << " : ";
        for (int e_i=0; e_i<edges_MILP_.size(); e_i++) {
            if (edges_MILP_[e_i].edge_type == EdgeType::NAVIGATION_AND_LANDING) {
                std::cout << "f[" << k << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].j] << "] " << std::setprecision(6) << f[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ]->solution_value() << " · " << 1 << " + ";
                std::cout << "y[" << k << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].j] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ]->solution_value() << " · " << -_battery_drop_matrices[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] << " + ";
            }
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

    // (8) For all tours-drones and edges, there is only flow if edge served:
    std::cout << "(8):" << std::endl;
    for (int k=0; k<_battery_drop_matrices.size(); k++) {
        for (int e_i=0; e_i<edges_MILP_.size(); e_i++) {
            std::cout << "edge_" << e_i << " : ";
            std::cout << "f[" << k << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].j] << "] " << std::setprecision(6) << f[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ]->solution_value() << " + ";
            if (edges_MILP_[e_i].edge_type == EdgeType::INSPECTION) {
                std::cout << "x[" << k << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].j] << "] " << std::setprecision(6) << x[ from_k_i_j_to_x_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ]->solution_value() << " + ";
            }
            if ((edges_MILP_[e_i].edge_type != EdgeType::TAKEOFF_AND_NAVIGATION && edges_MILP_[e_i].edge_type != EdgeType::NAVIGATION_AND_LANDING) || k==edges_MILP_[e_i].k) {
                std::cout << "y[" << k << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].i] << "][" << from_matrix_index_to_graph_index_[edges_MILP_[e_i].j] << "] " << std::setprecision(6) << y[ from_k_i_j_to_y_and_f_index_[k][ edges_MILP_[e_i].i ][ edges_MILP_[e_i].j ] ]->solution_value() << " + ";
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

    //////////////////// End defining the MILP problem according to agarwal_icra20 and solve it using OR-Tools ////////////////////

    // Assign flight_plans_ according to the edges in the solution:
    for (int k=0; k<_battery_drop_matrices.size(); k++) {

        struct fStruct {
            float f;
            int x;
            int y;
            int k;
            int i;
            int j;
        };
        std::vector<fStruct> f_struct_vector;

        for (int i=1; i<_battery_drop_matrices[k].size(); i++) {
            for (int j=1; j<_battery_drop_matrices[k][i].size(); j++) {
                if (f[ from_k_i_j_to_y_and_f_index_[k][i][j] ]->solution_value()>0.001) {
                    fStruct new_f_struct;
                    new_f_struct.f = f[ from_k_i_j_to_y_and_f_index_[k][i][j] ]->solution_value();
                    new_f_struct.x = x[ from_k_i_j_to_x_index_[k][i][j] ]->solution_value();
                    new_f_struct.y = y[ from_k_i_j_to_y_and_f_index_[k][i][j] ]->solution_value();
                    new_f_struct.k = k;
                    new_f_struct.i = from_matrix_index_to_graph_index_[i];
                    new_f_struct.j = from_matrix_index_to_graph_index_[j];
                    f_struct_vector.push_back(new_f_struct);
                }
            }
        }

        std::sort(f_struct_vector.begin(), f_struct_vector.end(), [](const fStruct& a, const fStruct& b) { 
            return a.f > b.f; 
        }); // Sort in descending order.

        aerialcore_msgs::FlightPlan current_flight_plan;

        for (const auto& f_struct : f_struct_vector) {
            current_flight_plan.nodes.push_back(f_struct.i);
        }
        if (f_struct_vector.size()>0) {
            current_flight_plan.uav_id = _graph[ f_struct_vector.front().i ].id;
            current_flight_plan.nodes.push_back( f_struct_vector.back().j );
        }

        if (current_flight_plan.nodes.size()>3) {   // Consider the flight plan only if it visits two or more pylons.
#ifdef DEBUG
            std::cout << "current_flight_plan.uav_id: " << current_flight_plan.uav_id << std::endl;
            std::cout << "current_flight_plan.nodes:" << std::endl;

            for (int node : current_flight_plan.nodes) {
                std::cout << node << std::endl;
            }

            bool milp_solution_coherent = f_struct_vector.front().i == f_struct_vector.back().j ? true : false;
            for (int index = 0; index<f_struct_vector.size()-1; index++) {
                if (milp_solution_coherent) {
                    milp_solution_coherent = f_struct_vector[index].j == f_struct_vector[index + 1].i ? true : false;
                } else if (!milp_solution_coherent) {
                    break;
                }
            }
            std::cout << "milp_solution_coherent = " << milp_solution_coherent << std::endl;
#endif

            flight_plans_.push_back(current_flight_plan);
        }
    }

    // Calculate the path free of obstacles between nodes outside in the Mission Controller. There is a path warantied between the nodes.

    return flight_plans_;

} // end getPlanMILP method


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


} // end namespace aerialcore
