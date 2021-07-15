/**
 * AERIALCORE Project:
 *
 * Plan Monitor.
 * 
 */

#include <plan_monitor.h>

#include <ros/ros.h>
#include <math.h>

// #define PLOT_GRAPH              // Uncoment for plotting a graphic (using matplotlib-cpp) of the perpendicular to the segment.
#ifdef PLOT_GRAPH
#include "matplotlibcpp.h"      // matplotlib-cpp has a MIT License (MIT), Copyright (c) 2014 Benno Evers. The full license description of matplotlib, matplotlib-cpp and its README can be found at its root.
namespace plt = matplotlibcpp;
#endif

namespace aerialcore {

#define DEBUG       // UNCOMMENT FOR PRINTING VISUALIZATION OF RESULTS (DEBUG MODE)

// Brief Constructor
PlanMonitor::PlanMonitor() {
    ros::NodeHandle nh;
    nh.param<float>("deviation_limit", deviation_limit_, 0.1);
}


// Brief Destructor
PlanMonitor::~PlanMonitor() {}


// Method called periodically in an external thread, located in the Mission Controller, that will call the planner in the same thread if it returns true:
bool PlanMonitor::enoughDeviationToReplan(const std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector<aerialcore_msgs::FlightPlan>& _flight_plans, const std::vector< std::tuple<float, float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices, const std::map<int, std::map<int, std::map<int, float> > >& _battery_drop_matrices) {
    ROS_INFO("Plan Monitor: checking if enoughDeviationToReplan.");

    ros::Time time_now = ros::Time::now();

    if (previous_flight_plans_ != _flight_plans) {
        UAVs_.clear();
    }
    previous_flight_plans_ = _flight_plans;

    constructUAVs(_drone_info);

    double total_duration_planned = 0;   // Current planned duration up until the place where this method is called (for all UAVs).
    double total_duration_real = 0;      // Real current duration up until the place where this method is called (for all UAVs).
    std::map<int, double> duration_planned;  // Current planned duration up until the place where this method is called (for each UAV).
    std::map<int, double> duration_real;     // Real current duration up until the place where this method is called (for each UAV).

    double total_battery_drop_planned = 0;   // Current planned battery drop up until the place where this method is called (for all UAVs).
    double total_battery_drop_real = 0;      // Real current battery drop up until the place where this method is called (for all UAVs).
    std::map<int, double> battery_drop_planned;  // Current planned battery drop up until the place where this method is called (for each UAV).
    std::map<int, double> battery_drop_real;     // Real current battery drop up until the place where this method is called (for each UAV).

    // Iterate the planned UAVs to calculate the duration battery drop both planned and real for each one of them and all of them in total:
    for (int i=0; i<_flight_plans.size(); i++) {
        int uav_index = findUavIndexById(_flight_plans[i].uav_id);

        // Search for the initial node in the graph of the current UAV of the plan:
        float x_ini, y_ini/*, z_ini*/;   // Define the x-y variables for the initial UAV position.
        for (int g_i=0; g_i<_graph.size(); g_i++) {
            if (_graph[g_i].type == aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION && _graph[g_i].id==_flight_plans[i].uav_id) {
                x_ini = _graph[g_i].x;
                y_ini = _graph[g_i].y;
                // z_ini = _graph[g_i].z;
                break;
            }
        }

        // Calculate the segment (first pose of the segment) where the UAV is right now:
        std::vector<float> perpendicular_distances;
        std::vector<int> indexes;
        for (int j=UAVs_[uav_index].last_pose_index; j<_flight_plans[i].poses.size()-1; j++) {

            // Find the intersection point between the two following lines: segment of the current pair of nodes, and the perpendicular line to that previous line passing by the UAV pose:

            // Angle of the current segment (the one perpendicular to this one will be alpha + M_PI/2):
            float alpha = atan2(_flight_plans[i].poses[j+1].pose.position.y - _flight_plans[i].poses[j].pose.position.y, _flight_plans[i].poses[j+1].pose.position.x - _flight_plans[i].poses[j].pose.position.x);

            // x-y for the intersection point between the two lines:
            float x_inter = (y_ini-_flight_plans[i].poses[j].pose.position.y - tan(alpha + M_PI/2) * (x_ini-_flight_plans[i].poses[j].pose.position.x)) / (tan(alpha) - tan(alpha + M_PI/2))+_flight_plans[i].poses[j].pose.position.x;
            float y_inter = y_ini + tan(alpha + M_PI/2) * (x_inter - x_ini);

            // With that intersection calculate its perpendicular distance:
            if (_flight_plans[i].poses[j].pose.position.x<=_flight_plans[i].poses[j+1].pose.position.x && x_inter>=_flight_plans[i].poses[j].pose.position.x && x_inter<=_flight_plans[i].poses[j+1].pose.position.x
             || _flight_plans[i].poses[j].pose.position.x>=_flight_plans[i].poses[j+1].pose.position.x && x_inter<=_flight_plans[i].poses[j].pose.position.x && x_inter>=_flight_plans[i].poses[j+1].pose.position.x) {
                perpendicular_distances.push_back(sqrt( pow(x_ini-x_inter,2) + pow(y_ini-y_inter,2) ));
            } else {
                // If the initial point isn't in the projection of the segment, push its distance to the start of the segment.
                perpendicular_distances.push_back( sqrt( pow(x_ini - _flight_plans[i].poses[j].pose.position.x,2) + pow(y_ini - _flight_plans[i].poses[j].pose.position.y,2) ) );
            }
            indexes.push_back(j);
        }
        int minimum_perpendicular_distance_index = std::min_element(perpendicular_distances.begin(),perpendicular_distances.end()) - perpendicular_distances.begin();
        UAVs_[uav_index].last_pose_index = indexes[minimum_perpendicular_distance_index];

        // Now we know the first pose of the segment, let's find out its node and the next one:
        int last_node_in_poses = -1;
        for (int j=UAVs_[uav_index].last_pose_index; j>=0; j--) {
            if (_flight_plans[i].type[j]!=aerialcore_msgs::FlightPlan::TYPE_PASS_NFZ_WP) {
                last_node_in_poses = j;
                break;
            }
        }
        int next_node_in_poses = -1;
        for (int j= UAVs_[uav_index].last_pose_index+1<_flight_plans[i].poses.size() ? UAVs_[uav_index].last_pose_index+1 : UAVs_[uav_index].last_pose_index ; j<_flight_plans[i].poses.size(); j++) {
            if (_flight_plans[i].type[j]!=aerialcore_msgs::FlightPlan::TYPE_PASS_NFZ_WP) {
                next_node_in_poses = j;
                break;
            }
        }
        int last_node_in_graph = -1;
        for (int g_i=0; g_i<_graph.size(); g_i++) {
            if (last_node_in_poses==0 && _graph[g_i].type==aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION && _graph[g_i].id==_flight_plans[i].uav_id
             || _graph[g_i].x==_flight_plans[i].poses[last_node_in_poses].pose.position.x && _graph[g_i].y==_flight_plans[i].poses[last_node_in_poses].pose.position.y) {
                last_node_in_graph = g_i;
                break;
            }
        }
        UAVs_[uav_index].last_flight_plan_graph_node = last_node_in_graph;
        int next_node_in_graph = -1;
        for (int g_i=0; g_i<_graph.size(); g_i++) {
            if (next_node_in_poses==0 && _graph[g_i].type==aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION && _graph[g_i].id==_flight_plans[i].uav_id
             || _graph[g_i].x==_flight_plans[i].poses[next_node_in_poses].pose.position.x && _graph[g_i].y==_flight_plans[i].poses[next_node_in_poses].pose.position.y) {
                next_node_in_graph = g_i;
                break;
            }
        }

        // If edge of takeoff or landing, do not try to estimate proportional time executed in the edge as is not linear during the execution of those edges:
        if ( (_flight_plans[i].type[last_node_in_poses]==aerialcore_msgs::FlightPlan::TYPE_TAKEOFF_WP || _flight_plans[i].type[next_node_in_poses]==aerialcore_msgs::FlightPlan::TYPE_LAND_WP) && UAVs_[uav_index].flying_or_landed_initially) {
            continue;
        }
        // Also, don't calculate deviation if not flying.
        if ( !UAVs_[uav_index].flying_or_landed_initially ) {
            continue;
        }

        // Calculate the duration and battery drop planned:
        duration_planned[uav_index] = 0;
        battery_drop_planned[uav_index] = 0;
        for (int j=0; j<_flight_plans[i].nodes.size()-1; j++) {
            if (_flight_plans[i].nodes[j]==last_node_in_graph) {
                break;
            }
            duration_planned[uav_index] += _time_cost_matrices.at( _flight_plans[i].uav_id ).at( _flight_plans[i].nodes[j] ).at( _flight_plans[i].nodes[j+1] );
            battery_drop_planned[uav_index] += _battery_drop_matrices.at( _flight_plans[i].uav_id ).at( _flight_plans[i].nodes[j] ).at( _flight_plans[i].nodes[j+1] );
        }
        float distance_from_last_to_next_node = 0;
        float distance_from_last_node_to_intersection = 0;
        for (int j=last_node_in_poses; j<next_node_in_poses; j++) {
            float segment_distance;
            // if (j==0) { // The initial position of the UAV is not in the pose path, but should be taken into account:
            //     segment_distance = sqrt( pow(_flight_plans[i].poses[0].pose.position.x - x_ini,2) + pow(_flight_plans[i].poses[0].pose.position.y - y_ini,2) + pow(_flight_plans[i].poses[0].pose.position.z - z_ini,2) );
            //     distance_from_last_to_next_node += segment_distance;
            //     distance_from_last_node_to_intersection += segment_distance;
            // }
            segment_distance = sqrt( pow(_flight_plans[i].poses[j+1].pose.position.x-_flight_plans[i].poses[j].pose.position.x,2) + pow(_flight_plans[i].poses[j+1].pose.position.y-_flight_plans[i].poses[j].pose.position.y,2) );
            distance_from_last_to_next_node += segment_distance;
            if (j<UAVs_[uav_index].last_pose_index) {
                distance_from_last_node_to_intersection += segment_distance;
            } else if (j==UAVs_[uav_index].last_pose_index) {
                // Angle of the current segment (the one perpendicular to this one will be alpha + M_PI/2):
                float alpha = atan2(_flight_plans[i].poses[j+1].pose.position.y - _flight_plans[i].poses[j].pose.position.y, _flight_plans[i].poses[j+1].pose.position.x - _flight_plans[i].poses[j].pose.position.x);

                // x-y for the intersection point between the two lines:
                float x_inter = (y_ini-_flight_plans[i].poses[j].pose.position.y - tan(alpha + M_PI/2) * (x_ini-_flight_plans[i].poses[j].pose.position.x)) / (tan(alpha) - tan(alpha + M_PI/2))+_flight_plans[i].poses[j].pose.position.x;
                float y_inter = y_ini + tan(alpha + M_PI/2) * (x_inter - x_ini);

                distance_from_last_node_to_intersection += sqrt( pow(x_inter-_flight_plans[i].poses[j].pose.position.x,2) + pow(y_inter-_flight_plans[i].poses[j].pose.position.y,2) );

#ifdef PLOT_GRAPH
                std::vector<double> x_plot_segment, y_plot_segment;
                x_plot_segment.push_back( _flight_plans[i].poses[j].pose.position.x );
                x_plot_segment.push_back( _flight_plans[i].poses[j+1].pose.position.x );
                y_plot_segment.push_back( _flight_plans[i].poses[j].pose.position.y );
                y_plot_segment.push_back( _flight_plans[i].poses[j+1].pose.position.y );
                plt::plot(x_plot_segment,y_plot_segment,"r");

                std::vector<double> x_plot_perp, y_plot_perp;
                x_plot_perp.push_back( x_ini );
                x_plot_perp.push_back( x_inter );
                y_plot_perp.push_back( y_ini );
                y_plot_perp.push_back( y_inter );
                plt::plot(x_plot_perp,y_plot_perp,"b");

                plt::axis("equal");

                std::cout << "x_ini   = " << x_ini << std::endl;
                std::cout << "y_ini   = " << y_ini << std::endl;
                std::cout << "x_inter = " << x_inter << std::endl;
                std::cout << "y_inter = " << y_inter << std::endl;
                std::cout << "_flight_plans[i].poses[j].pose.position.x   = " << _flight_plans[i].poses[j].pose.position.x << std::endl;
                std::cout << "_flight_plans[i].poses[j].pose.position.y   = " << _flight_plans[i].poses[j].pose.position.y << std::endl;
                std::cout << "_flight_plans[i].poses[j+1].pose.position.x = " << _flight_plans[i].poses[j+1].pose.position.x << std::endl;
                std::cout << "_flight_plans[i].poses[j+1].pose.position.y = " << _flight_plans[i].poses[j+1].pose.position.y << std::endl;

                plt::show();
#endif
            }
        }

// # ifdef DEBUG
//         std::cout << "duration_planned[uav_index]             = " << duration_planned[uav_index] << std::endl;
//         std::cout << "distance_from_last_node_to_intersection = " << distance_from_last_node_to_intersection << std::endl;
//         std::cout << "distance_from_last_to_next_node         = " << distance_from_last_to_next_node << std::endl;
//         float aux_1 = distance_from_last_node_to_intersection/distance_from_last_to_next_node;
//         std::cout << "distance_from_last_node_to_intersection/distance_from_last_to_next_node = " << aux_1 << std::endl;
//         std::cout << "_time_cost_matrices.at( _flight_plans[i].uav_id ).at(last_node_in_graph).at(next_node_in_graph) = " << _time_cost_matrices.at( _flight_plans[i].uav_id ).at(last_node_in_graph).at(next_node_in_graph) << std::endl;
//         float aux_2 = distance_from_last_node_to_intersection/distance_from_last_to_next_node * _time_cost_matrices.at( _flight_plans[i].uav_id ).at(last_node_in_graph).at(next_node_in_graph);
//         std::cout << "distance_from_last_node_to_intersection/distance_from_last_to_next_node * _time_cost_matrices.at( _flight_plans[i].uav_id ).at(last_node_in_graph).at(next_node_in_graph) = " << aux_2 << std::endl;
// # endif

        // Calculate the real and planned duration and battery drop only if real duration positive (negative if delay):
        duration_real[uav_index] = time_now.toSec() - _flight_plans[i].header.stamp.toSec();
        if (duration_real[uav_index]>=0) {
            total_duration_real += duration_real[uav_index];

            duration_planned[uav_index] += distance_from_last_node_to_intersection/distance_from_last_to_next_node * _time_cost_matrices.at( _flight_plans[i].uav_id ).at(last_node_in_graph).at(next_node_in_graph);
            total_duration_planned += duration_planned[uav_index];

            // Calculate the real battery drop:
            battery_drop_real[uav_index] = UAVs_[uav_index].initial_battery - UAVs_[uav_index].battery;
            total_battery_drop_real += battery_drop_real[uav_index];

            // Calculate the planned battery droped:
            battery_drop_planned[uav_index] += distance_from_last_node_to_intersection/distance_from_last_to_next_node * _battery_drop_matrices.at( _flight_plans[i].uav_id ).at(last_node_in_graph).at(next_node_in_graph);   // TODO: wind considered properly up until the last node, for poses of no-fly zones (middle cost inside the edge) wind cost calculated with a rule of three by distances.
            total_battery_drop_planned += battery_drop_planned[uav_index];
        } else {
            duration_real.erase(uav_index);
            duration_planned.erase(uav_index);
            battery_drop_real.erase(uav_index);
            battery_drop_planned.erase(uav_index);
        }

# ifdef DEBUG
        // std::cout << "UAV_id " << _flight_plans[i].uav_id << " -> last_node_in_poses = " << last_node_in_poses << std::endl;
        // std::cout << "UAV_id " << _flight_plans[i].uav_id << " -> last_pose_index          = " << UAVs_[uav_index].last_pose_index << std::endl;
        // std::cout << "UAV_id " << _flight_plans[i].uav_id << " -> next_node_in_poses = " << next_node_in_poses << std::endl;
        // std::cout << "UAV_id " << _flight_plans[i].uav_id << " -> last_node_in_graph = " << last_node_in_graph << std::endl;
        // std::cout << "UAV_id " << _flight_plans[i].uav_id << " -> next_node_in_graph = " << next_node_in_graph << std::endl;
        std::cout << "duration_planned [ " << _flight_plans[i].uav_id << " ] = " << duration_planned[uav_index] << std::endl;
        std::cout << "duration_real    [ " << _flight_plans[i].uav_id << " ] = " << duration_real[uav_index] << std::endl;
        std::cout << "battery_drop_planned [ " << _flight_plans[i].uav_id << " ] = " << battery_drop_planned[uav_index] << std::endl;
        std::cout << "battery_drop_real    [ " << _flight_plans[i].uav_id << " ] = " << battery_drop_real[uav_index] << std::endl;
# endif

    }

# ifdef DEBUG
    std::cout << "total_duration_planned = " << total_duration_planned << std::endl;
    std::cout << "total_duration_real    = " << total_duration_real << std::endl;
    std::cout << "total_battery_drop_planned = " << total_battery_drop_planned << std::endl;
    std::cout << "total_battery_drop_real    = " << total_battery_drop_real << std::endl;
# endif

    // Compare the total planned duration and battery drop with the real ones:
    if ( (total_duration_planned>0 && total_duration_real>0) && (total_duration_planned<total_duration_real*(1-deviation_limit_) || total_duration_planned>total_duration_real*(1+deviation_limit_)) ) {
        ROS_INFO("Plan Monitor: Replanning true. Total sum of duration deviations among all UAVs greater than threshold.");
        return true;
    } else if ( (total_battery_drop_planned>0 && total_battery_drop_real>0) && (total_battery_drop_planned<total_battery_drop_real*(1-deviation_limit_) || total_battery_drop_planned>total_battery_drop_real*(1+deviation_limit_)) ) {
        ROS_INFO("Plan Monitor: Replanning true. Total sum of battery drop deviations among all UAVs greater than threshold.");
        return true;
    } else {
        // Compare the planned duration of each UAV with the real one:
        for (int i=0; i<_flight_plans.size(); i++) {
            int uav_index = findUavIndexById(_flight_plans[i].uav_id);

            if (duration_real.count(uav_index)>0 && duration_real[uav_index]>0 && (duration_planned[uav_index]<duration_real[uav_index]*(1-deviation_limit_) || duration_planned[uav_index]>duration_real[uav_index]*(1+deviation_limit_))) {
                ROS_INFO("Plan Monitor: Replanning true. Deviation of duration for the UAV of id=%d greater than threshold.", _flight_plans[i].uav_id);
                return true;
            } else if (battery_drop_real.count(uav_index)>0 && battery_drop_real[uav_index]>0 && battery_drop_planned[uav_index]<battery_drop_real[uav_index]*(1-deviation_limit_) || battery_drop_planned[uav_index]>battery_drop_real[uav_index]*(1+deviation_limit_)) {
                ROS_INFO("Plan Monitor: Replanning true. Deviation of battery drop for the UAV of id=%d greater than threshold.", _flight_plans[i].uav_id);
                return true;
            }
        }
        ROS_INFO("Plan Monitor: Replanning false.");
        return false;
    }

} // end enoughDeviationToReplan


void PlanMonitor::constructUAVs(const std::vector< std::tuple<float, float, float, int, int, int, int, int, int, bool, bool> >& _drone_info) {

    std::vector<UAV> UAVs_new;

    // _drone_info it's a vector of tuples, each tuple with 11 elements. The first in the tuple is the initial battery, and so on with all the elements in the "UAV" structure defined here below.
    for (const std::tuple<float, float, float, int, int, int, int, int, int, bool, bool>& current_drone : _drone_info) {
        UAV actual_UAV;
        actual_UAV.battery = std::get<0>(current_drone);
        actual_UAV.initial_battery = std::get<1>(current_drone);
        actual_UAV.minimum_battery = std::get<2>(current_drone);
        actual_UAV.time_until_fully_charged = std::get<3>(current_drone);
        actual_UAV.time_max_flying = std::get<4>(current_drone);
        actual_UAV.speed_xy = std::get<5>(current_drone);
        actual_UAV.speed_z_down = std::get<6>(current_drone);
        actual_UAV.speed_z_up = std::get<7>(current_drone);
        actual_UAV.id = std::get<8>(current_drone);
        actual_UAV.flying_or_landed_initially = std::get<9>(current_drone);
        actual_UAV.recharging_initially = std::get<10>(current_drone);
        int uav_index = findUavIndexById(actual_UAV.id);
        actual_UAV.last_pose_index = uav_index==-1 ? 0 : UAVs_[uav_index].last_pose_index;
        actual_UAV.last_flight_plan_graph_node = uav_index==-1 ? -1 : UAVs_[uav_index].last_flight_plan_graph_node;
        UAVs_new.push_back(actual_UAV);
    }

    UAVs_.clear();
    UAVs_ = UAVs_new;
} // end constructUAVs


int PlanMonitor::findUavIndexById(int _UAV_id) {
    int uav_index = -1;
    for (int i=0; i<UAVs_.size(); i++) {
        if (UAVs_[i].id == _UAV_id) {
            uav_index = i;
            break;
        }
    }
    return uav_index;
} // end findUavIndexById


std::map<int, int> PlanMonitor::lastFlightPlanGraphNode() {
    std::map<int, int> map_to_return;
    for (int i=0; i<UAVs_.size(); i++) {
        map_to_return[ UAVs_[i].id ] = UAVs_[i].last_flight_plan_graph_node;
    }
    return map_to_return;
} // end lastFlightPlanGraphNode


} // end namespace aerialcore
