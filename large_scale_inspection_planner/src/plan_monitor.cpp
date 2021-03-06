/**
 * AERIALCORE Project:
 *
 * Plan Monitor.
 * 
 */

#include <plan_monitor.h>

#include <ros/ros.h>
#include <math.h>

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
bool PlanMonitor::enoughDeviationToReplan(const std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector<aerialcore_msgs::FlightPlan>& _flight_plans, const std::vector< std::tuple<float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices, const std::map<int, std::map<int, std::map<int, float> > >& _battery_drop_matrices) {

    ros::Time time_now = ros::Time::now();

    constructUAVs(_drone_info);

    float total_duration_planned = 0;   // Current planned duration up until the place where this method is called (for all UAVs).
    float total_duration_real = 0;      // Real current duration up until the place where this method is called (for all UAVs).
    std::map<int, float> duration_planned;  // Current planned duration up until the place where this method is called (for each UAV).
    std::map<int, float> duration_real;     // Real current duration up until the place where this method is called (for each UAV).

    // Iterate the planned UAVs to calculate the duration planned and real for each one of them and all of them in total:
    for (int i=0; i<_flight_plans.size(); i++) {
        int uav_index = findUavIndexById(_flight_plans[i].uav_id);

        // Search for the initial node in the graph of the current UAV of the plan:
        float x_ini, y_ini;   // Define the x-y variables for the initial UAV position.
        for (int g_i=0; g_i<_graph.size(); g_i++) {
            if (_graph[g_i].type == aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION && _graph[g_i].id==_flight_plans[i].uav_id) {
                x_ini = _graph[g_i].x;
                y_ini = _graph[g_i].y;
                break;
            }
        }

        // Calculate the segment (first pose of the segment) where the UAV is right now:
        std::vector<float> perpendicular_distances;
        for (int j=UAVs_[uav_index].last_segment; j<_flight_plans[i].poses.size()-1; j++) {

            // Find the intersection point between the two following lines: segment of the current pair of nodes, and the perpendicular line to that previous line passing by the UAV pose:

            // Angle of the current segment (the one perpendicular to this one will be alpha + M_PI/2):
            float alpha = atan2(_flight_plans[i].poses[j+1].pose.position.y - _flight_plans[i].poses[j].pose.position.y, _flight_plans[i].poses[j+1].pose.position.x - _flight_plans[i].poses[j].pose.position.x);

            // x-y for the intersection point between the two lines:
            float x_inter = (y_ini - tan(alpha + M_PI/2) * x_ini) / (tan(alpha) - tan(alpha + M_PI/2));
            float y_inter = y_ini + tan(alpha + M_PI/2) * (x_inter - x_ini);

            // With that intersection calculate its perpendicular distance:
            if (_flight_plans[i].poses[j].pose.position.x<=_flight_plans[i].poses[j+1].pose.position.x && x_inter>=_flight_plans[i].poses[j].pose.position.x && x_inter<=_flight_plans[i].poses[j+1].pose.position.x
             || _flight_plans[i].poses[j].pose.position.x>=_flight_plans[i].poses[j+1].pose.position.x && x_inter<=_flight_plans[i].poses[j].pose.position.x && x_inter>=_flight_plans[i].poses[j+1].pose.position.x) {
                perpendicular_distances.push_back(sqrt( pow(x_ini-x_inter,2) + pow(y_ini-y_inter,2) ));
            } else {
                perpendicular_distances.push_back(std::numeric_limits<float>::max());
            }

            // Iterate only the first 3 segments, usually the first or second one will be the closest:
            if (perpendicular_distances.size()==3) {
                if (perpendicular_distances[2]<perpendicular_distances[1] && perpendicular_distances[2]<perpendicular_distances[0]) {
                    UAVs_[uav_index].last_segment = j;
                    break;
                } else if (perpendicular_distances[1]<perpendicular_distances[2] && perpendicular_distances[1]<perpendicular_distances[0]) {
                    UAVs_[uav_index].last_segment = j-1;
                    break;
                } else {
                    UAVs_[uav_index].last_segment = j-2;
                    break;
                }
            }
        }

        // Now we know the first pose of the segment, let's find out its node and the next one:
        int last_index_node_in_poses = -1;
        for (int j=UAVs_[uav_index].last_segment; j>=0; j--) {
            if (_flight_plans[i].type[j]!=aerialcore_msgs::FlightPlan::TYPE_PASS_NFZ_WP) {
                last_index_node_in_poses = j;
                break;
            }
        }
        int next_index_node_in_poses = -1;
        for (int j= UAVs_[uav_index].last_segment+1<_flight_plans[i].poses.size() ? UAVs_[uav_index].last_segment+1 : UAVs_[uav_index].last_segment ; j<_flight_plans[i].poses.size(); j++) {
            if (_flight_plans[i].type[j]!=aerialcore_msgs::FlightPlan::TYPE_PASS_NFZ_WP) {
                next_index_node_in_poses = j;
                break;
            }
        }
        int last_index_node_in_graph = -1;
        for (int g_i=0; g_i<_graph.size(); g_i++) {
            if (last_index_node_in_poses==0 && _graph[g_i].type==aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION && _graph[g_i].id==_flight_plans[i].uav_id
             || _graph[g_i].x==_flight_plans[i].poses[last_index_node_in_poses].pose.position.x && _graph[g_i].y==_flight_plans[i].poses[last_index_node_in_poses].pose.position.y) {
                last_index_node_in_graph = g_i;
                break;
            }
        }
        int next_index_node_in_graph = -1;
        for (int g_i=0; g_i<_graph.size(); g_i++) {
            if (next_index_node_in_poses==0 && _graph[g_i].type==aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION && _graph[g_i].id==_flight_plans[i].uav_id
             || _graph[g_i].x==_flight_plans[i].poses[next_index_node_in_poses].pose.position.x && _graph[g_i].y==_flight_plans[i].poses[next_index_node_in_poses].pose.position.y) {
                next_index_node_in_graph = g_i;
                break;
            }
        }

        // Calculate the duration planned:
        duration_planned[uav_index] = 0;
        for (int j=0; j<_flight_plans[i].nodes.size()-1; j++) {
            duration_planned[uav_index] += _time_cost_matrices.at( _flight_plans[i].uav_id ).at( _flight_plans[i].nodes[j] ).at( _flight_plans[i].nodes[j+1] );
            if (_flight_plans[i].nodes[j]==last_index_node_in_graph) {
                break;
            }
        }
        if (_flight_plans[i].type[ UAVs_[uav_index].last_segment ]==aerialcore_msgs::FlightPlan::TYPE_PASS_NFZ_WP) {
            float distance_up_until_next_node = 0;
            float distance_up_until_intersection = 0;
            for (int j=last_index_node_in_poses; j<=next_index_node_in_poses-1; j++) {
                float segment_distance = sqrt( pow(_flight_plans[i].poses[j+1].pose.position.x-_flight_plans[i].poses[j].pose.position.x,2) + pow(_flight_plans[i].poses[j+1].pose.position.y-_flight_plans[i].poses[j].pose.position.y,2) );
                distance_up_until_next_node += segment_distance;
                if (j<UAVs_[uav_index].last_segment) {
                    distance_up_until_intersection += segment_distance;
                } else if (j==UAVs_[uav_index].last_segment) {
                    // Angle of the current segment (the one perpendicular to this one will be alpha + M_PI/2):
                    float alpha = atan2(_flight_plans[i].poses[j+1].pose.position.y - _flight_plans[i].poses[j].pose.position.y, _flight_plans[i].poses[j+1].pose.position.x - _flight_plans[i].poses[j].pose.position.x);

                    // x-y for the intersection point between the two lines:
                    float x_inter = (y_ini - tan(alpha + M_PI/2) * x_ini) / (tan(alpha) - tan(alpha + M_PI/2));
                    float y_inter = y_ini + tan(alpha + M_PI/2) * (x_inter - x_ini);

                    distance_up_until_intersection += sqrt( pow(x_inter-_flight_plans[i].poses[j].pose.position.x,2) + pow(y_inter-_flight_plans[i].poses[j].pose.position.y,2) );
                }
            }
            duration_planned[uav_index] += distance_up_until_intersection/distance_up_until_next_node * _time_cost_matrices.at( _flight_plans[i].uav_id ).at(last_index_node_in_graph).at(next_index_node_in_graph);   // TODO: wind considered properly up until the last node, for poses of no-fly zones (middle cost inside the edge) wind cost calculated with a rule of three by distances.
        }
        total_duration_planned += duration_planned[uav_index];

# ifdef DEBUG
        std::cout << "UAV_id " << _flight_plans[i].uav_id << " -> last_segment pose        = " << UAVs_[uav_index].last_segment << std::endl;
        std::cout << "UAV_id " << _flight_plans[i].uav_id << " -> last_index_node_in_poses = " << last_index_node_in_poses << std::endl;
        std::cout << "UAV_id " << _flight_plans[i].uav_id << " -> next_index_node_in_poses = " << next_index_node_in_poses << std::endl;
        std::cout << "UAV_id " << _flight_plans[i].uav_id << " -> last_index_node_in_graph = " << last_index_node_in_graph << std::endl;
        std::cout << "UAV_id " << _flight_plans[i].uav_id << " -> next_index_node_in_graph = " << next_index_node_in_graph << std::endl;
# endif

        // Calculate the real duration:
        duration_real[uav_index] = time_now.toSec() - _flight_plans[i].header.stamp.toSec();
        total_duration_real += duration_real[uav_index];

    }

# ifdef DEBUG
    std::cout << "total_duration_planned = " << total_duration_planned << std::endl;
    std::cout << "total_duration_real = " << total_duration_real << std::endl;
# endif

    // Compare the total planned duration with the real one:
    if (total_duration_planned<total_duration_real*(1-deviation_limit_)
     || total_duration_planned>total_duration_real*(1+deviation_limit_)) {
        ROS_INFO("Plan Monitor: Replanning true.");
        return false;
        // return true;
    } else {
        // Compare the planned duration of each UAV with the real one:
        for (int i=0; i<_flight_plans.size(); i++) {
            int uav_index = findUavIndexById(_flight_plans[i].uav_id);

# ifdef DEBUG
            std::cout << "duration_planned[ " << _flight_plans[i].uav_id << " ] = " << duration_planned[uav_index] << std::endl;
            std::cout << "duration_real[ " << _flight_plans[i].uav_id << " ] = " << duration_real[uav_index] << std::endl;
# endif

            if (duration_planned[uav_index]<duration_real[uav_index]*(1-deviation_limit_)
             || duration_planned[uav_index]>duration_real[uav_index]*(1+deviation_limit_)) {
                ROS_INFO("Plan Monitor: Replanning true.");
                return false;
                // return true;
            }
        }
        ROS_INFO("Plan Monitor: Replanning false.");
        return false;
    }

} // end enoughDeviationToReplan


// // Check if _flight_plans has changed compared with before:
// bool PlanMonitor::checkIfPlansHaveChanged (const std::vector<aerialcore_msgs::FlightPlan>& _flight_plans) const {
//     if (last_flight_plans_.size()==0) {
//         return true;
//     } else {
//         if (last_flight_plans_.size()!=_flight_plans.size()) {
//             return true;
//         } else {
//             for (int i=0; i<last_flight_plans_.size(); i++) {
//                 if (last_flight_plans_[i].uav_id!=_flight_plans[i].uav_id) {
//                     return true;
//                 } else {
//                     for (int j=0; last_flight_plans_[i].nodes.size(); j++) {
//                         if (last_flight_plans_[i].nodes[j]!=_flight_plans[i].nodes[j]) {
//                             return true;
//                         }
//                     }
//                 }
//             }
//         }
//     }
//     return false;
// } // end checkIfPlansHaveChanged


void PlanMonitor::constructUAVs(const std::vector< std::tuple<float, float, int, int, int, int, int, int, bool, bool> >& _drone_info) {

    std::vector<UAV> UAVs_new;

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
        int uav_index = findUavIndexById(actual_UAV.id);
        actual_UAV.last_segment = uav_index==-1 ? 0 : UAVs_[uav_index].last_segment;
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


} // end namespace aerialcore
