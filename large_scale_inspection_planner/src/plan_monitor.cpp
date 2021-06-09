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


// Brief Constructor
PlanMonitor::PlanMonitor() {
    ros::NodeHandle nh;
    nh.param<float>("deviation_limit", deviation_limit_, 0.1);
}


// Brief Destructor
PlanMonitor::~PlanMonitor() {}


// Method called periodically in an external thread, located in the Mission Controller, that will call the planner (in the same thread) if it returns true:
bool PlanMonitor::enoughDeviationToReplan(const std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector<aerialcore_msgs::FlightPlan>& _flight_plans, const std::vector< std::tuple<float, float, int, int, int, int, int, int, bool, bool> >& _drone_info, const std::vector< std::vector< std::vector<float> > >& _time_cost_matrices, const std::vector< std::vector< std::vector<float> > >& _battery_drop_matrices) {

    if (checkIfPlansHaveChanged(_flight_plans)) {
        last_flight_plans_ = _flight_plans;

        constructUAVs(_drone_info);

        // Calculate the duration planned of these plans and its total:
        total_duration_planned_ = 0;
        for (int i=0; i<_flight_plans.size(); i++) {
            int uav_index = findUavIndexById(_flight_plans[i].uav_id);
            UAVs_[uav_index].duration_planned = 0;
            for (int j=0; j<_flight_plans[i].nodes.size()-1; j++) {
                UAVs_[uav_index].duration_planned += _time_cost_matrices[uav_index][ _flight_plans[i].nodes[j] ][ _flight_plans[i].nodes[j+1] ];    // TODO: the k in the matrices doesn't need to be the same as the one in the plans. DANGER. May be better to use the id of the UAVs than some random index.
            }
            total_duration_planned_ += UAVs_[uav_index].duration_planned;
        }
    }

    // Calculate current estimation of the duration: TODO
    float total_duration_estimated;
    std::map<int, float> duration_estimated;
    // use last node to speed up the process. Perpendicular.

    // Compare the total planned duration with the one estimated just before:
    if (total_duration_planned_<total_duration_estimated*(1-deviation_limit_) ||
        total_duration_planned_>total_duration_estimated*(1+deviation_limit_)) {
        return true;
    } else {
        // Compare the planned duration of each UAV with the one estimated just before:
        for (int i=0; i<_flight_plans.size(); i++) {
            int uav_index = findUavIndexById(_flight_plans[i].uav_id);
            if (UAVs_[uav_index].duration_planned<duration_estimated[uav_index]*(1-deviation_limit_) ||
                UAVs_[uav_index].duration_planned>duration_estimated[uav_index]*(1+deviation_limit_)) {
                return true;
            }
        }

        return false;
    }

} // end enoughDeviationToReplan


// Check if _flight_plans has changed compared with before:
bool PlanMonitor::checkIfPlansHaveChanged (const std::vector<aerialcore_msgs::FlightPlan>& _flight_plans) const {
    if (last_flight_plans_.size()==0) {
        return true;
    } else {
        if (last_flight_plans_.size()!=_flight_plans.size()) {
            return true;
        } else {
            for (int i=0; i<last_flight_plans_.size(); i++) {
                if (last_flight_plans_[i].uav_id!=_flight_plans[i].uav_id) {
                    return true;
                } else {
                    for (int j=0; last_flight_plans_[i].nodes.size(); j++) {
                        if (last_flight_plans_[i].nodes[j]!=_flight_plans[i].nodes[j]) {
                            return true;
                        }
                    }
                    for (int j=0; last_flight_plans_[i].edges.size(); j++) {
                        if (last_flight_plans_[i].edges[j]!=_flight_plans[i].edges[j]) {
                            return true;
                        }
                    }
                }
            }
        }
    }
    return false;
} // end checkIfPlansHaveChanged


void PlanMonitor::constructUAVs(const std::vector< std::tuple<float, float, int, int, int, int, int, int, bool, bool> >& _drone_info) {

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


int PlanMonitor::findUavIndexById(int _UAV_id) {
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
