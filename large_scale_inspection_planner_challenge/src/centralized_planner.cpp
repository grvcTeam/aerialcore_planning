/**
 * AERIALCORE Project:
 *
 * Centralized planner.
 * 
 */

#include <centralized_planner.h>

namespace aerialcore {


// Brief Constructor
CentralizedPlanner::CentralizedPlanner() : path_planner_() {}


// Brief Destructor
CentralizedPlanner::~CentralizedPlanner() {}


std::vector<aerialcore_msgs::FlightPlan> CentralizedPlanner::getPlan(const std::vector<aerialcore_msgs::GraphNode>& _graph, const std::map< int, std::tuple<geometry_msgs::PoseStamped,float, float, int, int, int, int, int, bool, bool> >& _drone_info) {

    graph_.clear();
    UAVs_.clear();
    flight_plan_.clear();

    graph_ = _graph;

    // _drone_info it's a map of tuples. The keys are the drone ids, the values a tuple with 10 elements. The first in the tuple is the initial pose of the drone, the second it's the initial battery, and so on with all the elements in the "UAV" structure defined in the header file.
    for ( std::map< int, std::tuple<geometry_msgs::PoseStamped,float, float, int, int, int, int, int, bool, bool> >::const_iterator it = _drone_info.begin(); it != _drone_info.end(); it++ ) {
        UAV actual_UAV;
        actual_UAV.initial_pose = std::get<0>(it->second);
        actual_UAV.initial_battery = std::get<1>(it->second);
        actual_UAV.minimum_battery = std::get<2>(it->second);
        actual_UAV.time_until_fully_charged = std::get<3>(it->second);
        actual_UAV.time_max_flying = std::get<4>(it->second);
        actual_UAV.speed_xy = std::get<5>(it->second);
        actual_UAV.speed_z_down = std::get<6>(it->second);
        actual_UAV.speed_z_up = std::get<7>(it->second);
        actual_UAV.landed_or_flying_initially = std::get<8>(it->second);
        actual_UAV.recharging_initially = std::get<9>(it->second);
        UAVs_[it->first] = actual_UAV;
    }

} // end getPlan method


} // end namespace aerialcore
