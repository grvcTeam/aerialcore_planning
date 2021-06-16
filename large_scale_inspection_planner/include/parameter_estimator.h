/**
 * AERIALCORE Project:
 *
 * Parameter Estimator.
 *
 */

#ifndef PARAMETER_ESTIMATOR_H
#define PARAMETER_ESTIMATOR_H

#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <tuple>
#include <utility>
#include <mutex>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <geographic_msgs/GeoPoint.h>
#include <aerialcore_msgs/GraphNode.h>

#include <path_planner.h>

namespace aerialcore {

/// ParameterEstimator class that works as interface
class ParameterEstimator {

public:
    ParameterEstimator();
    ~ParameterEstimator();

    // Method called periodically in an external thread, located in the Mission Controller, that will update both the cost and battery drop matrices with the last prediction:
    void updateMatrices(const std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector<geometry_msgs::Polygon>& _no_fly_zones, const geometry_msgs::Polygon& _geofence /* poses, batteries, plan, wind sensor?*/);

    // Getters that the Mission Controller will use for the Planner and Plan Monitor:
    const std::map<int, std::map<int, float> >& getDistanceCostMatrix();
    const std::map<int, std::map<int, std::map<int, float> > >& getTimeCostMatrices();
    const std::map<int, std::map<int, std::map<int, float> > >& getBatteryDropMatrices();

private:
    // The following are the capacity and cost matrices. For them, each row and column are graph nodes and the elements between them are the edges.
    // The rows and columns are the following graph nodes: initial UAV pose, regular land stations, recharging land stations, and pylons.
    std::map<int, std::map<int, float> > distance_cost_matrix_;                    // Square symetrical matrix (maps by graph nodes indexes). The elements are the distance (in meters) to cover that edge, if the edge doesn't make sense or there is no connection possible then the value is -1.
    std::map<int, std::map<int, std::map<int, float> > > time_cost_matrices_;      // One square symetrical matrix for each UAV (map by id and by graph nodes indexes). The elements are the time (in seconds) to cover that edge, if the edge doesn't make sense or there is no connection possible then the value is -1.
    std::map<int, std::map<int, std::map<int, float> > > battery_drop_matrices_;   // One square non-symetrical matrix for each UAV (map by id and by graph nodes indexes). The elements are the battery drop (per unit, not %) to cover that edge, if the edge doesn't make sense or there is no connection possible then the value is -1.

    std::vector<float> nodes_indexes_in_order_;

    grvc::PathPlanner path_planner_;

    std::mutex distance_cost_matrix_mutex_;
    std::mutex time_cost_matrices_mutex_;
    std::mutex battery_drop_matrices_mutex_;

    std::string distance_cost_matrix_yaml_path_;
    bool construct_distance_cost_matrix_ = true;        // Parameter estimator constructs the distance_cost_matrix from the graph and export it to a default yaml file (if true), or swiftly read the last default yaml file of the graph (if false).

    struct UAV {
        std::string airframe_type;  // Not the one from the roslaunch (PX4), but the one in the yaml.

        float speed_xy;         // Maximum horizontal velocity (m/s) of this specific UAV (in AUTO mode, if higher speeds are commanded in a mission they will be capped to this velocity).
        float speed_z_down;     // Maximum vertical descent velocity (m/s) of this specific UAV (in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL)).
        float speed_z_up;       // Maximum vertical ascent velocity (m/s) of this specific UAV (in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL)).

        float minimum_battery = 0.2;    // Battery in parts per unit (not percentage or %) considered fully discharged. LiPo batteries should never discharge to less than 20% or else the life span (number of charge/discharge cycles) will be dramatically reduced.
        int time_until_fully_charged;   // Used for battery charge time estimation, time expected to charge completely the batteries in the charging pad from discharge state.

        int time_max_flying;  // Used for battery drop estimation, this is the estimated maximum flying time (in seconds) of this specific UAV before the drone runs out of battery.

        int id;
    };
    std::vector<UAV> UAVs_;

    geometry_msgs::Point32 wind_vector_;    // ENU: x positive is East, y positive is North direction and z positive is up. Module of the vector is the wind speed in m/s.
    float wind_speed_;                      // Module of the wind vector (m/s).

    void getWindFromInternet();
    static size_t writeCallback(void *contents, size_t size, size_t nmemb, void *userp);

    float batteryDropMulticopter(int _uav_id, int _index_i, int _index_j);
    float batteryDropFixedWing(int _uav_id, int _index_i, int _index_j);
    float batteryDropVTOL(int _uav_id, int _index_i, int _index_j);

};  // end ParameterEstimator class

}   // end namespace aerialcore

#endif  // PARAMETER_ESTIMATOR_H