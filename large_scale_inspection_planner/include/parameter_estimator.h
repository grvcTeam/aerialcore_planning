/**
 * AERIALCORE Project:
 *
 * Parameter Estimator.
 *
 */

#ifndef PARAMETER_ESTIMATOR_H
#define PARAMETER_ESTIMATOR_H

#include <ros/ros.h>
#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <tuple>
#include <utility>
#include <mutex>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point32.h>
#include <geographic_msgs/GeoPoint.h>
#include <aerialcore_msgs/GraphNode.h>
#include <aerialcore_msgs/SetWindVector.h>

#include <path_planner.h>

namespace aerialcore {

/// ParameterEstimator class that works as interface
class ParameterEstimator {

public:
    ParameterEstimator();
    ~ParameterEstimator();

    // Method called periodically in an external thread, located in the Mission Controller, that will update both the cost and battery drop matrices with the last prediction:
    void updateMatrices(const std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector<geometry_msgs::Polygon>& _no_fly_zones, const geometry_msgs::Polygon& _geofence, bool _recalculate_initial_UAV_points, bool _update_wind);

    // Getters that the Mission Controller will use for the Planner and Plan Monitor:
    const std::map<int, std::map<int, float> >& getDistanceCostMatrix();
    const std::map<int, std::map<int, std::map<int, float> > >& getTimeCostMatrices();
    const std::map<int, std::map<int, std::map<int, float> > >& getBatteryDropMatrices();

    void printMatrices();

    float multicopterHoverPower(int _uav_id);                               // Watts output [W].
    float multicopterClimbPower(int _uav_id, float _climb_velocity);        // Watts output [W]. Climb velocity (m/s) should be >0.
    float multicopterDescentPower(int _uav_id, float _descent_velocity);    // Watts output [W]. Descent velocity (m/s) should be <0.
    float multicopterForwardPower(int _uav_id, float _forward_velocity);    // Watts output [W]. Forward velocity (m/s) relative to the air mass, in other words, it includes the wind.
    float multicopterForwardPower(int _uav_id, geometry_msgs::TwistStamped _forward_velocity);    // Watts output [W]. Forward velocity (m/s) NOT relative to the air mass (absolute).

private:
    // The following are the capacity and cost matrices. For them, each row and column are graph nodes and the elements between them are the edges.
    // The rows and columns are the following graph nodes: initial UAV pose, regular land stations, recharging land stations, and pylons.
    std::map<int, std::map<int, float> > distance_cost_matrix_;                    // Square symetrical matrix (maps by graph nodes indexes). The elements are the distance (in meters) to cover that edge, if the edge doesn't make sense or there is no connection possible then the value is -1.
    std::map<int, std::map<int, std::map<int, float> > > time_cost_matrices_;      // One square symetrical matrix for each UAV (map by id and by graph nodes indexes). The elements are the time (in seconds) to cover that edge, if the edge doesn't make sense or there is no connection possible then the value is -1.
    std::map<int, std::map<int, std::map<int, float> > > battery_drop_matrices_;   // One square non-symetrical matrix for each UAV (map by id and by graph nodes indexes). The elements are the battery drop (per unit, not %) to cover that edge, if the edge doesn't make sense or there is no connection possible then the value is -1.

    struct WpsVectors {             // Vectors between the wps of the path to go from one graph node to the other. Will be useful to correct the battery drop with the wind.
        std::vector<geometry_msgs::Point32> wp_vector;  // Vector from one wp to the next one (takes into account both the initial and final points of the path).
        std::vector<float> distance;                    // Distance to go to that wp. All 3 vectors with the same size. Equal size of both wp_vector and the path from the Path Planner.
    };
    std::map<int, std::map<int, WpsVectors> > paths_matrix_;   // Matrix similar to distance_cost_matrix_, but stores not only the distance, but also the path to go from one node to the other.

    std::vector<int> nodes_indexes_in_order_;

    grvc::PathPlanner path_planner_;

    std::mutex distance_cost_matrix_mutex_;
    std::mutex time_cost_matrices_mutex_;
    std::mutex battery_drop_matrices_mutex_;

    struct UAV {
        std::string airframe_type;  // Not the one from the roslaunch (PX4), but the one in the yaml.

        float speed_xy;         // Maximum horizontal velocity (m/s) of this specific UAV (in AUTO mode, if higher speeds are commanded in a mission they will be capped to this velocity).
        float speed_z_down;     // Maximum vertical descent velocity (m/s) of this specific UAV (in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL)).
        float speed_z_up;       // Maximum vertical ascent velocity (m/s) of this specific UAV (in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL)).

        float minimum_battery = 0.2;    // Battery in parts per unit (not percentage or %) considered fully discharged. LiPo batteries should never discharge to less than 20% or else the life span (number of charge/discharge cycles) will be dramatically reduced.

        float takeoff_climb_speed;              // Climb speed (m/s) during takeoff. Only for multicopter and VTOL.
        float landing_descend_speed;            // Descend speed (m/s) during takeoff. Only for multicopter and VTOL.
        int time_delay_between_wps;             // Each time the UAV pass by a waypoint, it slowsdown and have to maneuver a little. This is the delay estimation of that.
        int hardcoded_takeoff_landing_height;   // Only multicopters in mission flight mode. Takeoff and land to a hardcoded height (in meters).
        int time_delay_landing;                 // Only for fixed wing. Estimation of how much time it takes to land a fixed wing.
        int time_delay_start_transition;        // Only for VTOL. Time delay because of the transition from MC to VTOL modes.
        int time_delay_end_transition;          // Only for VTOL. Time delay because of the transition from VTOL to MC modes.

        int time_until_fully_charged;   // Used for battery charge time estimation, time expected to charge completely the batteries in the charging pad from discharge state.

        int time_max_flying;  // Used for battery drop estimation, this is the estimated maximum flying time (in seconds) of this specific UAV before the drone runs out of battery.

        int id;

        // --------------------------------------------------------
        //  Battery drop parameters (from Alvaro Caballero's work):
        // --------------------------------------------------------
        // Mass and geometry:
        float m;      // total mass                 [kg]
        int   nr;     // number of rotors           [-]
        int   b;      // number of rotor blades     [-]
        float R;      // rotor radius               [m]
        float c;      // blade chord (x=0.7*R)      [m]
        // Aerodynamics:
        float Cl;     // section lift coefficient   [-]
        float Cd;     // section drag coefficient   [-]
        float kappa;  // induced power factor       [-]
        float eta;    // energy efficiency          [-]
        float K_mu;   // P0 numerical constant      [-]
        float f;      // equivalent flat-plate area [m^2]
        // Battery:
        float joules; // Capacity of the battery    [J]
    };
    std::vector<UAV> UAVs_;

    geographic_msgs::GeoPoint map_origin_geo_;
    std::string openweathermap_appid_;

    static size_t writeCallback(void *_contents, size_t _size, size_t _nmemb, void *_userp);
    int  findUavIndexById(int _UAV_id);

    float wind_speed_;                      // Norm of the wind vector (m/s).
    geometry_msgs::Point32 wind_vector_;    // ENU: x positive is East, y positive is North direction and z positive is up. Norm of the vector is the wind speed in m/s.
    float air_density_; // [kg/m^3]
    float temperature_; // [K]
    float pressure_;    // [hPa]
    void getCurrentWeatherFromInternet();

    float gravity_;     // [m/s^2]

    ros::NodeHandle n_;
    bool setWindVectorServiceCallback(aerialcore_msgs::SetWindVector::Request& _req, aerialcore_msgs::SetWindVector::Response& _res);
    ros::ServiceServer set_wind_vector_srv_;

    float timeCostMulticopter(int _uav_id, int _index_i, int _index_j, const std::map<int, std::map<int, float> >& _distance_cost_matrix, const std::map<int, std::map<int, WpsVectors> >& _paths_matrix, const std::vector<aerialcore_msgs::GraphNode>& _graph);
    float timeCostFixedWing(int _uav_id, int _index_i, int _index_j, const std::map<int, std::map<int, float> >& _distance_cost_matrix, const std::map<int, std::map<int, WpsVectors> >& _paths_matrix, const std::vector<aerialcore_msgs::GraphNode>& _graph);
    float timeCostVTOL(int _uav_id, int _index_i, int _index_j, const std::map<int, std::map<int, float> >& _distance_cost_matrix, const std::map<int, std::map<int, WpsVectors> >& _paths_matrix, const std::vector<aerialcore_msgs::GraphNode>& _graph);

    float batteryDropMulticopter(int _uav_id, int _index_i, int _index_j, const std::map<int, std::map<int, float> >& _distance_cost_matrix, const std::map<int, std::map<int, WpsVectors> >& _paths_matrix, const std::vector<aerialcore_msgs::GraphNode>& _graph);
    float batteryDropFixedWing(int _uav_id, int _index_i, int _index_j, const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices);
    float batteryDropVTOL(int _uav_id, int _index_i, int _index_j, const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices);

};  // end ParameterEstimator class

}   // end namespace aerialcore

#endif  // PARAMETER_ESTIMATOR_H