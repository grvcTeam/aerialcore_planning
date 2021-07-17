/**
 * AERIALCORE Project:
 *
 * Parameter Estimator.
 * 
 */

#include <parameter_estimator.h>

#include <math.h>
#include <sstream>
#include <XmlRpcValue.h>
#include <fstream>
#include <curl/curl.h>

#define DEBUG       // UNCOMMENT FOR PRINTING VISUALIZATION OF RESULTS (DEBUG MODE)

#define FLYING_THRESHOLD 1  // Height threshold above which an UAV is considered to be flyinh (in meters). TODO: better way to know if flying?

namespace aerialcore {


// Brief Constructor
ParameterEstimator::ParameterEstimator() {
    // Read and construct parameters of the UAVs (from the yaml):
    std::map<std::string, std::string> drones;
    ros::param::get("drones", drones);              // Dictionary of the actual drones used in the simulation (key: UAV id, value: airframe type).
    if (drones.size() == 0) {
        ROS_ERROR("Parameter Estimator: error in the description of the UAVs (YAML file), no drones found. Are you sure you loaded to the server parameter the config YAML?");
        exit(EXIT_FAILURE);
    }
    for (std::map<std::string, std::string>::iterator it = drones.begin(); it != drones.end(); it++) {
        UAV new_uav;
        new_uav.id = stoi(it->first);
        new_uav.airframe_type = it->second;

        ros::param::get(it->second+"/time_delay_between_wps", new_uav.time_delay_between_wps);
        if (new_uav.airframe_type == "MULTICOPTER") {
            ros::param::get(it->second+"/takeoff_climb_speed", new_uav.takeoff_climb_speed);
            ros::param::get(it->second+"/landing_descend_speed", new_uav.landing_descend_speed);
            ros::param::get(it->second+"/hardcoded_takeoff_landing_height", new_uav.hardcoded_takeoff_landing_height);

        } else if (new_uav.airframe_type == "FIXED_WING") {
            ros::param::get(it->second+"/time_delay_landing", new_uav.time_delay_landing);

        } else if (new_uav.airframe_type == "VTOL") {
            ros::param::get(it->second+"/takeoff_climb_speed", new_uav.takeoff_climb_speed);
            ros::param::get(it->second+"/landing_descend_speed", new_uav.landing_descend_speed);
            ros::param::get(it->second+"/time_delay_start_transition", new_uav.time_delay_start_transition);
            ros::param::get(it->second+"/time_delay_end_transition", new_uav.time_delay_end_transition);
        }

        ros::param::get(it->second+"/time_max_flying", new_uav.time_max_flying);
        ros::param::get(it->second+"/speed_xy", new_uav.speed_xy);
        ros::param::get(it->second+"/speed_z_down", new_uav.speed_z_down);
        ros::param::get(it->second+"/speed_z_up", new_uav.speed_z_up);
        ros::param::get(it->second+"/minimum_battery", new_uav.minimum_battery);
        ros::param::get(it->second+"/time_until_fully_charged", new_uav.time_until_fully_charged);
        UAVs_.push_back(new_uav);
    }

    // map_origin_geo is the geographic coordinate origin of the cartesian coordinates. Loaded to the param server in the YAML config file.
    std::vector<double> map_origin_geo_vector;
    ros::param::get("map_origin_geo", map_origin_geo_vector);
    map_origin_geo_.latitude  = map_origin_geo_vector[0];
    map_origin_geo_.longitude = map_origin_geo_vector[1];
    map_origin_geo_.altitude  = map_origin_geo_vector[2];

    // Load the API ID of OpenWeatherMap:
    ros::param::get("openweathermap_appid", openweathermap_appid_);

    // Advertised services:
    set_wind_vector_srv_ = n_.advertiseService("parameter_estimator/set_wind_vector", &ParameterEstimator::setWindVectorServiceCallback, this);
}


// Brief Destructor
ParameterEstimator::~ParameterEstimator() {}


const std::map<int, std::map<int, float> >& ParameterEstimator::getDistanceCostMatrix() {
    distance_cost_matrix_mutex_.lock(); // Wait here until the attribute is fully created before the getter returns the reference.
    distance_cost_matrix_mutex_.unlock();
    return distance_cost_matrix_;
}


const std::map<int, std::map<int, std::map<int, float> > >& ParameterEstimator::getTimeCostMatrices() {
    time_cost_matrices_mutex_.lock();     // Wait here until the attribute is fully created before the getter returns the reference.
    time_cost_matrices_mutex_.unlock();
    return time_cost_matrices_;
}


const std::map<int, std::map<int, std::map<int, float> > >& ParameterEstimator::getBatteryDropMatrices() {
    battery_drop_matrices_mutex_.lock();  // Wait here until the attribute is fully created before the getter returns the reference.
    battery_drop_matrices_mutex_.unlock();
    return battery_drop_matrices_;
}


// Method called periodically in an external thread, located in the Mission Controller, that will update both the cost and battery drop matrices with the last prediction:
void ParameterEstimator::updateMatrices(const std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector<geometry_msgs::Polygon>& _no_fly_zones, const geometry_msgs::Polygon& _geofence, bool _recalculate_initial_UAV_points) {

    std::map<int, std::map<int, float> > new_distance_cost_matrix;
    std::map<int, std::map<int, Wps> > new_paths_matrix;
    std::map<int, std::map<int, std::map<int, float> > > new_time_cost_matrices;

    if (distance_cost_matrix_.size()==0 || _recalculate_initial_UAV_points) {  // Only enter the first time this method is called or if recalculating UAV points.

        // Construct the path_planner_:
        if (_geofence.points.size()>0 && _no_fly_zones.size()>0) {
            path_planner_ = grvc::PathPlanner(_no_fly_zones, _geofence);
        }

        // If recalculate initial UAVs (and there is already something in the matrices):
        if (_recalculate_initial_UAV_points && distance_cost_matrix_.size()>0) {

            distance_cost_matrix_mutex_.lock();
            new_distance_cost_matrix = distance_cost_matrix_;
            new_paths_matrix = paths_matrix_;
            distance_cost_matrix_mutex_.unlock();

            std::vector<int> indexes_to_erase;  // Build the vector of indexes to erase, which is formed from the previous existing initial UAVs in the matrices.
            for (std::map<int, std::map<int, float> >::iterator it = new_distance_cost_matrix.begin(); it != new_distance_cost_matrix.end(); it++) {
                if (_graph[it->first].type == aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION) {
                    indexes_to_erase.push_back(it->first);
                }
            }

            // Erase the previous initial UAVs row and columns from the new_distance_cost_matrix:
            for (int i=0; i<indexes_to_erase.size(); i++) {
                new_distance_cost_matrix.erase(indexes_to_erase[i]);
                for (std::map<int, std::map<int, float> >::iterator it_i = new_distance_cost_matrix.begin(); it_i != new_distance_cost_matrix.end(); it_i++) {
                    it_i->second.erase(indexes_to_erase[i]);
                }
            }

            // Erase the previous initial UAVs row and columns from the new_paths_matrix:
            for (int i=0; i<indexes_to_erase.size(); i++) {
                new_paths_matrix.erase(indexes_to_erase[i]);
                for (std::map<int, std::map<int, Wps> >::iterator it_i = new_paths_matrix.begin(); it_i != new_paths_matrix.end(); it_i++) {
                    it_i->second.erase(indexes_to_erase[i]);
                }
            }

            // We could also erase only the UAV initial positions from time_cost_matrices_, but is easier to don't do it.

            // The battery_drop_matrices_ need to be calculated entirely so no need to erase anything.
        }

        // Create the vector with the nodes, which has this order: UAV initial positions, regular land stations, recharging land stations and pylons.
        std::vector<int> uav_initial_positions_indexes, uav_regular_land_stations_indexes, uav_recharging_land_stations_indexes, pylons_indexes;
        for (int i=0; i<_graph.size(); i++) {
            if (_graph[i].type == aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION) {
                uav_initial_positions_indexes.push_back(i);
            } else if (_graph[i].type == aerialcore_msgs::GraphNode::TYPE_REGULAR_LAND_STATION) {
                uav_regular_land_stations_indexes.push_back(i);
            } else if (_graph[i].type == aerialcore_msgs::GraphNode::TYPE_RECHARGE_LAND_STATION) {
                uav_recharging_land_stations_indexes.push_back(i);
            } else if (_graph[i].type == aerialcore_msgs::GraphNode::TYPE_PYLON) {
                pylons_indexes.push_back(i);
            }
        }
        nodes_indexes_in_order_.clear();
        nodes_indexes_in_order_.insert(nodes_indexes_in_order_.end(), uav_initial_positions_indexes.begin(), uav_initial_positions_indexes.end());
        nodes_indexes_in_order_.insert(nodes_indexes_in_order_.end(), uav_regular_land_stations_indexes.begin(), uav_regular_land_stations_indexes.end());
        nodes_indexes_in_order_.insert(nodes_indexes_in_order_.end(), uav_recharging_land_stations_indexes.begin(), uav_recharging_land_stations_indexes.end());
        nodes_indexes_in_order_.insert(nodes_indexes_in_order_.end(), pylons_indexes.begin(), pylons_indexes.end());

        // Initialize the distance_cost_matrix with -1 in all elements:
        for (int i=0; i<nodes_indexes_in_order_.size(); i++) {
            for (int j=0; j<nodes_indexes_in_order_.size(); j++) {
                if (_recalculate_initial_UAV_points && distance_cost_matrix_.size()>0) {
                    if (_graph[ nodes_indexes_in_order_[i] ].type==aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION || _graph[ nodes_indexes_in_order_[j] ].type==aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION) {
                        new_distance_cost_matrix[ nodes_indexes_in_order_[i] ][ nodes_indexes_in_order_[j] ] = -1;
                    }       // Not change to -1 if recalculating but edge without UAV initial position
                } else {    // If not recalculate, evertything initialized to -1.
                    new_distance_cost_matrix[ nodes_indexes_in_order_[i] ][ nodes_indexes_in_order_[j] ] = -1;
                }
            }
        }

        // Call the path planner between initial UAV poses and landing stations to pylons, and between pylons:
        geometry_msgs::Point32 from_here;
        geometry_msgs::Point32 to_here;
        for (int i=0; i<nodes_indexes_in_order_.size(); i++) {
            if (_recalculate_initial_UAV_points && distance_cost_matrix_.size()>0 && _graph[ nodes_indexes_in_order_[i] ].type!=aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION) {
                continue;       // Not recalculate the edge if rebuild only the UAVs initial position and the edge doesn't involve a UAV initial position.
            }
            for (int j= i >= uav_initial_positions_indexes.size()+uav_regular_land_stations_indexes.size()+uav_recharging_land_stations_indexes.size() ? i+1 : uav_initial_positions_indexes.size()+uav_regular_land_stations_indexes.size()+uav_recharging_land_stations_indexes.size() ; j<nodes_indexes_in_order_.size(); j++) {
                from_here.x = _graph[ nodes_indexes_in_order_[i] ].x;
                from_here.y = _graph[ nodes_indexes_in_order_[i] ].y;
                from_here.z = _graph[ nodes_indexes_in_order_[i] ].z;
                to_here.x =   _graph[ nodes_indexes_in_order_[j] ].x;
                to_here.y =   _graph[ nodes_indexes_in_order_[j] ].y;
                to_here.z =   _graph[ nodes_indexes_in_order_[j] ].z;
                auto path = path_planner_.getPath(from_here, to_here);
                if (path.size() == 0) { // No path found.
                    new_distance_cost_matrix[ nodes_indexes_in_order_[i] ][ nodes_indexes_in_order_[j] ] = -1;
                    new_distance_cost_matrix[ nodes_indexes_in_order_[j] ][ nodes_indexes_in_order_[i] ] = -1;

                    Wps empty_wps;
                    new_paths_matrix[ nodes_indexes_in_order_[i] ][ nodes_indexes_in_order_[j] ] = empty_wps;
                    new_paths_matrix[ nodes_indexes_in_order_[j] ][ nodes_indexes_in_order_[i] ] = empty_wps;
                } else {
                    new_distance_cost_matrix[ nodes_indexes_in_order_[i] ][ nodes_indexes_in_order_[j] ] = path_planner_.getFlatDistance();   // TODO: maybe consider also height distances? I think it's better as it is.
                    new_distance_cost_matrix[ nodes_indexes_in_order_[j] ][ nodes_indexes_in_order_[i] ] = path_planner_.getFlatDistance();

                    Wps new_wps, new_wps_reversed;
                    new_wps.path = path;
                    path.insert(path.begin(), from_here);   // Path returned doesn' contain the first wp, but it will be convenient to have it.
                    for (int k=1; k<path.size(); k++) {
                        new_wps.angle.push_back( atan2(path[k].y-path[k-1].y, path[k].x-path[k-1].x) );
                        new_wps.distance.push_back( sqrt( pow(path[k].y-path[k-1].y,2) + pow(path[k].x-path[k-1].x,2) ) );
                    }
                    new_paths_matrix[ nodes_indexes_in_order_[i] ][ nodes_indexes_in_order_[j] ] = new_wps;
                    for (int k=path.size()-2; k>=0; k--) {
                        new_wps_reversed.path.push_back( path[k] );
                        new_wps_reversed.angle.push_back( M_PI + new_wps.angle[k] );
                        new_wps_reversed.distance.push_back( new_wps.distance[k] );
                    }
                    new_paths_matrix[ nodes_indexes_in_order_[j] ][ nodes_indexes_in_order_[i] ] = new_wps_reversed;

                }
            }
        }

        distance_cost_matrix_mutex_.lock();
        distance_cost_matrix_.clear();
        distance_cost_matrix_ = new_distance_cost_matrix;
        paths_matrix_.clear();
        paths_matrix_ = new_paths_matrix;
        distance_cost_matrix_mutex_.unlock();

        // For that distance_cost_matrix, create the time_cost_matrices for each UAV (depending on its speed):
        for (int k=0; k<UAVs_.size(); k++) {    // The time is different for each UAV because of the different speed.
            std::map<int, std::map<int, float> > new_time_cost_matrix;
            for (std::map<int, std::map<int, float> >::iterator it_i = new_distance_cost_matrix.begin(); it_i != new_distance_cost_matrix.end(); it_i++) {
                for (std::map<int, float>::iterator it_j = it_i->second.begin(); it_j != it_i->second.end(); it_j++) {
                    if (new_distance_cost_matrix[ it_i->first ][ it_j->first ]==-1) {
                        new_time_cost_matrix[ it_i->first ][ it_j->first ] = new_distance_cost_matrix[ it_i->first ][ it_j->first ];
                    } else {

                        if (UAVs_[k].airframe_type == "MULTICOPTER") {

                            new_time_cost_matrix[ it_i->first ][ it_j->first ] = timeCostMulticopter(UAVs_[k].id, it_i->first, it_j->first, new_distance_cost_matrix, new_paths_matrix, _graph);

                        } else if (UAVs_[k].airframe_type == "FIXED_WING") {

                            new_time_cost_matrix[ it_i->first ][ it_j->first ] = timeCostFixedWing(UAVs_[k].id, it_i->first, it_j->first, new_distance_cost_matrix, new_paths_matrix, _graph);

                        } else if (UAVs_[k].airframe_type == "VTOL") {

                            new_time_cost_matrix[ it_i->first ][ it_j->first ] = timeCostVTOL(UAVs_[k].id, it_i->first, it_j->first, new_distance_cost_matrix, new_paths_matrix, _graph);

                        }

                    }
                }
            }
            new_time_cost_matrices[ UAVs_[k].id ] = new_time_cost_matrix;
        }
        time_cost_matrices_mutex_.lock();
        time_cost_matrices_.clear();
        time_cost_matrices_ = new_time_cost_matrices;
        time_cost_matrices_mutex_.unlock();
    }   // End of building distance and time cost matrices the first time this method is called (or if needed to recalculate UAV poses).

    // Construct the battery_drop_matrices.
    time_cost_matrices_mutex_.lock();
    std::map<int, std::map<int, std::map<int, float> > > new_battery_drop_matrices;
    for (std::map<int, std::map<int, std::map<int, float> > >::iterator it_k = time_cost_matrices_.begin(); it_k != time_cost_matrices_.end(); it_k++) {
        std::map<int, std::map<int, float> > new_battery_drop_matrix;
        int uav_index = findUavIndexById(it_k->first);
        for (std::map<int, std::map<int, float> >::iterator it_i = it_k->second.begin(); it_i != it_k->second.end(); it_i++) {
            for (std::map<int, float>::iterator it_j = it_i->second.begin(); it_j != it_i->second.end(); it_j++) {
                if (it_j->second==-1) {
                    new_battery_drop_matrix[ it_i->first ][ it_j->first ] = it_j->second;
                } else {
                    if (UAVs_[uav_index].airframe_type == "MULTICOPTER") {

                        new_battery_drop_matrix[ it_i->first ][ it_j->first ] = batteryDropMulticopter(it_k->first, it_i->first, it_j->first, time_cost_matrices_);

                    } else if (UAVs_[uav_index].airframe_type == "FIXED_WING") {

                        new_battery_drop_matrix[ it_i->first ][ it_j->first ] = batteryDropFixedWing(it_k->first, it_i->first, it_j->first, time_cost_matrices_);

                    } else if (UAVs_[uav_index].airframe_type == "VTOL") {

                        new_battery_drop_matrix[ it_i->first ][ it_j->first ] = batteryDropVTOL(it_k->first, it_i->first, it_j->first, time_cost_matrices_);

                    }
                }
            }
        }
        new_battery_drop_matrices[ it_k->first ] = new_battery_drop_matrix;
    }
    time_cost_matrices_mutex_.unlock();
    battery_drop_matrices_mutex_.lock();
    battery_drop_matrices_.clear();
    battery_drop_matrices_ = new_battery_drop_matrices;
    battery_drop_matrices_mutex_.unlock();

# ifdef DEBUG
    printMatrices();
    getWindFromInternet();
# endif

}   // end updateMatrices


void ParameterEstimator::getWindFromInternet() {
    CURL *curl;
    CURLcode res;
    std::string readBuffer;

    curl = curl_easy_init();
    if(curl) {
        // Prepare the URL for cURL:
        std::string url = "https://api.openweathermap.org/data/2.5/weather?lat=";
        url.append(std::to_string(map_origin_geo_.latitude).c_str());
        url.append("&lon=");
        url.append(std::to_string(map_origin_geo_.longitude).c_str());
        url.append("&appid=");
        url.append(openweathermap_appid_.c_str());
        std::cout << url << std::endl;

        // Get string data to the readBuffer from the URL with cURL:
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);

        // std::cout << readBuffer << std::endl;
    }

    if (readBuffer.size() == 0) {
        ROS_ERROR("Parameter Estimator: OpenWeatherMap didn't answer. Returning empty wind vector (no wind).");
    } else {
        std::string wind_speed_search_pattern = "\"speed\":";       // 8 elements
        std::string wind_direction_deg_search_pattern = "\"deg\":"; // 6 elements
        std::string end_pattern = ",";

        // Find the place where the first wind direction and speed are:
        size_t wind_speed_found = readBuffer.find(wind_speed_search_pattern);
        size_t wind_speed_end_found = readBuffer.find(end_pattern, wind_speed_found+8);
        size_t wind_direction_deg_found = readBuffer.find(wind_direction_deg_search_pattern);
        size_t wind_direction_deg_end_found = readBuffer.find(end_pattern, wind_direction_deg_found+6);

        if (wind_speed_found==std::string::npos || wind_speed_end_found==std::string::npos || wind_direction_deg_found==std::string::npos || wind_direction_deg_end_found==std::string::npos) {
            ROS_ERROR("Parameter Estimator: parse error from OpenWeatherMap");
        }

        std::string wind_speed_string = readBuffer.substr(wind_speed_found+8, wind_speed_end_found-(wind_speed_found+8));
        std::string wind_direction_deg_string = readBuffer.substr(wind_direction_deg_found+6, wind_direction_deg_end_found-(wind_direction_deg_found+6));

// # ifdef DEBUG
        // std::cout << "wind_speed_found = "<< wind_speed_found << std::endl;
        // std::cout << "wind_speed_end_found = "<< wind_speed_end_found << std::endl;
        // std::cout << "wind_direction_deg_found = "<< wind_direction_deg_found << std::endl;
        // std::cout << "wind_direction_deg_end_found = "<< wind_direction_deg_end_found << std::endl;
        // std::cout << std::endl << "wind_speed_string = " << wind_speed_string << std::endl;
        // std::cout << "wind_direction_deg_string = " << wind_direction_deg_string << std::endl << std::endl;
// # endif

        std::string::size_type sz;

        std::stringstream wind_speed_stringstream, wind_direction_deg_stringstream;
        wind_speed_stringstream << wind_speed_string;
        wind_direction_deg_stringstream << wind_direction_deg_string;

        float wind_direction_deg;

        try {   // try to parse into float the wind strings...
            wind_speed_ = (float) std::stod ( wind_speed_stringstream.str() , &sz);
            wind_direction_deg = (float) std::stod ( wind_direction_deg_stringstream.str() , &sz);
        } catch (...) { // catch any exception
            ROS_ERROR("Parameter Estimator: error reading the parsed string from OpenWeatherMap");
            return;
        }

        // Wind direction is reported by the direction from which it originates (from which is COMING). For example, a north wind blows from the north to the south, with a direction angle of 0° (or 360°). A wind blowing from the east has a wind direction referred to as 90°, etc.
        wind_vector_.x = -wind_speed_*sin(wind_direction_deg*M_PI/180.0);
        wind_vector_.y = -wind_speed_*cos(wind_direction_deg*M_PI/180.0);
        wind_vector_.z = 0;

# ifdef DEBUG
        std::cout << "wind_speed_ = " << wind_speed_ << std::endl;
        std::cout << "wind_vector_.x = " << wind_vector_.x << std::endl;
        std::cout << "wind_vector_.y = " << wind_vector_.y << std::endl;
        std::cout << "wind_vector_.z = " << wind_vector_.z << std::endl << std::endl;
# endif
    }
}   // end getWindFromInternet


bool ParameterEstimator::setWindVectorServiceCallback(aerialcore_msgs::SetWindVector::Request& _req, aerialcore_msgs::SetWindVector::Response& _res) {
    // Wind direction is reported by the direction from which it originates (from which is COMING). For example, a north wind blows from the north to the south, with a direction angle of 0° (or 360°). A wind blowing from the east has a wind direction referred to as 90°, etc.
    wind_vector_.x = -_req.speed*sin(_req.direction_deg*M_PI/180.0);
    wind_vector_.y = -_req.speed*cos(_req.direction_deg*M_PI/180.0);
    wind_vector_.z = 0;
    wind_speed_ = _req.speed;

# ifdef DEBUG
    std::cout << "wind_speed_ = " << wind_speed_ << std::endl;
    std::cout << "wind_vector_.x = " << wind_vector_.x << std::endl;
    std::cout << "wind_vector_.y = " << wind_vector_.y << std::endl;
    std::cout << "wind_vector_.z = " << wind_vector_.z << std::endl << std::endl;
# endif

    _res.success=true;
    return true;
}   // end setWindVectorServiceCallback


void ParameterEstimator::printMatrices() {
    std::string distance_cost_matrix_string = "distance_cost_matrix: [\n";
    for (int i=0; i<nodes_indexes_in_order_.size(); i++) {
        distance_cost_matrix_string.append("  [");
        if (i==0) {
            distance_cost_matrix_string.append(std::to_string(-1.0).c_str());
            distance_cost_matrix_string.append(", ");
            for (int j=0; j<nodes_indexes_in_order_.size(); j++) {
                if (j>0) {
                    distance_cost_matrix_string.append(", ");
                }
                distance_cost_matrix_string.append(std::to_string( nodes_indexes_in_order_[j] ).c_str());
            }
            distance_cost_matrix_string.append("],\n  [");
        }
        distance_cost_matrix_string.append(std::to_string( nodes_indexes_in_order_[i] ).c_str());
        distance_cost_matrix_string.append(", ");
        for (int j=0; j<nodes_indexes_in_order_.size(); j++) {
            if (j>0) {
                distance_cost_matrix_string.append(", ");
            }
            distance_cost_matrix_mutex_.lock();
            distance_cost_matrix_string.append(std::to_string( distance_cost_matrix_[ nodes_indexes_in_order_[i] ][ nodes_indexes_in_order_[j] ]).c_str());
            distance_cost_matrix_mutex_.unlock();
        }
        distance_cost_matrix_string.append("],\n");
    }
    distance_cost_matrix_string.append("]");

    std::string time_cost_matrices_string = "time_cost_matrices: [\n";
    time_cost_matrices_mutex_.lock();
    for (std::map<int, std::map<int, std::map<int, float> > >::iterator it_k = time_cost_matrices_.begin(); it_k != time_cost_matrices_.end(); it_k++) {
        time_cost_matrices_string.append("  [ uav_id = ");
        time_cost_matrices_string.append(std::to_string( it_k->first ).c_str());
        time_cost_matrices_string.append("\n");
        for (int i=0; i<nodes_indexes_in_order_.size(); i++) {
            time_cost_matrices_string.append("    [");
            if (i==0) {
                time_cost_matrices_string.append(std::to_string(-1.0).c_str());
                time_cost_matrices_string.append(", ");
                for (int j=0; j<nodes_indexes_in_order_.size(); j++) {
                    if (j>0) {
                        time_cost_matrices_string.append(", ");
                    }
                    time_cost_matrices_string.append(std::to_string( nodes_indexes_in_order_[j] ).c_str());
                }
                time_cost_matrices_string.append("],\n    [");
            }
            time_cost_matrices_string.append(std::to_string( nodes_indexes_in_order_[i] ).c_str());
            time_cost_matrices_string.append(", ");
            for (int j=0; j<nodes_indexes_in_order_.size(); j++) {
                if (j>0) {
                    time_cost_matrices_string.append(", ");
                }
                time_cost_matrices_string.append(std::to_string( it_k->second[ nodes_indexes_in_order_[i] ][ nodes_indexes_in_order_[j] ]).c_str());
            }
            time_cost_matrices_string.append("],\n");
        }
        time_cost_matrices_string.append("  ],\n");
    }
    time_cost_matrices_string.append("]");
    time_cost_matrices_mutex_.unlock();

    std::string battery_drop_matrices_string = "battery_drop_matrices: [\n";
    battery_drop_matrices_mutex_.lock();
    for (std::map<int, std::map<int, std::map<int, float> > >::iterator it_k = battery_drop_matrices_.begin(); it_k != battery_drop_matrices_.end(); it_k++) {
        battery_drop_matrices_string.append("  [ uav_id = ");
        battery_drop_matrices_string.append(std::to_string( it_k->first ).c_str());
        battery_drop_matrices_string.append("\n");
        for (int i=0; i<nodes_indexes_in_order_.size(); i++) {
            battery_drop_matrices_string.append("    [");
            if (i==0) {
                battery_drop_matrices_string.append(std::to_string(-1.0).c_str());
                battery_drop_matrices_string.append(", ");
                for (int j=0; j<nodes_indexes_in_order_.size(); j++) {
                    if (j>0) {
                        battery_drop_matrices_string.append(", ");
                    }
                    battery_drop_matrices_string.append(std::to_string( nodes_indexes_in_order_[j] ).c_str());
                }
                battery_drop_matrices_string.append("],\n    [");
            }
            battery_drop_matrices_string.append(std::to_string( nodes_indexes_in_order_[i] ).c_str());
            battery_drop_matrices_string.append(", ");
            for (int j=0; j<nodes_indexes_in_order_.size(); j++) {
                if (j>0) {
                    battery_drop_matrices_string.append(", ");
                }
                battery_drop_matrices_string.append(std::to_string( it_k->second[ nodes_indexes_in_order_[i] ][ nodes_indexes_in_order_[j] ]).c_str());
            }
            battery_drop_matrices_string.append("],\n");
        }
        battery_drop_matrices_string.append("  ],\n");
    }
    battery_drop_matrices_string.append("]");
    battery_drop_matrices_mutex_.unlock();

    std::cout << distance_cost_matrix_string << std::endl << std::endl;
    std::cout << time_cost_matrices_string << std::endl << std::endl;
    std::cout << battery_drop_matrices_string << std::endl << std::endl;
}   // end printMatrices


size_t ParameterEstimator::writeCallback(void *_contents, size_t _size, size_t _nmemb, void *_userp) {
    ((std::string*)_userp)->append((char*)_contents, _size * _nmemb);
    return _size * _nmemb;
}   // end writeCallback


int ParameterEstimator::findUavIndexById(int _UAV_id) {
    int uav_index = -1;
    for (int i=0; i<UAVs_.size(); i++) {
        if (UAVs_[i].id == _UAV_id) {
            uav_index = i;
            break;
        }
    }
    return uav_index;
} // end findUavIndexById


float ParameterEstimator::timeCostMulticopter(int _uav_id, int _index_i, int _index_j, const std::map<int, std::map<int, float> >& _distance_cost_matrix, const std::map<int, std::map<int, Wps> >& _paths_matrix, const std::vector<aerialcore_msgs::GraphNode>& _graph) {
    float time_cost = 0;

    int uav_index = findUavIndexById(_uav_id);

    if (_graph[_index_i].type==aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION
        || _graph[_index_i].type==aerialcore_msgs::GraphNode::TYPE_REGULAR_LAND_STATION
        || _graph[_index_i].type==aerialcore_msgs::GraphNode::TYPE_RECHARGE_LAND_STATION) { // Takeoff edge with navigation for a multicopter.
        time_cost = _distance_cost_matrix.at(_index_i).at(_index_j) / UAVs_[uav_index].speed_xy
        + UAVs_[uav_index].time_delay_between_wps * (_paths_matrix.at(_index_i).at(_index_j).path.size() + 1);

        time_cost += _graph[_index_i].z > FLYING_THRESHOLD ? 0 : UAVs_[uav_index].hardcoded_takeoff_landing_height / UAVs_[uav_index].takeoff_climb_speed ;    // Flying if UAV higher than FLYING_THRESHOLD (in meters). TODO: better way to know if flying?
        time_cost += _graph[_index_j].z > UAVs_[uav_index].hardcoded_takeoff_landing_height ? (_graph[_index_j].z - UAVs_[uav_index].hardcoded_takeoff_landing_height)/UAVs_[uav_index].speed_z_up : (UAVs_[uav_index].hardcoded_takeoff_landing_height - _graph[_index_j].z)/UAVs_[uav_index].speed_z_down ;

    } else if (_graph[_index_j].type==aerialcore_msgs::GraphNode::TYPE_REGULAR_LAND_STATION
    || _graph[_index_j].type==aerialcore_msgs::GraphNode::TYPE_RECHARGE_LAND_STATION) {     // Landing edge with navigation for a multicopter.
        time_cost = UAVs_[uav_index].hardcoded_takeoff_landing_height / UAVs_[uav_index].landing_descend_speed
        + _distance_cost_matrix.at(_index_i).at(_index_j) / UAVs_[uav_index].speed_xy
        + UAVs_[uav_index].time_delay_between_wps * (_paths_matrix.at(_index_i).at(_index_j).path.size() + 1);

        time_cost += _graph[_index_i].z > UAVs_[uav_index].hardcoded_takeoff_landing_height ? (_graph[_index_i].z - UAVs_[uav_index].hardcoded_takeoff_landing_height)/UAVs_[uav_index].speed_z_down : (UAVs_[uav_index].hardcoded_takeoff_landing_height - _graph[_index_i].z)/UAVs_[uav_index].speed_z_up ;

    } else {                                                                                // Regular navigation or inspection edge for a multicopter.
        time_cost = _distance_cost_matrix.at(_index_i).at(_index_j) / UAVs_[uav_index].speed_xy
        + UAVs_[uav_index].time_delay_between_wps * _paths_matrix.at(_index_i).at(_index_j).path.size();
    }

    return time_cost;
}


float ParameterEstimator::timeCostFixedWing(int _uav_id, int _index_i, int _index_j, const std::map<int, std::map<int, float> >& _distance_cost_matrix, const std::map<int, std::map<int, Wps> >& _paths_matrix, const std::vector<aerialcore_msgs::GraphNode>& _graph) {
    float time_cost = 0;

    int uav_index = findUavIndexById(_uav_id);

    if (_graph[_index_j].type==aerialcore_msgs::GraphNode::TYPE_REGULAR_LAND_STATION
        || _graph[_index_j].type==aerialcore_msgs::GraphNode::TYPE_RECHARGE_LAND_STATION) { // Landing edge with navigation for a fixed wing.
        time_cost = _distance_cost_matrix.at(_index_i).at(_index_j) / UAVs_[uav_index].speed_xy
        + UAVs_[uav_index].time_delay_between_wps * (_paths_matrix.at(_index_i).at(_index_j).path.size() + 1)
        + UAVs_[uav_index].time_delay_landing;

    } else {                                                                                // Takeoff or regular navigation or inspection edge for a fixed wing.
        time_cost = _distance_cost_matrix.at(_index_i).at(_index_j) / UAVs_[uav_index].speed_xy
        + UAVs_[uav_index].time_delay_between_wps * _paths_matrix.at(_index_i).at(_index_j).path.size();
    }

    return time_cost;
}


float ParameterEstimator::timeCostVTOL(int _uav_id, int _index_i, int _index_j, const std::map<int, std::map<int, float> >& _distance_cost_matrix, const std::map<int, std::map<int, Wps> >& _paths_matrix, const std::vector<aerialcore_msgs::GraphNode>& _graph) {
    float time_cost = 0;

    int uav_index = findUavIndexById(_uav_id);

    if (_graph[_index_i].type==aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION
        || _graph[_index_i].type==aerialcore_msgs::GraphNode::TYPE_REGULAR_LAND_STATION
        || _graph[_index_i].type==aerialcore_msgs::GraphNode::TYPE_RECHARGE_LAND_STATION) { // Takeoff edge with navigation for a VTOL.
        time_cost = (_graph[_index_j].z-_graph[_index_i].z) / UAVs_[uav_index].takeoff_climb_speed
        + _distance_cost_matrix.at(_index_i).at(_index_j) / UAVs_[uav_index].speed_xy
        + UAVs_[uav_index].time_delay_between_wps * (_paths_matrix.at(_index_i).at(_index_j).path.size() + 1)
        + UAVs_[uav_index].time_delay_start_transition;

    } else if (_graph[_index_j].type==aerialcore_msgs::GraphNode::TYPE_REGULAR_LAND_STATION
    || _graph[_index_j].type==aerialcore_msgs::GraphNode::TYPE_RECHARGE_LAND_STATION) {     // Landing edge with navigation for a VTOL.
        time_cost = (_graph[_index_i].z-_graph[_index_j].z) / UAVs_[uav_index].landing_descend_speed
        + _distance_cost_matrix.at(_index_i).at(_index_j) / UAVs_[uav_index].speed_xy
        + UAVs_[uav_index].time_delay_between_wps * (_paths_matrix.at(_index_i).at(_index_j).path.size() + 1)
        + UAVs_[uav_index].time_delay_end_transition;

    } else {                                                                                // Regular navigation or inspection edge for a VTOL.
        time_cost = _distance_cost_matrix.at(_index_i).at(_index_j) / UAVs_[uav_index].speed_xy
        + UAVs_[uav_index].time_delay_between_wps * _paths_matrix.at(_index_i).at(_index_j).path.size();
    }

    return time_cost;
}


float ParameterEstimator::batteryDropMulticopter(int _uav_id, int _index_i, int _index_j, const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices) {
    return _time_cost_matrices.at(_uav_id).at(_index_i).at(_index_j) / UAVs_[ findUavIndexById(_uav_id) ].time_max_flying;
}


float ParameterEstimator::batteryDropFixedWing(int _uav_id, int _index_i, int _index_j, const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices) {
    return _time_cost_matrices.at(_uav_id).at(_index_i).at(_index_j) / UAVs_[ findUavIndexById(_uav_id) ].time_max_flying;
}


float ParameterEstimator::batteryDropVTOL(int _uav_id, int _index_i, int _index_j, const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices) {
    return _time_cost_matrices.at(_uav_id).at(_index_i).at(_index_j) / UAVs_[ findUavIndexById(_uav_id) ].time_max_flying;
}


} // end namespace aerialcore
