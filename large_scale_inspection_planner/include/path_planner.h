/**
 * MULTIDRONE and AERIAL-CORE Projects:
 *
 * Path Planner.
 *
 *
 * This library can optionally use the following extra software for additional features (all with MIT Licenses):
 *  - "ajnisbet/opentopodata", version 1.5.0. Open Topo Data is a REST API server for your elevation data. Copyright (c) 2020 Andrew Nisbet.
 *  - "nlohmann/json" library, version 3.9.1. JSON parser used to extract the elevation data from Open Topo Data localhost responses. Copyright (c) Copyright (c) 2013-2021 Niels Lohmann.
 *  - "lava/matplotlib-cpp". Plotting library that works by wrapping the popular python plotting library matplotlib. Needs a working python installation. Copyright (c) 2014 Benno Evers.
 * 
 * MIT License 
 * 
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 * 
 */

#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include <utility>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <geographic_msgs/GeoPoint.h>

#include <geographic_to_cartesian.h>

namespace multidrone {

enum struct ResultLastPath {OK, ERROR_INI_AND_END_OUTSIDE_MAP, ERROR_INI_OUTSIDE_MAP, ERROR_END_OUTSIDE_MAP, ERROR_INI_AND_END_OUTSIDE_GEOFENCE, ERROR_INI_OUTSIDE_GEOFENCE, ERROR_END_OUTSIDE_GEOFENCE, ERROR_INI_AND_END_INSIDE_OBSTACLE_POLYGON, ERROR_INI_INSIDE_OBSTACLE_POLYGON, ERROR_END_INSIDE_OBSTACLE_POLYGON, ERROR_INI_AND_END_INSIDE_GRID_OBSTACLE_LOW_RES, ERROR_INI_INSIDE_GRID_OBSTACLE_LOW_RES, ERROR_END_INSIDE_GRID_OBSTACLE_LOW_RES, ERROR_PATH_NOT_POSSIBLE, UNINITIATED};

/// PathPlanner class that works as interface
class PathPlanner {

public:
    PathPlanner();    // Brief Constructor for the simplest case of path planning: return the final point, straight line.
    PathPlanner(const std::vector< std::vector<bool> >& _no_fly_zones, double _min_x, double _max_x, double _min_y, double _max_y);  // Constructor that receives directly the no_fly_zones (rectangular boolean matrix) calculated out of the class. _min_x, _max_x, _min_y, _max_y are the x-y and coordinates of the limits of the grid.
    PathPlanner(const std::vector<geometry_msgs::Polygon>& _obstacle_polygon_vector_cartesian, const geometry_msgs::Polygon& _geofence_cartesian, unsigned int _max_grid_side = 100);  // Constructor that receives the obstacles list and geofence directly in a polygon vector. Polygons must start and end with the same point (closed polygon).
    PathPlanner(const std::vector< std::vector<geographic_msgs::GeoPoint> >& _obstacle_polygon_vector_geo, const std::vector<geographic_msgs::GeoPoint>& _geofence_polygon_geo, unsigned int _max_grid_side = 100);  // Constructor that receives the obstacles list and geofence directly in a vector of geographic coordinates (geographic polygons). Polygons must start and end with the same point (closed polygon).

    ~PathPlanner() {}

    // The input of getPath are the initial and final points in cartesian coordinates. IMPORTANT: the origin of coordinates of these points and the origin of coordinates of the obstacles (no-fly zone matrix) must be the same.
    // Also, the method has optional bool arguments:
    //      _movement_pattern:  if 0 (default) the movement of the agent can be in any direction, and if 1 the agent can only move in angles multiple of 45º (faster to compute if the grid is big, doesn't do visibility-loops).

    std::vector<geometry_msgs::PointStamped> getPath(const geometry_msgs::PointStamped& _initial_point_stamped, const geometry_msgs::PointStamped& _final_point_stamped, bool _movement_pattern=0);   // Method that returns a feasible path calculated with A* algorithm. All the waypoints at the height (z) of the final point (CAUTION).
    std::vector<geometry_msgs::Point32>      getPath(const geometry_msgs::Point32& _initial_point, const geometry_msgs::Point32& _final_point, bool _movement_pattern=0);                             // getPath but overloaded using Point32 instead of PointStamped.
    std::vector<geometry_msgs::Point>        getPath(const geometry_msgs::Point& _initial_point, const geometry_msgs::Point& _final_point, bool _movement_pattern=0);                                 // getPath but overloaded using Point instead of PointStamped.
    std::vector<geographic_msgs::GeoPoint>   getPath(const geographic_msgs::GeoPoint& _initial_geopoint, const geographic_msgs::GeoPoint& _final_geopoint, bool _movement_pattern=0);                 // getPath but overloaded using GeoPoint.

    std::vector<geometry_msgs::PointStamped> getPathWithTimePredictions(const geometry_msgs::PointStamped& _initial_point_stamped, const geometry_msgs::PointStamped& _final_point_stamped, int _full_speed_xy, int _full_speed_z_down, int _full_speed_z_up, bool _movement_pattern=0);                 // "getPath" method that estimates the time in each waypoint given the speed of the drone: A* algorithm implementation that returns a path for one robot from its initial position to the end position.
    std::vector<geometry_msgs::PointStamped> getPathWithTimePredictionsAndInitialPoint(const geometry_msgs::PointStamped& _initial_point_stamped, const geometry_msgs::PointStamped& _final_point_stamped, int _full_speed_xy, int _full_speed_z_down, int _full_speed_z_up, bool _movement_pattern=0);  // The same as "getPathWithTimePredictions" but inserting the initial point in the trajectory.

    std::vector<geometry_msgs::PointStamped> getPathCorrectingHeightFromOrigin(const geometry_msgs::PointStamped& _initial_point, const geometry_msgs::PointStamped& _final_point, bool _movement_pattern=0);   // getPath for PointStamped that gets in the points the desired height of the UAV from the ground, and corrects that height from the ground origin with the elevation variations of the irregular ground (communicating with a topographic REST API server).
    std::vector<geographic_msgs::GeoPoint>   getPathCorrectingHeightFromOrigin(const geographic_msgs::GeoPoint& _initial_point, const geographic_msgs::GeoPoint& _final_point, bool _movement_pattern=0);       // getPath for GeoPoint     that gets in the points the desired height of the UAV from the ground, and corrects that height from the ground origin with the elevation variations of the irregular ground (communicating with a topographic REST API server).
    std::vector<geographic_msgs::GeoPoint>   getPathWithAltitude(const geographic_msgs::GeoPoint& _initial_point, const geographic_msgs::GeoPoint& _final_point, bool _movement_pattern=0);                         // getPath for GeoPoint     that gets in the points the desired height of the UAV from the ground, and has as output the absolute altitude of the waypoints from the sea level.

    double getDistance()     const { return path_distance_; }        // Getter that returns path_distance_.
    double getFlatDistance() const { return path_flat_distance_; }   // Getter that returns path_flat_distance_.

    // Return true if using trivial path planner, false if not:
    bool getTrivialPathPlannerOrNot() const { return trivial_path_planner_; }

    ResultLastPath getResultLastPath() const { return result_last_path_; };

    bool checkIfPointInsidePolygon(const std::vector<geometry_msgs::Point32>& _polygon, const geometry_msgs::Point32& _test_point) const;
    bool checkIfPointInsidePolygon(const std::vector<geometry_msgs::Point32>& _polygon, const geometry_msgs::PointStamped& _test_point_stamped) const;
    bool checkIfPointInsidePolygon(const geometry_msgs::Polygon& _polygon, const geometry_msgs::Point32& _test_point) const;
    bool checkIfPointInsidePolygon(const std::vector<geographic_msgs::GeoPoint>& _polygon, const geographic_msgs::GeoPoint& _test_geopoint) const;

    bool checkIfPointInsideGeofence (const geometry_msgs::Point32& _test_point) const;
    bool checkIfPointInsideGeofence (const geometry_msgs::PointStamped& _test_point_stamped) const;
    bool checkIfPointInsideGeofence (const geographic_msgs::GeoPoint& _test_geopoint) const;

    bool checkIfPointInsideObstacles(const geometry_msgs::Point32& _test_point) const;
    bool checkIfPointInsideObstacles(const geometry_msgs::PointStamped& _test_point_stamped) const;
    bool checkIfPointInsideObstacles(const geographic_msgs::GeoPoint& _test_geopoint) const;

    // Return true if the test point is inside the geofence, but outside of any obstacle:
    bool checkIfPointIsValid(const geometry_msgs::Point32& _test_point) const;
    bool checkIfPointIsValid(const geometry_msgs::PointStamped& _test_point_stamped) const;
    bool checkIfPointIsValid(const geographic_msgs::GeoPoint& _test_geopoint) const;

    bool checkIfTwoPointsAreVisible(const geometry_msgs::Point32& _initial_point, const geometry_msgs::Point32& _final_point) const;
    bool checkIfTwoPointsAreVisible(const geometry_msgs::PointStamped& _initial_point_stamped, const geometry_msgs::PointStamped& _final_point_stamped) const;
    bool checkIfTwoPointsAreVisible(const geographic_msgs::GeoPoint& _initial_geopoint, const geographic_msgs::GeoPoint& _final_geopoint) const;

private:

    void polygonConstructorFunction(const std::vector<geometry_msgs::Polygon>& _obstacle_polygon_vector_cartesian, const geometry_msgs::Polygon& _geofence_polygon_cartesian, unsigned int _max_grid_side);

    std::vector<float> getElevations(const std::vector<geographic_msgs::GeoPoint>& _geopoints);
    static size_t writeCallback(void *contents, size_t size, size_t nmemb, void *userp);

    // Struct that will be initialized for each cell explored in the A* algorithm. The algorithm will construct a map of "CellInfo" (with a cell identifier as key) in order to reach the solution.
    struct CellInfo {
    public:
        unsigned int x;             // x coordinate of the cell in the grid (origin of x, x=0, is in the grid is on the left)
        unsigned int y;             // y coordinate of the cell in the grid (origin of y, y=0, is in the grid is on the top)
        unsigned int came_from;     // father of the cell (the one that minimize the cost if the cell is closed)
        unsigned int g_score = std::numeric_limits<unsigned int>::max();    // cost from the origin to the cell. Initialized at the beginning to "infinity" (maximum value possible for unsigned int) for each cell.
        unsigned int f_score = std::numeric_limits<unsigned int>::max();    // g_scores + heuristic cost from the cell to the goal position. Initialized at the beginning to "infinity" (maximum value possible for unsigned int) for each cell.
        bool closed = false;        // cell closed true if is fully studied
    };

    std::vector< std::pair<double,double> > calculateIntersectionsOfSegmentWithGrid (double first_absolute_point_of_segment_x, double first_absolute_point_of_segment_y, double last_absolute_point_of_segment_x, double last_absolute_point_of_segment_y) const;
    void fillCellsWithObstacles (const std::vector< std::pair<double,double> >& points_intersections_of_segment_with_grid);
    bool checkCollisionByVisibility (const std::vector< std::pair<double,double> >& points_intersections_of_segment_with_grid) const;

    std::vector< std::vector<bool> > no_fly_zones_;   // Rectangular grid needed for the path planner algorithm. Elements with value of 0 (false) are part of the free space, and elements with value of 1 (true) contains obstacles.

    std::vector<geometry_msgs::Polygon> no_fly_zones_cartesian_;  // Vector of polygons (vector of point32) that represent the obstacles. Polygons MUST be CLOSED, meaning that the first and end points are coincident (the same).
    geometry_msgs::Polygon              geofence_cartesian_;      // Polygon (vector of point32) that represent the geofence or limits of the fly area. Polygons MUST be CLOSED, meaning that the first and end points are coincident (the same).

    double x_cell_width_;    // Width in meters of each cell in the x axis.
    double y_cell_width_;    // Width in meters of each cell in the y axis.

    double path_distance_      = std::numeric_limits<double>::max();   // Distance sum of norms between waypoints. Initialized to "infinity" (maximum value possible for double).
    double path_flat_distance_ = std::numeric_limits<double>::max();   // Distance sum of norms between waypoints supposed flat path. Initialized to "infinity" (maximum value possible for double).

    // Minimum and maximum values x and y in Cartesian coordinates (initialized to zero):
    double min_x_ = 0;
    double max_x_ = 0;
    double min_y_ = 0;
    double max_y_ = 0;

    geographic_msgs::GeoPoint arbitrary_origin_geo_;    // This class internally works in Cartesian coordinates, so if points given in geographic coordinates, pass them to Cartesian to work internally in Cartesian. The origin will be the first geopoint received. Before returning the path solution pass it to geographic again.
    bool arbitrary_origin_geo_exist_ = false;

    ResultLastPath result_last_path_ = ResultLastPath::UNINITIATED;

    bool trivial_path_planner_ = false;

};  // end PathPlanner class

}   // end namespace multidrone

#endif  // PATH_PLANNER_H