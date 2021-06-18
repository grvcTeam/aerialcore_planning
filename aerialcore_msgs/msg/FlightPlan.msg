uint8 TYPE_TAKEOFF_WP = 0
uint8 TYPE_PASS_PYLON_WP = 1 # Pass waypoint extracted from a pylon of the graph.
uint8 TYPE_PASS_NFZ_WP = 2   # Pass waypoint added to avoid flying above a no-fly zone.
uint8 TYPE_LAND_WP = 3

Header header

uint16 uav_id
int32[] nodes

geometry_msgs/PoseStamped[] poses   # ATTENTION: includes waypoints added to avoid flying above no-fly zones.
uint8[] type                        # Same size as poses