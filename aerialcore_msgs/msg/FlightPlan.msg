uint8 TYPE_TAKEOFF_WP = 0
uint8 TYPE_PASS_WP = 1
uint8 TYPE_LAND_WP = 2

Header header

uint16 uav_id
int32[] nodes
int32[] edges

geometry_msgs/PoseStamped[] poses   # ATTENTION: includes waypoints added to avoid flying above no-fly zones.
uint8[] type                        # Same size as poses