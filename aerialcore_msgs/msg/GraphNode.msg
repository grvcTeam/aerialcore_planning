uint8 TYPE_PYLON = 0
uint8 TYPE_RECHARGE_LAND_STATION = 1
uint8 TYPE_REGULAR_LAND_STATION = 2
uint8 TYPE_UAV_INITIAL_POSITION = 3
uint8 TYPE_NO_FLY_ZONE = 4               # Can be any pylon, land station or UAV initial position, but once categorized like this it will be ignored.

uint8 type

int32[] connections_indexes # If pylon, indexes of other pylons to which the current pylon is connected by a wire.
int32 id                    # UAV id if TYPE_UAV_INITIAL_POSITION
float32 x                   # x location of the graph node.
float32 y                   # y location of the graph node.
float32 z                   # desired height of the UAV above the ground in the location of the graph node.
float32 latitude            # latitude location of the graph node.
float32 longitude           # longitude location of the graph node.
float32 altitude            # elevation (altitude of the ground) in the location of the graph node.