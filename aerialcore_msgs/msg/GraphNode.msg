uint8 TYPE_PYLON = 0
uint8 TYPE_RECHARGE_LAND_STATION = 1
uint8 TYPE_REGULAR_LAND_STATION = 2
uint8 TYPE_UAV_INITIAL_POSITION = 3
uint8 TYPE_PASS_WP_AVOIDING_NO_FLY_ZONE = 4

uint8 type

int32[] connections_indexes # If pylon, indexes of other pylons to which the current pylon is connected by a wire.
int32 id                    # UAV id if TYPE_UAV_INITIAL_POSITION
float32 x                   # x location of the graph node.
float32 y                   # y location of the graph node.
float32 z                   # z location of the graph node.
float32 latitude            # latitude location of the graph node.
float32 longitude           # longitude location of the graph node.
float32 altitude            # altitude location of the graph node.