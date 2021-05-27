uint8 TYPE_INSPECTION = 0
uint8 TYPE_NAVIGATION = 1
uint8 TYPE_TAKEOFF_AND_NAVIGATION = 2
uint8 TYPE_NAVIGATION_AND_LANDING = 3

uint8 type

int32 i     # Refered to the time cost and battery drop matrices.
int32 j     # Refered to the time cost and battery drop matrices.
int32 k     # Only different that -1 if TAKEOFF_AND_NAVIGATION (and NAVIGATION_AND_LANDING if UAV only can land in its), meaning that only that particular drone k can do that edge.
