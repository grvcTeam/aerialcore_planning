#mode STATIC                -> 0
#mode RECHARGE_WHEN_LANDED  -> 1
#mode RECHARGE_IN_BASE      -> 2
#mode RECHARGE_DISABLED     -> 3

#percentage keep            -> -1
#percentage set             -> [0, 1]

#Speed                      -> [0, 1]

uint8 mode
float32 percentage
float32 battery_increase
float32 battery_decrease
