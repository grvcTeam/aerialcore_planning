#type M/m   ->  Monitor
#type I/i   ->  Inspect
#type D/d   ->  Deliver
#type R/r   ->  Recharge
#type W/w   ->  Wait

string id
uint8 type
MonitorParams monitor
MonitorUGVParams monitor_ugv
InspectParams inspect
DeliverToolParams deliver
RechargeParams recharge
WaitParams wait
