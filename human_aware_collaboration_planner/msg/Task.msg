#type M/m   ->  Monitor
#type F/f   ->  MonitorUGV
#type I/i   ->  Inspect
#type A/a   ->  InspectPVArray
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
