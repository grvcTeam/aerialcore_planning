#!/usr/bin/env python
from aerialcore_msgs.msg import GraphNode
from aerialcore_msgs.srv import DoSpecificSupervision, DoSpecificSupervisionRequest, StartSupervising, StartSupervisingRequest, StopSupervising, StopSupervisingRequest
from std_srvs.srv import Trigger, TriggerRequest
import rospkg
import rospy
import time
from os import system
import signal
import sys


def main_menu():
    
    print "\nMain menu of the pydashboard. Please choose the option to send:"
    print "1. Start supervising"
    print "2. Stop supervising"
    print "3. Do specific supervision"
    print "4. Do continuous supervision"

    selected = raw_input(" >> ")
    system("clear")
    if selected == "1":
        start_supervising_menu()
    elif selected == "2":
        stop_supervising_menu()
    elif selected == "3":
        do_specific_supervision_menu()
    elif selected == "4":
        do_continuous_supervision_menu()

    else:
        system("clear")
        print "Not a valid option."


# 1. Start supervising:
def start_supervising_menu():
    system("clear")
    print "Start supervising selected."
    start_supervising_request = StartSupervisingRequest()
    try:
        print start_supervising_client.call(start_supervising_request)
    except rospy.ServiceException, e:
        print "\nService call failed: %s"%e


# 2. Stop supervising:
def stop_supervising_menu():
    system("clear")
    print "Stop supervising selected."
    stop_supervising_request = StopSupervisingRequest()
    try:
        print stop_supervising_client.call(stop_supervising_request)
    except rospy.ServiceException, e:
        print "\nService call failed: %s"%e


# 3. Do specific supervision:
def do_specific_supervision_menu():
    system("clear")
    print "Do specific supervision."
    do_specific_supervision_request = DoSpecificSupervisionRequest()
    graph_node_struct_1 = GraphNode()
    graph_node_struct_1.type = GraphNode.TYPE_ELECTRIC_PILAR
    graph_node_struct_1.x = 50
    graph_node_struct_1.y = -10
    graph_node_struct_1.connections_indexes.append(1)
    do_specific_supervision_request.specific_subgraph.append(graph_node_struct_1)
    graph_node_struct_2 = GraphNode()
    graph_node_struct_2.type = GraphNode.TYPE_ELECTRIC_PILAR
    graph_node_struct_2.x = -50
    graph_node_struct_2.y = -10
    graph_node_struct_2.connections_indexes.append(0)
    do_specific_supervision_request.specific_subgraph.append(graph_node_struct_2)
    graph_node_struct_2 = GraphNode()
    graph_node_struct_2.type = GraphNode.TYPE_RECHARGE_LAND_STATION
    graph_node_struct_2.x = 5
    graph_node_struct_2.y = 0
    do_specific_supervision_request.specific_subgraph.append(graph_node_struct_2)
    graph_node_struct_4 = GraphNode()
    graph_node_struct_4.type = GraphNode.TYPE_REGULAR_LAND_STATION
    graph_node_struct_4.x = -5
    graph_node_struct_4.y = 0
    do_specific_supervision_request.specific_subgraph.append(graph_node_struct_4)
    try:
        print do_specific_supervision_client.call(do_specific_supervision_request)
    except rospy.ServiceException, e:
        print "\nService call failed: %s"%e


# 4. Do continuous supervision:
def do_continuous_supervision_menu():
    system("clear")
    print "Do continuous supervision selected."
    try:
        print do_continuous_supervision_client.call()
    except rospy.ServiceException, e:
        print "\nService call failed: %s"%e


# Finish the execution directly when Ctrl+C is pressed (signal.SIGINT received), without escalating to SIGTERM.
def signal_handler(sig, frame):
    print('Ctrl+C pressed, signal.SIGINT received.')
    sys.exit(0)


if __name__ == "__main__":
    rospy.init_node('pydashboard', anonymous=True)
    start_supervising_client = rospy.ServiceProxy('mission_controller/start_supervising',StartSupervising)
    stop_supervising_client = rospy.ServiceProxy('mission_controller/stop_supervising',StopSupervising)
    do_specific_supervision_client = rospy.ServiceProxy('mission_controller/do_specific_supervision',DoSpecificSupervision)
    do_continuous_supervision_client = rospy.ServiceProxy('mission_controller/do_continuous_supervision',Trigger)
    signal.signal(signal.SIGINT, signal_handler)    # Associate signal SIGINT (Ctrl+C pressed) to handler (function "signal_handler")

    system("clear")
    print "Welcome to the pydashboard,"

    while not rospy.is_shutdown():
        main_menu()
        time.sleep(0.1)
