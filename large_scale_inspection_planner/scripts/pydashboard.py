#!/usr/bin/env python
from aerialcore_msgs.msg import GraphNode
from aerialcore_msgs.srv import DoSpecificSupervision, DoSpecificSupervisionRequest, StartSupervising, StartSupervisingRequest, StopSupervising, StopSupervisingRequest, PostString, PostStringRequest
from std_srvs.srv import Trigger, TriggerRequest
import rospkg
import rospy
import time
from os import system
from os import listdir
from os.path import isfile, join
import signal
import sys


def main_menu():
    
    print "\nMain menu of the pydashboard. Please choose the option to send:"
    print "1. Start supervising"
    print "2. Stop supervising"
    print "3. Do specific supervision"
    print "4. Do continuous supervision"
    print "5. Start specific supervision plan"

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
    elif selected == "5":
        start_specific_supervision_plan_menu()

    else:
        system("clear")
        print "Not a valid option."


# 1. Start supervising:
def start_supervising_menu():
    system("clear")
    print "Start supervising selected."
    print "To start all UAVs don't input any id. Write wrong id (char or negative integer) to stop the input."
    start_supervising_request = StartSupervisingRequest()
    while True and not rospy.is_shutdown():
        uav_id_input = raw_input("Write UAV id >> ")
        uav_id_input_unicode = unicode(uav_id_input, 'utf-8')
        if uav_id_input_unicode.isnumeric() and int(uav_id_input)>=0:   # uav_id_input.isalpha() return true if uav_id_input is a leter
            start_supervising_request.uav_id.append(int(uav_id_input))
        else:
            break
    try:
        print start_supervising_client.call(start_supervising_request)
    except rospy.ServiceException, e:
        print "\nService call failed: %s"%e


# 2. Stop supervising:
def stop_supervising_menu():
    system("clear")
    print "Stop supervising selected."
    print "To stop all UAVs don't input any id. Write wrong id (char or negative integer) to stop the input."
    stop_supervising_request = StopSupervisingRequest()
    while True and not rospy.is_shutdown():
        uav_id_input = raw_input("Write UAV id >> ")
        uav_id_input_unicode = unicode(uav_id_input, 'utf-8')
        if uav_id_input_unicode.isnumeric() and int(uav_id_input)>=0:   # uav_id_input.isalpha() return true if uav_id_input is a leter
            stop_supervising_request.uav_id.append(int(uav_id_input))
        else:
            break
    try:
        print stop_supervising_client.call(stop_supervising_request)
    except rospy.ServiceException, e:
        print "\nService call failed: %s"%e


# 3. Do specific supervision:
def do_specific_supervision_menu():
    system("clear")
    print "Do specific supervision."

    # Custom specific subgraph to send:
    do_specific_supervision_request = DoSpecificSupervisionRequest()

    # Pylon 0
    graph_node_struct = GraphNode()
    graph_node_struct.type = GraphNode.TYPE_PYLON
    graph_node_struct.x = 699.429
    graph_node_struct.y = 219.137
    graph_node_struct.z = 20
    graph_node_struct.latitude = 38.140705
    graph_node_struct.longitude = -3.165848
    graph_node_struct.altitude = 20
    graph_node_struct.connections_indexes.append(1)
    do_specific_supervision_request.specific_subgraph.append(graph_node_struct)

    # Pylon 1
    graph_node_struct = GraphNode()
    graph_node_struct.type = GraphNode.TYPE_PYLON
    graph_node_struct.x = 641.646
    graph_node_struct.y = 169.93
    graph_node_struct.z = 20
    graph_node_struct.latitude = 38.140261
    graph_node_struct.longitude = -3.166506
    graph_node_struct.altitude = 20
    graph_node_struct.connections_indexes.append(0)
    graph_node_struct.connections_indexes.append(2)
    do_specific_supervision_request.specific_subgraph.append(graph_node_struct)

    # Pylon 2
    graph_node_struct = GraphNode()
    graph_node_struct.type = GraphNode.TYPE_PYLON
    graph_node_struct.x = 162.857
    graph_node_struct.y = 5.807
    graph_node_struct.z = 20
    graph_node_struct.latitude = 38.138774
    graph_node_struct.longitude = -3.171967
    graph_node_struct.altitude = 20
    graph_node_struct.connections_indexes.append(1)
    graph_node_struct.connections_indexes.append(3)
    do_specific_supervision_request.specific_subgraph.append(graph_node_struct)

    # Pylon 3
    graph_node_struct = GraphNode()
    graph_node_struct.type = GraphNode.TYPE_PYLON
    graph_node_struct.x = 62.321
    graph_node_struct.y = -28.535
    graph_node_struct.z = 20
    graph_node_struct.latitude = 38.138462
    graph_node_struct.longitude = -3.173113
    graph_node_struct.altitude = 20
    graph_node_struct.connections_indexes.append(2)
    graph_node_struct.connections_indexes.append(4)
    do_specific_supervision_request.specific_subgraph.append(graph_node_struct)

    # Pylon 4
    graph_node_struct = GraphNode()
    graph_node_struct.type = GraphNode.TYPE_PYLON
    graph_node_struct.x = -492.757
    graph_node_struct.y = -240.6
    graph_node_struct.z = 20
    graph_node_struct.latitude = 38.136542
    graph_node_struct.longitude = -3.179443
    graph_node_struct.altitude = 20
    graph_node_struct.connections_indexes.append(3)
    graph_node_struct.connections_indexes.append(5)
    do_specific_supervision_request.specific_subgraph.append(graph_node_struct)

    # Pylon 5
    graph_node_struct = GraphNode()
    graph_node_struct.type = GraphNode.TYPE_PYLON
    graph_node_struct.x = -490.135
    graph_node_struct.y = -216.032
    graph_node_struct.z = 20
    graph_node_struct.latitude = 38.136763
    graph_node_struct.longitude = -3.179413
    graph_node_struct.altitude = 20
    graph_node_struct.connections_indexes.append(4)
    graph_node_struct.connections_indexes.append(6)
    do_specific_supervision_request.specific_subgraph.append(graph_node_struct)

    # Pylon 6
    graph_node_struct = GraphNode()
    graph_node_struct.type = GraphNode.TYPE_PYLON
    graph_node_struct.x = -503.375
    graph_node_struct.y = -157.510
    graph_node_struct.z = 20
    graph_node_struct.latitude = 38.137290
    graph_node_struct.longitude = -3.179566
    graph_node_struct.altitude = 20
    graph_node_struct.connections_indexes.append(5)
    do_specific_supervision_request.specific_subgraph.append(graph_node_struct)

    # Pylon 7
    graph_node_struct = GraphNode()
    graph_node_struct.type = GraphNode.TYPE_PYLON
    graph_node_struct.x = -488.455
    graph_node_struct.y = -298.354
    graph_node_struct.z = 20
    graph_node_struct.latitude = 38.136031
    graph_node_struct.longitude = -3.179382
    graph_node_struct.altitude = 20
    graph_node_struct.connections_indexes.append(7)
    do_specific_supervision_request.specific_subgraph.append(graph_node_struct)

    # Pylon 8
    graph_node_struct = GraphNode()
    graph_node_struct.type = GraphNode.TYPE_PYLON
    graph_node_struct.x = -33.345
    graph_node_struct.y = -276.977
    graph_node_struct.z = 20
    graph_node_struct.latitude = 38.136222
    graph_node_struct.longitude = -3.174199
    graph_node_struct.altitude = 20
    graph_node_struct.connections_indexes.append(6)
    graph_node_struct.connections_indexes.append(8)
    do_specific_supervision_request.specific_subgraph.append(graph_node_struct)

    # Pylon 9
    graph_node_struct = GraphNode()
    graph_node_struct.type = GraphNode.TYPE_PYLON
    graph_node_struct.x = 465.385
    graph_node_struct.y = -626.630
    graph_node_struct.z = 20
    graph_node_struct.latitude = 38.133079
    graph_node_struct.longitude = -3.168501
    graph_node_struct.altitude = 20
    graph_node_struct.connections_indexes.append(7)
    do_specific_supervision_request.specific_subgraph.append(graph_node_struct)

    graph_node_struct = GraphNode()
    graph_node_struct.type = GraphNode.TYPE_RECHARGE_LAND_STATION
    graph_node_struct.x = 28
    graph_node_struct.y = 61
    graph_node_struct.z = 0.32
    graph_node_struct.latitude = 38.139275
    graph_node_struct.longitude = -3.173516
    graph_node_struct.altitude = 444
    do_specific_supervision_request.specific_subgraph.append(graph_node_struct)

    graph_node_struct = GraphNode()
    graph_node_struct.type = GraphNode.TYPE_REGULAR_LAND_STATION
    graph_node_struct.x = 38.8
    graph_node_struct.y = 65
    graph_node_struct.z = 0
    graph_node_struct.latitude = 38.139309
    graph_node_struct.longitude = -3.173386
    graph_node_struct.altitude = 444
    do_specific_supervision_request.specific_subgraph.append(graph_node_struct)

    graph_node_struct = GraphNode()
    graph_node_struct.type = GraphNode.TYPE_REGULAR_LAND_STATION
    graph_node_struct.x = 32.5
    graph_node_struct.y = 61
    graph_node_struct.z = 0
    graph_node_struct.latitude = 38.139272
    graph_node_struct.longitude = -3.173451
    graph_node_struct.altitude = 444
    do_specific_supervision_request.specific_subgraph.append(graph_node_struct)

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


# 5. Start specific supervision plan:
def start_specific_supervision_plan_menu():
    system("clear")
    #show the files in the directory:
    onlyfiles = [f for f in listdir(plans_path) if isfile(join(plans_path, f))]
    print "Start specific supervision plan selected. The next plans' YAMLs are available:"
    cont = 0
    orded_list = sorted(onlyfiles)
    for i in range(len(orded_list)):
        num = i+int(1)
        print("%d. %s" %(num, orded_list[i]) )
        cont = cont+1
    print("Press any other key to quit.")
    selected = raw_input(" >> ")
    try:
        if not( selected.isalpha() ) and int(selected)>=1 and int(selected) <= len(orded_list): # selected.isalpha() return true if selected is a leter
            yaml_plan_name = orded_list[int(selected)-1]
            print("%s selected" %yaml_plan_name )
            yaml_string = open( plans_path + yaml_plan_name, 'r').read()
            yaml_string_plan_to_post = PostStringRequest()
            yaml_string_plan_to_post.data = yaml_string
            try:
                print start_specific_supervision_plan_client.call(yaml_string_plan_to_post)
            except rospy.ServiceException, e:
                print "\nService call failed: %s"%e
        else:
            system("clear")
            print "Not a valid option."
    except:
        system("clear")
        print "Not a valid option."


# Finish the execution directly when Ctrl+C is pressed (signal.SIGINT received), without escalating to SIGTERM.
def signal_handler(sig, frame):
    print('Ctrl+C pressed, signal.SIGINT received.')
    sys.exit(0)


if __name__ == "__main__":
    rospy.init_node('pydashboard', anonymous=True)
    plans_path = rospy.get_param('~plans_path')
    start_supervising_client = rospy.ServiceProxy('mission_controller/start_supervising',StartSupervising)
    stop_supervising_client = rospy.ServiceProxy('mission_controller/stop_supervising',StopSupervising)
    do_specific_supervision_client = rospy.ServiceProxy('mission_controller/do_specific_supervision',DoSpecificSupervision)
    do_continuous_supervision_client = rospy.ServiceProxy('mission_controller/do_continuous_supervision',Trigger)
    start_specific_supervision_plan_client = rospy.ServiceProxy('mission_controller/start_specific_supervision_plan',PostString)
    signal.signal(signal.SIGINT, signal_handler)    # Associate signal SIGINT (Ctrl+C pressed) to handler (function "signal_handler")

    system("clear")
    print "Welcome to the pydashboard,"

    while not rospy.is_shutdown():
        main_menu()
        time.sleep(0.1)
