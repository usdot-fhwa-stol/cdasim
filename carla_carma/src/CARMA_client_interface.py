#!/usr/bin/env python
import json
import socket
import time
import rospy
# import carma_srvs
from std_msgs.msg import String, Header, Bool
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, PoseWithCovariance, Twist, TwistWithCovariance, Vector3
from math import cos, sin

# When using CARMA_client_interface_updated.py, run CARMA first on the same server.
# If the interface node and CARMA are on separate computers, set ROS_MASTER_URI variable before rosrun (http://wiki.ros.org/ROS/Tutorials/MultipleMachines).

# TODO: change imports below for where the messages actually are
#from carma_msgs.msg import ExternalObjectList, CarlaEgoVehicleControl, CarmaInit
#from cav_msgs.msg import ExternalObject
#from autoware_msgs.msg import VehicleStatus
#from carma_carla_comm import ExternalObjectList, CarlaEgoVehicleControl, CarmaInit
#from carma_carla_comm import ExternalObject
#from carma_carla_comm import VehicleStatus
from carla_carma.msg import VehicleStatus, CarmaInit, CarlaEgoVehicleControl
from cav_msgs.msg import ExternalObjectList, RobotEnabled, CarlaEnabled 

def send_init_message_2_CARMA(carla_2_carma_init):
    # Publish initialization data from CARLA to CARMA topic
#    print('Sending init message to CARMA')
    ci = CarmaInit()
    ci.size = Vector3()
    ci.size.x = float(carla_2_carma_init["ego_info"]["x_length"])
    ci.size.y = float(carla_2_carma_init["ego_info"]["y_length"])
    ci.size.z = float(carla_2_carma_init["ego_info"]["z_length"])
#    ci.mass = float(carla_2_carma_init["ego_info"]["mass"])
    ci.pose = Pose()
    ci.pose.position = Point()
    state = carla_2_carma_init["ego_state"]
    ci.pose.position.x = float(state["x_pose"])
    ci.pose.position.y = float(state["y_pose"])
    ci.pose.position.z = float(state["z_pose"])
    yaw = float(state["yaw"])
    pitch = float(state["pitch"])
    roll = float(state["roll"])
    ci.pose.orientation = ypr_to_quaternion(yaw, pitch, roll)
    ci.id = int(carla_2_carma_init["ego_id"])
    ci.map_id = int(carla_2_carma_init["map_id"])
    pub_init.publish(ci)
    ce = CarlaEnabled()
    ce.carla_enabled = True
    pub_ce.publish(ce) 

def receive_init_message_from_CARMA():
    # Listen to subscribed topic message from CARMA
    print('Receiving init message from CARMA')
    data = rospy.wait_for_message('RobotEnabled_Topic', Bool) 
    #data = rospy.wait_for_message('IsReady_Topic', Bool)
    return data.data

# callback function, data will be a CarlaEgoVehicleControl message
# currently NOT used, as we wait for CARMA data in main loop instead of using callback interrupts
def receive_data_from_CARMA_callback(data):
    # Listen to subscribed topic message from CARMA
    print('Receiving from CARMA')
    throttle, brake, steering = data.throttle, data.brake, data.steer
    dataDict = {"ego_id": ego_vehicle_id, "throttle": throttle, "brake": brake, "steering": steering, "drive_mode": "cruise"}
    send_CARMA_data_2_CARLA(dataDict)

# waits for a CarlaEgoVehicleControl message from CARMA
# currently used
def receive_data_from_CARMA():
    # Listen to subscribed topic message from CARMA
    print('Receiving from CARMA')
    data = rospy.wait_for_message('CarlaEgoVehicleControl_Topic', CarlaEgoVehicleControl)
    return data.throttle, data.brake, data.steer

# helper function to convert given yaw pitch roll to quaternion
# from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
def ypr_to_quaternion(yaw, pitch, roll):
    q = Quaternion()
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q
    
def get_pose(state):
    p = Pose()
    p.position = Point()
    p.position.x = float(state["global_x"])
    p.position.y = float(state["global_y"])
    p.position.z = float(state["global_z"])
    yaw = float(state["global_yaw"])
    pitch = float(state["global_pitch"])
    roll = float(state["global_roll"])
    p.orientation = ypr_to_quaternion(yaw, pitch, roll)
    return p
    
# helper function to publish VehicleStatus
def send_vs_2_CARMA(CARLA_data_dict, h):
    vs = VehicleStatus()
    vs.header = h
    vs.speed = float(CARLA_data_dict["ego_state"]["long_speed"])
    pub_vs.publish(vs)

# helper function to publish PoseStamped
def send_pose_2_CARMA(CARLA_data_dict, h):
    ps = PoseStamped()
    ps.pose = get_pose(CARLA_data_dict["ego_state"])
    ps.header = h
    pub_pose.publish(ps)
    
# helper function to publish ExternalObjectList
def send_eol_2_CARMA(CARLA_data_dict, h):
    eol = ExternalObjectList()
    eol.header = h
    eols = []
    for cur_veh in CARLA_data_dict["veh_array"]:
        cur_eo = ExternalObject()
        cur_eo.id = int(CARLA_data_dict["veh_array"][cur_veh]["id"])
        cur_eo.header = h
        cur_eo.pose = PoseWithCovariance()
        cur_eo.pose.pose = get_pose(CARLA_data_dict["veh_array"][cur_veh])
        cur_eo.velocity = TwistWithCovariance()
        cur_eo.velocity.twist = Twist()
        cur_eo.velocity.twist.linear = Vector3()
        cur_eo.velocity.twist.linear.x = CARLA_data_dict["veh_array"][cur_veh]["long_speed"]
        eols.append(cur_eo)
    eol.objects = eols
    pub_eol.publish(eol)

def send_data_2_CARMA(CARLA_data_dict):
    # Publish to ROS topic that goes to CARMA
    print('Sending to CARMA')
    h = Header()
    h.stamp = rospy.Time.from_sec(float(CARLA_data_dict["timestamp"]))
    send_vs_2_CARMA(CARLA_data_dict, h)
    send_pose_2_CARMA(CARLA_data_dict, h)
    send_eol_2_CARMA(CARLA_data_dict, h)

def send_CARMA_data_2_CARLA(carla_2_carma_dict):
    s.sendall(json.dumps(carla_2_carma_dict).encode('utf-8'))

def receive_data_from_CARLA():
    decoded_string = bytes(b'')
    received_string = bytearray('', 'utf-8')
    while True:
        rx_data = s.recv(4096)
        received_string.extend(rx_data)
        try:
            decoded_string = eval(received_string.decode('utf-8'))
            received_string = bytearray('', 'utf-8')
            break
        except:
            continue
    return json.loads(decoded_string)

# Socket handling
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('127.0.0.1', 8080)) # TODO: Change IP address for the CARLA server address

##### Setup subscribers and publishers for each of the exchanged messages to ROS
rospy.init_node('CARMA_Interface')
# # subscribe to a topic using rospy.Subscriber class
# UNCOMMENT line below for constant listening
# sub=rospy.Subscriber('CarlaEgoVehicleControl_Topic', CarlaEgoVehicleControl, receive_data_from_CARMA_callback)
# #publish messages to a topic using rospy.Publisher class
pub_vs=rospy.Publisher('VehicleStatus_Topic', VehicleStatus, queue_size=1)
pub_pose=rospy.Publisher('PoseStamped_Topic', PoseStamped, queue_size=1)
pub_eol=rospy.Publisher('ExternalObjectList_Topic', ExternalObjectList, queue_size=1)
pub_init=rospy.Publisher('CarmaInit_Topic', CarmaInit, queue_size=1)
pub_ce=rospy.Publisher('CarlaEnabled_Topic',CarlaEnabled,queue_size=1)
# Initialization stage
ego_vehicle_id = -1
init_carma_response = {"ego_id": ego_vehicle_id, "isReady": False}
while init_carma_response["isReady"] is False:
    s.sendall(json.dumps(init_carma_response).encode('utf-8'))
    rx_data = s.recv(4096)
    try:
        carla_init_dict = json.loads(rx_data.decode('utf-8'))
        ego_vehicle_id = carla_init_dict["ego_id"]
        print('Start message received from CARLA server')
        send_init_message_2_CARMA(carla_init_dict)
        print('Sending init message to CARMA')
        init_carma_response["isReady"] = receive_init_message_from_CARMA()
        print('Receiving reply from CARMA')
        if init_carma_response["isReady"]:
            print('Green light for simulation!')
            s.sendall(json.dumps(init_carma_response).encode('utf-8'))
            continue
    except:
        print('Something is wrong with the message')
    print('Waiting for the CARLA server')
    time.sleep(1)

# Starting recurrent execution
time.sleep(5)

dataDict = {"ego_id": ego_vehicle_id, "throttle": 0, "brake": 0, "steering": 0, "drive_mode": "cruise"}
send_CARMA_data_2_CARLA(dataDict)
CARLA_data_dict = receive_data_from_CARLA()
send_data_2_CARMA(CARLA_data_dict)
print(CARLA_data_dict)
while True:
    # BEGIN this section waits for CARMA data, can be removed and replaced with callback
    throttle, brake, steering = receive_data_from_CARMA()
    dataDict = {"ego_id": ego_vehicle_id, "throttle": throttle, "brake": brake, "steering": steering, "drive_mode": "cruise"}
    send_CARMA_data_2_CARLA(dataDict)
    # END
    
    CARLA_data_dict = receive_data_from_CARLA()
    send_data_2_CARMA(CARLA_data_dict)

    print(CARLA_data_dict["ego_state"]["global_y"])  # Example of how to access dictionary fields
