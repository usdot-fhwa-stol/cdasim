#!/usr/bin/env python
import json
import socket
import time
import rospy
# import carma_srvs
from std_msgs.msg import String, Header, Bool
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, PoseWithCovariance, Twist, TwistWithCovariance, Vector3
from math import cos, sin
from rosgraph_msgs.msg import Clock

# When using CARMA_client_interface_updated.py, run CARMA first on the same server.
# If the interface node and CARMA are on separate computers, set ROS_MASTER_URI variable before rosrun (http://wiki.ros.org/ROS/Tutorials/MultipleMachines).

# TODO: change imports below for where the messages actually are
# from carma_msgs.msg import ExternalObjectList, CarlaEgoVehicleControl, CarmaInit
# from cav_msgs.msg import ExternalObject
# from autoware_msgs.msg import VehicleStatus
# from carma_carla_comm import ExternalObjectList, CarlaEgoVehicleControl, CarmaInit
# from carma_carla_comm import ExternalObject
# from carma_carla_comm import VehicleStatus
from carla_msgs.msg import CarlaEgoVehicleControl
from autoware_msgs.msg import VehicleStatus
from cav_msgs.msg import ExternalObjectList, ExternalObject, RobotEnabled, CarlaEnabled


class CARMAInterface(object):

    def __init__(self):
        """
        Constructor

        """
        # Socket handling
        rospy.loginfo("starting init**************")
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.settimeout(5)   # 5 seconds

        self.ros_timestamp = 0
        self.last_timestamp = 0
        self.ce = CarlaEnabled()
        self.ce.carla_enabled = False
        self.re = RobotEnabled()
        self.re.robot_active = False

        self.ego_vehicle_id = -1

        self.throttle_cmd = 0
        self.brake_cmd = 0
        self.steering_cmd = 0

        self.current_veh_status = VehicleStatus()
        self.current_pose = PoseStamped()
        self.current_external_objects = ExternalObjectList()

        # init subscribers
        self.robot_status_sub = rospy.Subscriber(
            "controller/robot_status", RobotEnabled, self.robot_status_update)
        self.robot_status_sub = rospy.Subscriber(
            "/carla/ego_vehicle/vehicle_control_cmd", CarlaEgoVehicleControl, self.veh_control_cmd_update)

        # # subscribe to a topic using rospy.Subscriber class
        # UNCOMMENT line below for constant listening
        # sub=rospy.Subscriber('CarlaEgoVehicleControl_Topic', CarlaEgoVehicleControl, receive_data_from_CARMA_callback)
        # #publish messages to a topic using rospy.Publisher class
        self.sim_time_pub = rospy.Publisher(
            '/clock', Clock, queue_size=10)
        self.ce_pub = rospy.Publisher(
            'carla_enabled', CarlaEnabled, queue_size=1, latch=True)
        self.vs_pub = rospy.Publisher(
            'vehicle_status', VehicleStatus, queue_size=1)
        self.pose_pub = rospy.Publisher(
            'current_pose', PoseStamped, queue_size=1)
        self.eol_pub = rospy.Publisher(
            'external_objects', ExternalObjectList, queue_size=1)

    def destroy(self):
        """
        Function (override) to destroy this object.

        Terminate ROS subscription on AckermannDrive commands.
        Finish the PID controllers.
        Destroy the reconfiguration server

        :return:
        """
        self.robot_status_sub = None
        self.robot_status_sub = None

    def update_clock(self, carla_timestamp):
        """
        perform the update of the clock

        :param carla_timestamp: the current carla time
        :type carla_timestamp: carla.Timestamp
        :return:
        """
        self.last_timestamp = self.ros_timestamp
        self.ros_timestamp = rospy.Time.from_sec(carla_timestamp)
        self.sim_time_pub.publish(Clock(self.ros_timestamp))

    def carla_init_update(self):
        init_carma_response = {
            "ego_id": self.ego_vehicle_id, "isReady": self.re.robot_active}
        self.s.sendall(json.dumps(init_carma_response).encode('utf-8'))
        rospy.loginfo(init_carma_response)

        # Receive init data from Carla
        rx_data = self.s.recv(4096)
        rospy.loginfo(rx_data)
        carla_init_dict = json.loads(rx_data.decode('utf-8'))
        self.ego_vehicle_id = carla_init_dict["ego_id"]
        self.last_timestamp = self.ros_timestamp
        self.ros_timestamp = carla_init_dict["timestamp"]

    def carla_enabled_publish(self):
        # Publish initialization data from CARLA to CARMA topic
        self.ce.carla_enabled = True
        self.ce_pub.publish(self.ce)

    def robot_status_update(self, carma_robot_enabled):
        # Listen to subscribed topic message from CARMA
        rospy.loginfo('Receiving init message from CARMA')
        self.re.robot_active = carma_robot_enabled.robot_active
        rospy.loginfo(self.re.robot_active)

    # waits for a CarlaEgoVehicleControl message from CARMA
    # currently used

    def veh_control_cmd_update(self, carma_vehicle_control_cmd):
        # Listen to subscribed topic message from CARMA
        self.throttle_cmd = carma_vehicle_control_cmd.throttle
        self.brake_cmd = carma_vehicle_control_cmd.brake
        self.steering_cmd = carma_vehicle_control_cmd.steer
        rospy.loginfo('Receiving from CARMA: throttle %s', self.throttle_cmd)

    # helper function to convert given yaw pitch roll to quaternion
    # from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

    def ypr_to_quaternion(self, yaw, pitch, roll):
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

    def get_pose(self, state):
        p = Pose()
        p.position = Point()
        p.position.x = float(state["global_x"])
        p.position.y = float(state["global_y"])
        p.position.z = float(state["global_z"])
        yaw = float(state["global_yaw"])
        pitch = float(state["global_pitch"])
        roll = float(state["global_roll"])
        p.orientation = self.ypr_to_quaternion(yaw, pitch, roll)
        return p

    # helper function to publish VehicleStatus

    def vehilce_status_publish(self, CARLA_data_dict, h):
        self.current_veh_status.header = h
        self.current_veh_status.speed = float(
            CARLA_data_dict["ego_state"]["long_speed"])
        rospy.loginfo("vs.speed: %s", self.current_veh_status.speed)
        self.vs_pub.publish(self.current_veh_status)

    # helper function to publish PoseStamped

    def pose_publish(self, CARLA_data_dict, h):
        self.current_pose.pose = self.get_pose(CARLA_data_dict["ego_state"])
        self.current_pose.header = h
        rospy.loginfo("ps.pose.position.x: %s",
                      self.current_pose.pose.position.x)
        self.pose_pub.publish(self.current_pose)

    # helper function to publish ExternalObjectList

    def external_objects_publish(self, CARLA_data_dict, h):
        self.current_external_objects.header = h
        eols = []
        for cur_veh in CARLA_data_dict["veh_array"]:
            cur_eo = ExternalObject()
            cur_eo.id = int(CARLA_data_dict["veh_array"][cur_veh]["id"])
            cur_eo.header = h
            cur_eo.pose = PoseWithCovariance()
            cur_eo.pose.pose = self.get_pose(
                CARLA_data_dict["veh_array"][cur_veh])
            cur_eo.velocity = TwistWithCovariance()
            cur_eo.velocity.twist = Twist()
            cur_eo.velocity.twist.linear = Vector3()
            cur_eo.velocity.twist.linear.x = CARLA_data_dict["veh_array"][cur_veh]["long_speed"]
            eols.append(cur_eo)
            rospy.loginfo("cur_eo.velocity.twist.linear.x: %s",
                          cur_eo.velocity.twist.linear.x)
        self.current_external_objects.objects = eols
        self.eol_pub.publish(self.current_external_objects)

    def send_data_2_CARMA(self, CARLA_data_dict):
        # Publish to ROS topic that goes to CARMA
        rospy.loginfo('Sending to CARMA')
        h = Header()
        h.stamp = rospy.Time.from_sec(float(CARLA_data_dict["timestamp"]))
        self.vehilce_status_publish(CARLA_data_dict, h)
        self.pose_publish(CARLA_data_dict, h)
        self.external_objects_publish(CARLA_data_dict, h)

    def send_CARMA_data_2_CARLA(self, carla_2_carma_dict):
        rospy.loginfo('Sending to carla: carla_2_carma_dict')
        self.s.sendall(json.dumps(carla_2_carma_dict).encode('utf-8'))
        rospy.loginfo(carla_2_carma_dict)
        

    def receive_data_from_CARLA(self):
        decoded_string = bytes(b'')
        received_string = bytearray('', 'utf-8')
        while True:
            rx_data = self.s.recv(4096)
            received_string.extend(rx_data)
            rospy.loginfo("received_string rx_data: %s", received_string)
            try:
                decoded_string = eval(received_string.decode('utf-8'))
                received_string = bytearray('', 'utf-8')
                break
            except:
                continue
        return json.loads(decoded_string)

    def run(self):
        """
        Run the interface functionality.

        Registers on shutdown callback at rospy and spins ROS.

        :return:
        """

        # init
        self.sim_time_pub.publish(Clock(self.ros_timestamp))
        try:
            self.s.connect(('127.0.0.1', 8080))
        except socket.error as exc:
            rospy.logerr("Caught exception socket.error : %s", exc)
        rospy.loginfo("Socket connected *************")

        # Socket with CARLA Interface is connected, publish carla_enabled to carma
        self.carla_enabled_publish()
        rospy.loginfo("carla_enabled_publish")

        while not rospy.is_shutdown():

            while self.re.robot_active is False:
                # rospy.sleep(0.1)
                self.carla_init_update()
                self.update_clock(self.ros_timestamp)

            self.carla_init_update()

            dataDict = {"ego_id": self.ego_vehicle_id, "throttle": 0,
                        "brake": 0, "steering": 0, "drive_mode": "cruise"}
            self.send_CARMA_data_2_CARLA(dataDict)
            CARLA_data_dict = self.receive_data_from_CARLA()
            rospy.loginfo("CARLA_data_dict: %s",
                          CARLA_data_dict["timestamp"])

            # Publish /clock topic
            if self.last_timestamp.to_sec < float(CARLA_data_dict["timestamp"]):
                self.update_clock(float(CARLA_data_dict["timestamp"]))
                rospy.loginfo(self.ros_timestamp)

                # update_clock(CARLA_data_dict["timestamp"])
                self.send_data_2_CARMA(CARLA_data_dict)
                rospy.loginfo(CARLA_data_dict)

                dataDict = {"ego_id": self.ego_vehicle_id, "throttle": self.throttle_cmd,
                            "brake": self.brake_cmd, "steering": self.steering_cmd, "drive_mode": "cruise"}
                self.send_CARMA_data_2_CARLA(dataDict)

            rospy.spin()


def main():
    """

    main function

    :return:
    """
    rospy.init_node('CARMA_Interface', anonymous=True)
    carma_interface = CARMAInterface()
    try:
        carma_interface.run()
    finally:
        del carma_interface
        rospy.loginfo("Done")


if __name__ == "__main__":
    main()
