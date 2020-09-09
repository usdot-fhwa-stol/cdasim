#!/usr/bin/env python
import json
import socket
import time
import rospy
# import carma_srvs
from std_msgs.msg import String, Header, Bool
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, PoseWithCovariance, Twist, TwistWithCovariance, Vector3, TwistStamped
from math import cos, sin, tan
from rosgraph_msgs.msg import Clock

# When using CARMA_client_interface_updated.py, run CARMA first on the same server.
# If the interface node and CARMA are on separate computers, set ROS_MASTER_URI variable before rosrun (http://wiki.ros.org/ROS/Tutorials/MultipleMachines).

from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
from autoware_msgs.msg import VehicleStatus
from cav_msgs.msg import ExternalObjectList, ExternalObject, RobotEnabled, CarlaEnabled


class CARMAInterface(object):

    def __init__(self):
        """
        Constructor

        """
        # Socket handling
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.settimeout(1000)   # 1000 seconds

        self.sockert_ip = rospy.get_param('~sockert_ip', '127.0.0.1')
        self.sockert_port = rospy.get_param('~sockert_port', '8080')

        self.ros_timestamp = 0
        self.last_timestamp = 0
        self.ce = CarlaEnabled()
        self.ce.carla_enabled = False
        self.re = RobotEnabled()
        self.re.robot_enabled = False

        self.ego_vehicle_id = -1

        self.throttle_cmd = 0
        self.brake_cmd = 0
        self.steering_cmd = 0

        self.current_veh_status = VehicleStatus()
        self.current_ego_veh_status = CarlaEgoVehicleStatus()
        self.current_pose = PoseStamped()
        self.current_external_objects = ExternalObjectList()
        self.current_twist = TwistStamped()

        # init subscribers
        self.robot_status_sub = rospy.Subscriber(
            "controller/robot_status", RobotEnabled, self.robot_status_update)
        self.veh_cmd_sub = rospy.Subscriber(
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
        self.ego_vs_pub = rospy.Publisher(
            '/carla/ego_vehicle/vehicle_status', CarlaEgoVehicleStatus, queue_size=1)
        self.pose_pub = rospy.Publisher(
            '/localization/current_pose', PoseStamped, queue_size=1)
        self.eol_pub = rospy.Publisher(
            '/environment/external_objects', ExternalObjectList, queue_size=1)
        self.current_twist_pub = rospy.Publisher(
            'vehicle/twist', TwistStamped, queue_size=10)

    def destroy(self):
        """
        Function (override) to destroy this object.

        Terminate ROS subscription on AckermannDrive commands.
        Finish the PID controllers.
        Destroy the reconfiguration server

        :return:
        """
        self.robot_status_sub = None
        self.veh_cmd_sub = None

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
            "ego_id": self.ego_vehicle_id, "isReady": self.re.robot_enabled}
        self.s.sendall(json.dumps(init_carma_response).encode('utf-8'))

        # Receive init data from Carla
        rx_data = self.s.recv(4096)
        carla_init_dict = json.loads(rx_data.decode('utf-8'))
        self.ego_vehicle_id = carla_init_dict["ego_id"]
        self.last_timestamp = self.ros_timestamp
        self.ros_timestamp = rospy.Time.from_sec(
            float(carla_init_dict["timestamp"]))

    def robot_status_update(self, carma_robot_enabled):
        self.re.robot_enabled = carma_robot_enabled.robot_enabled

    def carla_enabled_publish(self):
        # Publish initialization data from CARLA to CARMA topic
        self.ce.carla_enabled = True
        self.ce_pub.publish(self.ce)

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
        yaw = float(state["rotation_yaw"])
        pitch = float(state["rotation_pitch"])
        roll = float(state["rotation_roll"])
        p.orientation = self.ypr_to_quaternion(yaw, pitch, roll)
        return p

    # helper function to publish VehicleStatus

    def vehilce_status_publish(self, CARLA_data_dict, h):
        self.current_veh_status.header = h
        self.current_veh_status.speed = float(
            CARLA_data_dict["ego_state"]["velocity_x"])*3.6
        # rospy.loginfo("vs.speed: %s", self.current_veh_status.speed)
        self.vs_pub.publish(self.current_veh_status)

    def ego_vehilce_status_publish(self, CARLA_data_dict, h):
        self.current_ego_veh_status.header = h
        self.current_ego_veh_status.velocity = float(
            CARLA_data_dict["ego_state"]["velocity_x"]) * 3.6
        self.current_ego_veh_status.orientation = self.current_pose.pose.orientation
        rospy.loginfo("vs.speed: %s", self.current_ego_veh_status.velocity)
        self.ego_vs_pub.publish(self.current_ego_veh_status)

    def current_twist_publish(self, CARLA_data_dict, h):
        # adaptive_gear_ratio_ = max(
        #     1e-5, 15.713 + 0.053 * velocity * velocity - 0.042 * steering_wheel_angle)
        # curvature = tan(steering_wheel_angle/adaptive_gear_ratio_)/2.79

        self.current_twist.header = h
        self.current_twist.twist.linear.x = float(
            CARLA_data_dict["ego_state"]["velocity_x"])
        self.current_twist.twist.linear.y = float(
            CARLA_data_dict["ego_state"]["velocity_y"])
        self.current_twist.twist.linear.z = float(
            CARLA_data_dict["ego_state"]["velocity_z"])
        self.current_twist.twist.angular.x = float(
            CARLA_data_dict["ego_state"]["angular_x"])
        self.current_twist.twist.angular.y = float(
            CARLA_data_dict["ego_state"]["angular_y"])
        self.current_twist.twist.angular.z = float(
            CARLA_data_dict["ego_state"]["angular_z"])
        self.current_twist_pub.publish(self.current_twist)

    def pose_publish(self, CARLA_data_dict, h):
        self.current_pose.pose = self.get_pose(CARLA_data_dict["ego_state"])
        self.current_pose.header = h
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
            cur_eo.velocity.twist.linear.x = CARLA_data_dict["veh_array"][cur_veh]["velocity_x"]
            eols.append(cur_eo)
        self.current_external_objects.objects = eols
        self.eol_pub.publish(self.current_external_objects)

    def send_data_2_CARMA(self, CARLA_data_dict):
        # Publish to ROS topic that goes to CARMA
        h = Header()
        h.stamp = rospy.Time.from_sec(float(CARLA_data_dict["timestamp"]))
        self.vehilce_status_publish(CARLA_data_dict, h)
        self.current_twist_publish(CARLA_data_dict, h)
        self.pose_publish(CARLA_data_dict, h)
        self.ego_vehilce_status_publish(CARLA_data_dict, h)
        self.external_objects_publish(CARLA_data_dict, h)

    def send_CARMA_data_2_CARLA(self, carla_2_carma_dict):
        self.s.sendall(json.dumps(carla_2_carma_dict).encode('utf-8'))

    def receive_data_from_CARLA(self):
        decoded_string = bytes(b'')
        received_string = bytearray('', 'utf-8')
        while True:
            rx_data = self.s.recv(4096)
            received_string.extend(rx_data)
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
        rospy.loginfo("ip: {}, port: {}".format(self.sockert_ip, int(self.sockert_port)))
        try:
            self.s.connect((self.sockert_ip, int(self.sockert_port)))
        except socket.error as exc:
            rospy.logerr("Caught exception socket.error : %s", exc)

        # Socket with CARLA Interface is connected, publish carla_enabled to carma
        self.carla_enabled_publish()

        while not rospy.is_shutdown():

            while self.re.robot_enabled is False:
                # rospy.sleep(0.1)
                self.carla_init_update()
                # self.update_clock(self.ros_timestamp)
                self.sim_time_pub.publish(Clock(self.ros_timestamp))

            self.carla_init_update()

            while True:
                dataDict = {"ego_id": self.ego_vehicle_id, "throttle": self.throttle_cmd,
                            "brake": self.brake_cmd, "steering": self.steering_cmd, "drive_mode": "cruise"}
                self.send_CARMA_data_2_CARLA(dataDict)
                CARLA_data_dict = self.receive_data_from_CARLA()
                self.update_clock(float(CARLA_data_dict["timestamp"]))
                self.send_data_2_CARMA(CARLA_data_dict)

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
