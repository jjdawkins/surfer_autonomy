import pluginlib
import rclpy
from transforms3d import euler
import numpy as np
import quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, PoseArray
from surfer_msgs.msg import Status


@pluginlib.Parent('autonomy_plugin')

class AutonomyPlugin(object):
    def __init__(self):
        self.cmd_vel_pub = []
        self.pos = np.zeros(3)
        self.vel_g = np.zeros(3)
        self.vel_b = np.zeros(3)
        self.quat = np.array([1,0,0,0])
        self.eul =  np.zeros(3)
        self.des_pos = np.zeros(3)
        self.wp_received = False
        self.path_received = False
        self.path = []
        self.params = []
        self.name = ''
        self.group = ''
        self.type = ''
        self.poses = {}
        

    @pluginlib.abstractmethod
    def init(self,params):
        self.params = params
        print(self.params)

    @pluginlib.abstractmethod
    def run(self):
        self.run = 0

    @pluginlib.abstractmethod
    def stop(self,string):
        self.stop = 0

    def set_status(self,status):
        print("my_status ",status)
        self.name = status.name
        self.group = status.group
        self.type = status.type

    def set_publisher(self,pub):
        self.cmd_vel_pub = pub

    def wp_cb(self,msg):
        self.des_pos = np.array([msg.position.x,msg.position.y,msg.position.z])
        self.wp_received = True

    def path_cb(self,msg):
        self.path = msg
        self.path_received = True   

    def poses_cb(self,caller,msg):
        pose = np.zeros(6)
        pose[0:3] = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        quat = np.array([msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z])

        pose[3:7] = euler.quat2euler(quat)

        self.poses[caller]=pose


    def odom_cb(self,msg):
        self.pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        self.quat = np.array([msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z])

        self.eul = euler.quat2euler(self.quat)

        # Ododmetry in simulation comes in body frame which is different from qualysis package will have to determine a standard. it probably makes the
        # the most sense for the odometry to represent the b frame velocity rather than inertial frame
        q_gb = np.quaternion(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)
        self.vel_b = np.array([msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z])
        self.vel_g = quaternion.rotate_vectors(q_gb.conjugate() ,self.vel_b)

    pass
