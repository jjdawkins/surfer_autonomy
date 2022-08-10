import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
from surfer_msgs.msg import Status
from surfer_msgs.srv import SetBehavior, GetBehaviors
#import tf2_py
## TODO: get behaviors service to populate website with options
valid_behaviors = ['CIRCLE','FOLLOW','WAYPOINT']

##TODO: Manager should own the behavior service and get the behaviors from a parameter file common to the autonomy node.
# this should allow the optional parameters to be loaded from the parameterr
# Down the line the parameters files can be configured from web interface

class BasicAutonomy(Node):

    def __init__(self):
        super().__init__('autonomy_node')
        self.status = Status()
        self.yaw = 0
        self.xvel_g = 0
        self.yvel_g = 0
        self.xvel_b = 0
        self.yvel_b = 0
        self.xpos = 0
        self.ypos = 0

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.subscription = self.create_subscription(
            Status,
            'status',
            self.statusCallback,
            10)
        self.subscription  # prevent unused variable warning
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odomCallBack,
            1
        )
        self.wp_sub = self.create_subscription(
            Pose,
            'waypoint',
            self.waypointCallBack,
            1
        )
        self.wp_sub

        self.des_pose = Pose()


    def odomCallBack(self,msg):
        orientation_q = msg.pose.pose.orientation # extract quaternion from pose message
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w] # put quaternion elements into an array
    #    euler = tf.transformations.euler_from_quaternion(orientation_list)
    #    self.yaw = euler[2]

        #(roll, pitch, yaw) = euler_from_quaternion (orientation_list) # convert quaternion to Euler angles
        self.xvel_g = msg.twist.twist.linear.x # parse x-component of inertial speed from odometry message
        self.yvel_g = msg.twist.twist.linear.y # parse y-component of inertial speed from odometry message

        self.xvel_b = math.cos(self.yaw)*self.xvel_g + math.sin(self.yaw)*self.yvel_g
        self.yvel_b = -math.sin(self.yaw)*self.xvel_g + math.cos(self.yaw)*self.yvel_g

        #self.speed = math.sqrt(speedx*speedx + speedy*speedy)
        self.xpos = msg.pose.pose.position.x # parse x-component of position from odometry message
        self.ypos = msg.pose.pose.position.y # parse y-component of position from odometry message

    def waypointCallBack(self,msg):
        self.des_pose = msg

    def statusCallback(self,msg):
        self.status = msg

    def waypointControl(self):
        msg = Twist()


    def timer_callback(self):
        msg = Twist()
        if(self.status.armed):
            msg.linear.x = 0.4
            msg.angular.z = 0.6
            self.cmd_vel_pub.publish(msg)
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    ba = BasicAutonomy()

    rclpy.spin(ba)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ba.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
