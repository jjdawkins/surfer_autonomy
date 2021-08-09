import pluginlib
import math
import surfer_autonomy.autonomy_utils as utils
import surfer_autonomy.autonomy as auto
from geometry_msgs.msg import Twist, Pose


class PathPlugin(auto.AutonomyPlugin):
    _alias_ ='path'

    def init(self,params):
        self.wp_rad = 0.2
        self.cruise_spd = 1


    def run(self):
        print("running path")
        msg = Twist()
        msg.angular.z = 1.57
        msg.linear.x = 0.0
        self.cmd_vel_pub.publish(msg)


    def stop(self,string):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)        
        self.des_pose = []
        print(stop)
