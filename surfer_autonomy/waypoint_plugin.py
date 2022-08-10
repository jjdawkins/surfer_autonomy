import pluginlib
import math
import surfer_autonomy.autonomy_utils as utils
import surfer_autonomy.autonomy as auto
from geometry_msgs.msg import Twist, Pose


class WaypointPlugin(auto.AutonomyPlugin):
    _alias_ ='waypoint'

    def init(self,params):
        self.params = params
        print(self.params)
        print("running waypoint")
        self.wp_rad = 0.2
        self.cruise_spd = 1
        print(self.params)


    def run(self):

        #print(self.poses)
        if(self.wp_received):
            msg = Twist()

            err = self.des_pos-self.pos
            dist = utils.norm2d(err)

            des_yaw = math.atan2(err[1],err[0])
            yaw_err = utils.wrapToPi(des_yaw - self.eul[2])
            msg.angular.z = 2*yaw_err
            if(yaw_err < 0.05):
                if(dist>1.0):
                    msg.linear.x = 1.0
                elif (dist < 1 and dist > self.wp_rad):
                    msg.linear.x = dist
                else:
                    msg.linear.x = 0.0

            else:
                msg.linear.x = 0.0

            self.cmd_vel_pub.publish(msg)


    def stop(self,string):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)
        self.des_pose = []
        print("stop")
