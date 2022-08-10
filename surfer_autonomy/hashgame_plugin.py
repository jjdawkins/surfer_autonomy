import pluginlib
import math
import surfer_autonomy.autonomy_utils as utils
import surfer_autonomy.autonomy as auto
from geometry_msgs.msg import Twist, Pose
import numpy as np


class HashGamePlugin(auto.AutonomyPlugin):
    _alias_ ='hashgame'

    def init(self,params):
        self.params = params
        print(self.params)
        print("running waypoint")
        self.wp_rad = 0.35
        self.cruise_spd = 1
        self.K = 1
        print(self.params)


    def run(self):
        self.names = ['Alice','Bob','Carol','Dave']

        my_indx = self.names.index(self.name)

        if(my_indx == 0):
            self.isleader = True
        else:
            self.isleader = False
            self.following = self.names[my_indx-1]
            print(self.name," ",self.following)

        if(self.wp_received):
            msg = Twist()

            if(self.isleader):
                err = self.des_pos-self.pos
                vel_cmd = self.K*(err)

                dist = utils.norm2d(err)
                dir = math.atan2(vel_cmd[1],vel_cmd[0])

                if(utils.norm2d(vel_cmd)>2):
                    vel_cmd = np.array([2*math.cos(dir),2*math.sin(dir),0])

                yaw_err = utils.wrapToPi(dir - self.eul[2])

                if(dist > self.wp_rad):
                    msg.angular.z = 2*yaw_err
                else:
                    msg.angular.z = 0.0

            else:
                print(self.poses)
                lead_pose = self.poses[self.following]
                L = 1
                offset = np.array([-L*math.cos(lead_pose[5]),-L*math.sin(lead_pose[5]),0])
                des_pos = lead_pose[0:3] + offset
                err = des_pos-self.pos
                vel_cmd = self.K*(err)
                dir = math.atan2(vel_cmd[1],vel_cmd[0])

                if(utils.norm2d(vel_cmd)>2):
                    vel_cmd = np.array([2*math.cos(dir),2*math.sin(dir),0])

                dist = utils.norm2d(err)

                if(dist > self.wp_rad):
                    yaw_err = utils.wrapToPi(dir - self.eul[2])
                else:
                    yaw_err = utils.wrapToPi(lead_pose[5] - self.eul[2])
                
                msg.angular.z = 2*yaw_err


            msg.linear.x = vel_cmd[0]*math.cos(self.eul[2]) + vel_cmd[1]*math.sin(self.eul[2])
            msg.linear.y = -vel_cmd[0]*math.sin(self.eul[2]) + vel_cmd[1]*math.cos(self.eul[2])

            self.cmd_vel_pub.publish(msg)

 #           if(yaw_err < 0.05):
 #               if(dist>1.0):
 #                   msg.linear.x = 1.0
 #               elif (dist < 1 and dist > self.wp_rad):
 #                   msg.linear.x = dist
 #               else:
 #                   msg.linear.x = 0.0

 #           else:
 #               msg.linear.x = 0.0

            


    def stop(self,string):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)
        self.des_pose = []
        print("stop")
