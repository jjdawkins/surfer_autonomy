import pluginlib
import math
import surfer_autonomy.autonomy_utils as utils
import surfer_autonomy.autonomy as auto
from geometry_msgs.msg import Twist, Pose
import numpy as np


class FormationPlugin(auto.AutonomyPlugin):
    _alias_ ='formation'

    def init(self,params):
        self.params = params
        print(self.params)
        print("running waypoint")
        self.wp_rad = 0.35
        self.cruise_spd = 1
        self.Kp = 1
        self.Kv = 0.2
        self.Cp = 1
        self.L = 0.15
        print(self.params)
        self.formation = {}
        self.formation['Alice']= np.zeros(3)
        self.formation['Bob']=np.array([1,0,0])
        self.formation['Carol']=np.array([0,1,0])
        self.formation['Dave']=np.array([1,1,0])

    def run(self):
        self.names = ['Alice','Bob','Carol','Dave']

        my_indx = self.names.index(self.name)

        if(my_indx == 0):
            self.isleader = True
            self.following = self.names[-1]
        else:
            self.isleader = False
            self.following = self.names[my_indx-1]
            print(self.name," ",self.following)

        if(self.wp_received):
            msg = Twist()

            vel_cmd = np.zeros(3)

            if(self.isleader):
                
                err = self.des_pos - self.pos + ctrl_pnt
                vel_cmd = self.Cp*(err)
                lead_pose = self.poses[self.following]
                lead_vel = self.velocities[self.following]
                offset = self.formation[self.following] - self.formation[self.name]
                #print(lead_vel)
                #print(self.des_pos, self.pos, vel_cmd)
                ctrl_pnt = np.array([self.L*math.cos(self.eul[2]), self.L*math.sin(self.eul[2]),0])
                form_err = self.pos + ctrl_pnt - lead_pose[0:3] - offset
                vel_cmd += -self.Kp*(form_err)
                vel_cmd += -self.Kv*(self.vel_g - lead_vel)

                dist = utils.norm2d(form_err)
                dir = math.atan2(vel_cmd[1],vel_cmd[0])
                

                if(utils.norm2d(vel_cmd)>2):
                    vel_cmd = np.array([2*math.cos(dir),2*math.sin(dir),0])

                yaw_err = utils.wrapToPi(dir - self.eul[2])

                if(dist > self.wp_rad):
                    msg.angular.z = 2*yaw_err
                else:
                    msg.angular.z = 0.0

            else:
                lead_pose = self.poses[self.following]
                lead_vel = self.velocities[self.following]
                offset = self.formation[self.following] - self.formation[self.name]
                #print(lead_vel)
                #print(self.des_pos, self.pos, vel_cmd)
                form_err = self.pos + ctrl_pnt - lead_pose[0:3] - offset
                vel_cmd += -self.Kp*(form_err)
                vel_cmd += -self.Kv*(self.vel_g - lead_vel)
                dir = math.atan2(vel_cmd[1],vel_cmd[0])

                if(utils.norm2d(vel_cmd)>2):
                    vel_cmd = np.array([2*math.cos(dir),2*math.sin(dir),0])

                dist = utils.norm2d(form_err)

                if(dist > self.wp_rad):
                    yaw_err = utils.wrapToPi(dir - self.eul[2])
                else:
                    yaw_err = utils.wrapToPi(lead_pose[5] - self.eul[2])
                
                msg.angular.z = 2*yaw_err


            msg.linear.x = vel_cmd[0]*math.cos(self.eul[2]) + vel_cmd[1]*math.sin(self.eul[2])
            msg.linear.y = -vel_cmd[0]*math.sin(self.eul[2]) + vel_cmd[1]*math.cos(self.eul[2])

            self.cmd_vel_pub.publish(msg)         


    def stop(self,string):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)
        self.des_pose = []
        print("stop")
